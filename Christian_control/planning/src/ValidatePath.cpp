#include "ValidatePath.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <functional>

#include "analytical_ik.h"  // KinovaGen3Params joint limits

namespace {

// Pose of the tool at a configuration, in the frame the planner works in.
Eigen::Isometry3d ToolPose(const PlannerModel& model,
                           const Eigen::Matrix<double, 7, 1>& q) {
    const gtsam::Pose3 pose = ToolPoseInMount(model, q);
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear() = pose.rotation().matrix();
    out.translation() = pose.translation();
    return out;
}

// Angle between two orientations, radians.
double OrientationDistance(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    const Eigen::AngleAxisd difference(a.transpose() * b);
    return std::abs(difference.angle());
}

// The requested pose at a path parameter, by linear interpolation in
// position and slerp in orientation. The desired path is sampled far more
// coarsely than the validation rate, so a nearest-sample comparison would
// measure the sampling, not the tracking.
Eigen::Isometry3d DesiredPoseAt(const CartesianPath& path, double t_s) {
    if (path.samples.empty()) return Eigen::Isometry3d::Identity();
    if (t_s <= path.samples.front().t_s) return path.samples.front().pose;
    if (t_s >= path.samples.back().t_s) return path.samples.back().pose;
    std::size_t hi = 1;
    while (hi < path.samples.size() && path.samples[hi].t_s < t_s) ++hi;
    const PathSample& a = path.samples[hi - 1];
    const PathSample& b = path.samples[hi];
    const double span = b.t_s - a.t_s;
    const double u = span > 0.0 ? (t_s - a.t_s) / span : 0.0;

    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = a.pose.translation() + u * (b.pose.translation() - a.pose.translation());
    out.linear() = Eigen::Quaterniond(a.pose.linear())
                       .slerp(u, Eigen::Quaterniond(b.pose.linear()))
                       .toRotationMatrix();
    return out;
}

double Percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(
        std::min<double>(values.size() - 1,
                         std::floor(fraction * (values.size() - 1) + 0.5)));
    return values[index];
}

}  // namespace

PathValidationReport ValidatePlannedPath(
    const PlannerModel& model,
    const TimedJointTrajectory& trajectory,
    const std::vector<gtsam::Vector>& gp_dense,
    double gp_dense_duration_s,
    const TimedJointSampler& sample_at,
    const gpmp2::SignedDistanceField& sdf,
    const std::string& sdf_contents,
    const ValidationInputs& inputs,
    bool optimiser_converged) {

    PathValidationReport report;
    report.optimiser_converged = optimiser_converged;
    report.sdf_contents = sdf_contents;

    if (!trajectory.valid || trajectory.samples.empty() ||
        !inputs.desired_task_path)
        return report;

    const double dt = inputs.validation_dt_s > 0.0 ? inputs.validation_dt_s : 0.002;

    // ---- dense sweep over the final internal joint trajectory -----------
    std::vector<double> command_position_errors;
    double worst_command = -1.0;
    double min_clearance = std::numeric_limits<double>::infinity();
    double min_clearance_t = 0.0;
    double max_velocity = 0.0, max_acceleration = 0.0;
    // Per-joint maxima for the dynamic verdict: limits differ per joint
    // (76 deg/s for 1-4, 66.5 deg/s for 5-7), so each joint must be
    // checked against its own limit — comparing the overall max against
    // the largest limit would let joints 5-7 exceed theirs unnoticed.
    Eigen::Matrix<double, 7, 1> max_velocity_per_joint =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> max_acceleration_per_joint =
        Eigen::Matrix<double, 7, 1>::Zero();
    double min_limit_margin = std::numeric_limits<double>::infinity();
    bool finite = true;
    Eigen::Matrix<double, 7, 1> previous_velocity = Eigen::Matrix<double, 7, 1>::Zero();
    bool have_previous = false;

    for (const TimedJointSample& sample : trajectory.samples) {
        const double t = sample.t_s;
        if (!sample.q_rad.allFinite() || !sample.qdot_rad_s.allFinite()) {
            finite = false;
            continue;
        }

        max_velocity = std::max(max_velocity, sample.qdot_rad_s.cwiseAbs().maxCoeff());
        max_velocity_per_joint =
            max_velocity_per_joint.cwiseMax(sample.qdot_rad_s.cwiseAbs());
        if (have_previous) {
            const Eigen::Matrix<double, 7, 1> acceleration =
                (sample.qdot_rad_s - previous_velocity) / dt;
            max_acceleration =
                std::max(max_acceleration, acceleration.cwiseAbs().maxCoeff());
            max_acceleration_per_joint =
                max_acceleration_per_joint.cwiseMax(acceleration.cwiseAbs());
        }
        previous_velocity = sample.qdot_rad_s;
        have_previous = true;

        // Joint-limit margin over BOUNDED joints only; 1/3/5/7 are
        // continuous on a Gen3 and carry sentinel limits.
        for (int j = 0; j < 7; ++j) {
            const double lo = analytical_ik::KinovaGen3Params::JOINT_LIMITS_LOWER[j];
            const double hi = analytical_ik::KinovaGen3Params::JOINT_LIMITS_UPPER[j];
            if (lo < -1e10 || hi > 1e10) continue;
            const double angle = std::remainder(sample.q_rad(j), 2.0 * M_PI);
            min_limit_margin =
                std::min(min_limit_margin, std::min(angle - lo, hi - angle));
        }

        // Clearance: every collision sphere against the SDF. Reported as
        // distance from the sphere SURFACE, so negative means penetrating.
        const gtsam::Matrix centres =
            model.arm_model->sphereCentersMat(gtsam::Vector(sample.q_rad));
        for (int s = 0; s < centres.cols(); ++s) {
            const gtsam::Point3 centre(centres(0, s), centres(1, s), centres(2, s));
            double distance = 0.0;
            try {
                distance = sdf.getSignedDistance(centre);
            } catch (const std::exception&) {
                // Outside the grid: gpmp2 cannot answer, and treating an
                // unanswerable query as clear would be the silent failure
                // this whole report exists to prevent.
                distance = std::numeric_limits<double>::quiet_NaN();
            }
            const double clearance =
                distance - model.arm_model->sphere_radius(static_cast<size_t>(s));
            if (std::isnan(clearance)) {
                finite = false;
                continue;
            }
            if (clearance < min_clearance) {
                min_clearance = clearance;
                min_clearance_t = t;
            }
        }

        // Fidelity over the TRACED phase only — the approach is
        // unconstrained and has no requested geometry to deviate from.
        if (t + 1e-12 < inputs.task_start_time_s) continue;
        const double path_t = t - inputs.task_start_time_s;
        const Eigen::Isometry3d desired = DesiredPoseAt(*inputs.desired_task_path, path_t);
        const Eigen::Isometry3d actual = ToolPose(model, sample.q_rad);

        const double position_error =
            (actual.translation() - desired.translation()).norm();
        const double orientation_error =
            OrientationDistance(desired.linear(), actual.linear());
        command_position_errors.push_back(position_error);
        report.command.max_orientation_rad =
            std::max(report.command.max_orientation_rad, orientation_error);
        if (position_error > worst_command) {
            worst_command = position_error;
            report.command.worst_time_s = t;
            report.command.worst_path_parameter =
                inputs.desired_task_path->duration_s() > 0.0
                    ? path_t / inputs.desired_task_path->duration_s()
                    : 0.0;
        }

        if (inputs.circle_applicable) {
            report.command_circle.applicable = true;
            const Eigen::Vector3d n = inputs.circle_normal.normalized();
            const Eigen::Vector3d offset = actual.translation() - inputs.circle_centre;
            report.command_circle.max_plane_error_m = std::max(
                report.command_circle.max_plane_error_m, std::abs(offset.dot(n)));
            const double in_plane = (offset - offset.dot(n) * n).norm();
            report.command_circle.max_radial_error_m =
                std::max(report.command_circle.max_radial_error_m,
                         std::abs(in_plane - inputs.circle_radius_m));
        }
    }

    // ---- the GP-dense trajectory, for attribution ----------------------
    // Same measurement against the same reference, so a fidelity failure
    // can be attributed to the optimiser or to the wire transport.
    std::vector<double> planner_errors, transport_errors;
    if (gp_dense.size() >= 2 && gp_dense_duration_s > 0.0) {
        const double gp_dt =
            gp_dense_duration_s / static_cast<double>(gp_dense.size() - 1);
        for (std::size_t i = 0; i < gp_dense.size(); ++i) {
            const double t = static_cast<double>(i) * gp_dt;
            if (t + 1e-12 < inputs.task_start_time_s) continue;
            if (gp_dense[i].size() != 7 || !gp_dense[i].allFinite()) {
                finite = false;
                continue;
            }
            const Eigen::Matrix<double, 7, 1> gp_q = gp_dense[i];
            const Eigen::Isometry3d gp_pose = ToolPose(model, gp_q);
            const Eigen::Isometry3d desired =
                DesiredPoseAt(*inputs.desired_task_path, t - inputs.task_start_time_s);
            planner_errors.push_back(
                (gp_pose.translation() - desired.translation()).norm());
            report.planner.max_orientation_rad =
                std::max(report.planner.max_orientation_rad,
                         OrientationDistance(desired.linear(), gp_pose.linear()));

            // Projection input fidelity: same instant, dense GP versus the
            // trajectory sampler that feeds world-Cartesian projection.
            const TimedJointSample at = sample_at(t);
            const Eigen::Isometry3d reconstructed_pose = ToolPose(model, at.q_rad);
            transport_errors.push_back(
                (reconstructed_pose.translation() - gp_pose.translation()).norm());
            report.reconstruction.max_orientation_rad =
                std::max(report.reconstruction.max_orientation_rad,
                         OrientationDistance(gp_pose.linear(), reconstructed_pose.linear()));
        }
    }

    const auto fill = [](FidelityError& error, const std::vector<double>& values) {
        if (values.empty()) return;
        error.max_position_m = *std::max_element(values.begin(), values.end());
        double sum_squares = 0.0;
        for (double v : values) sum_squares += v * v;
        error.rms_position_m = std::sqrt(sum_squares / static_cast<double>(values.size()));
        error.p95_position_m = Percentile(values, 0.95);
    };
    fill(report.command, command_position_errors);
    fill(report.planner, planner_errors);
    fill(report.reconstruction, transport_errors);

    // ---- start state ----------------------------------------------------
    const TimedJointSample& first = trajectory.samples.front();
    report.start_configuration_error_rad =
        (first.q_rad - inputs.measured_start).cwiseAbs().maxCoeff();
    report.start_velocity_rad_s = first.qdot_rad_s.cwiseAbs().maxCoeff();
    report.all_finite = finite;
    report.start_state_valid =
        finite && report.start_configuration_error_rad <= inputs.start_tolerance_rad;

    // ---- verdicts -------------------------------------------------------
    report.minimum_clearance_m =
        std::isfinite(min_clearance) ? min_clearance : 0.0;
    report.minimum_clearance_time_s = min_clearance_t;
    report.modelled_collision_valid = finite && std::isfinite(min_clearance) &&
                                      min_clearance > 0.0;

    report.max_joint_velocity_rad_s = max_velocity;
    report.max_joint_acceleration_rad_s2 = max_acceleration;
    report.minimum_joint_limit_margin_rad =
        std::isfinite(min_limit_margin) ? min_limit_margin : 0.0;
    report.joint_limits_valid = report.minimum_joint_limit_margin_rad > 0.0;
    // Each joint against ITS OWN limit (DynamicLimitsValid,
    // PathValidationReport.h). The previous max-vs-max comparison was
    // equivalent only while every joint shared one limit; with the
    // 76/66.5 deg/s split it would have passed a joint 5-7 running up to
    // the joint 1-4 limit.
    report.dynamic_limits_valid = DynamicLimitsValid(
        max_velocity_per_joint, max_acceleration_per_joint,
        inputs.joint_velocity_limits_rad_s,
        inputs.joint_acceleration_limits_rad_s2);

    report.task_fidelity_valid =
        report.command.max_position_m <= inputs.maximum_planning_error_m &&
        report.command.max_orientation_rad <= inputs.maximum_orientation_error_rad;

    report.hardware_execution_allowed =
        report.optimiser_converged && report.task_fidelity_valid &&
        report.modelled_collision_valid && report.joint_limits_valid &&
        report.dynamic_limits_valid && report.start_state_valid;
    return report;
}
