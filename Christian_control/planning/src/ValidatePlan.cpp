#include "ValidatePlan.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

struct Sample { Eigen::Matrix<double, 7, 1> q, qdot; double t; };

double OrientationError(const Eigen::Matrix3d& desired,
                        const Eigen::Matrix3d& actual) {
    return Eigen::AngleAxisd(desired.transpose() * actual).angle();
}

Sample At(const TrajectoryResult& result, double duration, double t) {
    const std::size_t n = result.trajectory_pos.size();
    const double base_dt = duration / static_cast<double>(n - 1);
    if (t <= 0.0) return {result.trajectory_pos.front(), result.trajectory_vel.front(), 0.0};
    if (t >= duration) return {result.trajectory_pos.back(), result.trajectory_vel.back(), duration};
    const double x = t / base_dt;
    const std::size_t i = std::min(n - 2, static_cast<std::size_t>(std::floor(x)));
    const double u = x - static_cast<double>(i);
    const auto& p0 = result.trajectory_pos[i];
    const auto& p1 = result.trajectory_pos[i + 1];
    const auto& v0 = result.trajectory_vel[i];
    const auto& v1 = result.trajectory_vel[i + 1];
    const double u2 = u * u, u3 = u2 * u;
    const double h00 = 2*u3 - 3*u2 + 1, h10 = u3 - 2*u2 + u;
    const double h01 = -2*u3 + 3*u2, h11 = u3 - u2;
    const double d00 = 6*u2 - 6*u, d10 = 3*u2 - 4*u + 1;
    const double d01 = -6*u2 + 6*u, d11 = 3*u2 - 2*u;
    return {h00*p0 + h10*base_dt*v0 + h01*p1 + h11*base_dt*v1,
            (d00*p0 + d01*p1) / base_dt + d10*v0 + d11*v1, t};
}

void UpdateConfigurationClearance(
    const PlannerModel& model, const Eigen::Matrix<double, 7, 1>& q,
    double time_s, const std::vector<NamedObstacleField>& obstacle_fields,
    double minimum_clearance_m, PlanValidationReport& report)
{
    for (const auto& field : obstacle_fields) {
        const auto centres =
            field.participating_arm->sphereCentersMat(gtsam::Vector(q));
        if (!field.participating_sphere_indices.empty())
            report.has_scene_pairs = true;
        for (int local = 0; local < centres.cols(); ++local) {
            const Eigen::Vector3d centre(
                centres(0, local), centres(1, local), centres(2, local));
            const auto query = QueryStaticObstacle(
                field.geometry, centre,
                field.participating_arm->sphere_radius(
                    static_cast<std::size_t>(local)));
            if (query.clearance_m < report.minimum_scene_clearance_m) {
                report.minimum_scene_clearance_m = query.clearance_m;
                report.worst_scene_object_id = field.id;
                report.worst_scene_sphere_index =
                    field.participating_sphere_indices[local];
                report.worst_scene_time_s = time_s;
            }
            if (query.clearance_m < minimum_clearance_m) {
                report.scene_valid = false;
                const bool already_recorded = std::any_of(
                    report.first_scene_violations.begin(),
                    report.first_scene_violations.end(),
                    [&](const SceneViolationEvidence& violation) {
                        return violation.object_id == field.id;
                    });
                if (!already_recorded) {
                    SceneViolationEvidence violation;
                    violation.object_id = field.id;
                    violation.sphere_index =
                        field.participating_sphere_indices[local];
                    violation.time_s = time_s;
                    violation.clearance_m = query.clearance_m;
                    violation.outward_normal_mount = query.outward_normal_mount;
                    violation.q = q;
                    report.first_scene_violations.push_back(std::move(violation));
                }
            }
        }
    }

    const auto centres = model.arm_model->sphereCentersMat(gtsam::Vector(q));
    for (const auto& pair : kSelfCollisionPairs) {
        const Eigen::Vector3d a(
            centres(0, pair.first_sphere_index),
            centres(1, pair.first_sphere_index),
            centres(2, pair.first_sphere_index));
        const Eigen::Vector3d b(
            centres(0, pair.second_sphere_index),
            centres(1, pair.second_sphere_index),
            centres(2, pair.second_sphere_index));
        const double clearance =
            (a - b).norm() -
            model.arm_model->sphere_radius(pair.first_sphere_index) -
            model.arm_model->sphere_radius(pair.second_sphere_index);
        if (clearance < report.minimum_self_clearance_m) {
            report.minimum_self_clearance_m = clearance;
            report.worst_self_first_sphere = pair.first_sphere_index;
            report.worst_self_second_sphere = pair.second_sphere_index;
            report.worst_self_time_s = time_s;
        }
        if (clearance < pair.minimum_surface_clearance_m)
            report.self_collision_valid = false;
    }
}

// No checked pair means "nothing was measured", which is infinite clearance,
// not zero clearance.
void FinalizeClearance(PlanValidationReport& report) {
    if (!report.has_scene_pairs)
        report.minimum_scene_clearance_m = std::numeric_limits<double>::infinity();
    if (!report.has_self_pairs)
        report.minimum_self_clearance_m = std::numeric_limits<double>::infinity();
}

// ---- 1. Basic trajectory sanity ------------------------------------------

bool ValidTrajectoryShape(const TrajectoryResult& trajectory, double duration_s,
                          const PlanValidationInputs& inputs)
{
    if (trajectory.trajectory_pos.size() < 2 ||
        trajectory.trajectory_pos.size() != trajectory.trajectory_vel.size() ||
        !(duration_s > 0.0))
        return false;
    if (!(inputs.validation_dt_s > 0.0) ||
        inputs.effective_velocity_rad_s.minCoeff() <= 0.0 ||
        inputs.effective_acceleration_rad_s2.minCoeff() <= 0.0)
        return false;
    for (std::size_t i = 0; i < trajectory.trajectory_pos.size(); ++i) {
        if (trajectory.trajectory_pos[i].size() != 7 ||
            trajectory.trajectory_vel[i].size() != 7)
            return false;
    }
    return true;
}

std::vector<Sample> SampleTrajectory(const TrajectoryResult& trajectory,
                                     double duration_s, double dt_s)
{
    const std::size_t count = static_cast<std::size_t>(duration_s / dt_s);
    std::vector<Sample> samples;
    samples.reserve(count + 2);
    for (std::size_t i = 0; i <= count; ++i)
        samples.push_back(At(trajectory, duration_s,
                             std::min(duration_s, i * dt_s)));
    if (samples.back().t < duration_s)
        samples.push_back(At(trajectory, duration_s, duration_s));
    return samples;
}

// ---- 2. Measurements. Each fills its slice of the report; none decides. --

void MeasureStartState(const std::vector<Sample>& samples,
                       const PlanValidationInputs& inputs,
                       PlanValidationReport& report)
{
    report.start_position_error_rad =
        (samples.front().q - inputs.measured_q_rad).cwiseAbs().maxCoeff();
    report.start_valid =
        report.start_position_error_rad <= inputs.start_position_tolerance_rad;
    if (inputs.measured_qdot_rad_s) {
        report.start_velocity_error_rad_s =
            (samples.front().qdot - *inputs.measured_qdot_rad_s)
                .cwiseAbs().maxCoeff();
        report.start_valid =
            report.start_valid &&
            report.start_velocity_error_rad_s <=
                inputs.start_velocity_tolerance_rad_s;
    }
}

void MeasureJointLimits(const std::vector<Sample>& samples,
                        const PlanValidationInputs& inputs,
                        PlanValidationReport& report)
{
    report.finite = true;
    report.joint_limits_valid = true;
    for (const Sample& s : samples) {
        if (!s.q.allFinite() || !s.qdot.allFinite()) {
            report.finite = false;
            continue;
        }
        if (((s.q.array() < inputs.position_lower_rad.array()) ||
             (s.q.array() > inputs.position_upper_rad.array())).any())
            report.joint_limits_valid = false;
    }
}

void MeasureDynamics(const std::vector<Sample>& samples,
                     const PlanValidationInputs& inputs,
                     PlanValidationReport& report)
{
    Eigen::Matrix<double, 7, 1> max_qdot = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> max_qddot = Eigen::Matrix<double, 7, 1>::Zero();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Sample& s = samples[i];
        if (!s.q.allFinite() || !s.qdot.allFinite()) continue;
        max_qdot = max_qdot.cwiseMax(s.qdot.cwiseAbs());
        if (i > 0) {
            const double dt = s.t - samples[i - 1].t;
            if (dt > 0.0)
                max_qddot = max_qddot.cwiseMax(
                    ((s.qdot - samples[i - 1].qdot) / dt).cwiseAbs());
        }
    }
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].t - samples[i - 1].t;
        if (dt > 0.0)
            report.integrated_joint_travel_rad +=
                0.5 * (samples[i - 1].qdot.cwiseAbs().sum() +
                       samples[i].qdot.cwiseAbs().sum()) * dt;
    }
    report.maximum_abs_joint_velocity_rad_s = max_qdot;
    report.maximum_abs_joint_acceleration_rad_s2 = max_qddot;
    report.max_velocity_ratio =
        (max_qdot.array() / inputs.effective_velocity_rad_s.array()).maxCoeff();
    report.max_acceleration_ratio =
        (max_qddot.array() / inputs.effective_acceleration_rad_s2.array())
            .maxCoeff();
}

void MeasureClearance(const PlannerModel& model,
                      const std::vector<Sample>& samples,
                      const PlanValidationInputs& inputs,
                      PlanValidationReport& report)
{
    report.scene_valid = true;
    report.self_collision_valid = true;
    report.has_self_pairs = !kSelfCollisionPairs.empty();
    report.has_scene_pairs = false;
    report.minimum_scene_clearance_m = std::numeric_limits<double>::infinity();
    report.minimum_self_clearance_m = std::numeric_limits<double>::infinity();
    if (inputs.obstacle_fields) {
        for (const Sample& s : samples) {
            if (!s.q.allFinite()) continue;
            UpdateConfigurationClearance(model, s.q, s.t,
                                         *inputs.obstacle_fields,
                                         inputs.minimum_clearance_m, report);
        }
    }
    FinalizeClearance(report);
}

void MeasureTerminalError(const PlannerModel& model, const Sample& final_sample,
                          const PlanValidationInputs& inputs,
                          PlanValidationReport& report)
{
    const gtsam::Pose3 final_pose = ToolPoseInMount(model, final_sample.q);
    report.requested_terminal_position_error_m =
        (final_pose.translation() -
         inputs.requested_terminal_mount.translation()).norm();
    report.requested_terminal_orientation_error_rad =
        gtsam::Rot3::Logmap(inputs.requested_terminal_mount.rotation().between(
                                final_pose.rotation())).norm();
    const gtsam::Pose3& target = inputs.intended_status == PlanStatus::kGoalBlocked
                                     ? inputs.candidate_terminal_mount
                                     : inputs.requested_terminal_mount;
    report.terminal_position_error_m =
        (final_pose.translation() - target.translation()).norm();
    report.terminal_orientation_error_rad =
        gtsam::Rot3::Logmap(target.rotation().between(final_pose.rotation()))
            .norm();
}

// Task quality against the requested Cartesian path. The dense sweep runs FK
// on EVERY task-phase validation sample against the requested path
// (piecewise-linear between its samples; their chord error is bounded at
// authoring, <= 1 mm for circles) and is authoritative. The per-anchor
// statistics sample only where the pose priors act and stay diagnostic.
void MeasureTaskQuality(const PlannerModel& model,
                        const TrajectoryResult& trajectory, double duration_s,
                        const std::vector<Sample>& samples,
                        const PlanValidationInputs& inputs,
                        PlanValidationReport& report)
{
    report.task_valid =
        report.terminal_position_error_m <=
            inputs.terminal_position_tolerance_m &&
        report.terminal_orientation_error_rad <=
            inputs.terminal_orientation_tolerance_rad;
    if (!inputs.desired_task_path) return;

    if (inputs.desired_task_path->samples.size() > 1) {
        const auto& requested = inputs.desired_task_path->samples;
        const double requested_start = requested.front().t_s;
        const double requested_span = requested.back().t_s - requested_start;
        const double task_span = duration_s - inputs.task_start_time_s;
        const auto requested_deviation = [&](const Eigen::Matrix<double, 7, 1>& q,
                                             double t) {
            const double u = (t - inputs.task_start_time_s) / task_span;
            const double t_req = requested_start + u * requested_span;
            std::size_t k = 1;
            while (k + 1 < requested.size() && requested[k].t_s < t_req) ++k;
            const auto& a = requested[k - 1];
            const auto& b = requested[k];
            const double span = b.t_s - a.t_s;
            const double w = span > 0.0 ? (t_req - a.t_s) / span : 0.0;
            const Eigen::Vector3d target =
                (1.0 - w) * a.pose.translation() + w * b.pose.translation();
            return (ToolPoseInMount(model, q).translation() - target).norm();
        };
        if (task_span > 0.0) {
            for (const Sample& s : samples) {
                if (s.t < inputs.task_start_time_s) continue;
                const double error = requested_deviation(s.q, s.t);
                if (error > report.trace_dense_max_position_m) {
                    report.trace_dense_max_position_m = error;
                    report.trace_dense_worst_time_s = s.t;
                }
            }
            report.task_valid =
                report.task_valid &&
                report.trace_dense_max_position_m <=
                    inputs.path_position_tolerance_m;
        }
    }

    if (!inputs.desired_task_path->samples.empty()) {
        std::vector<double> position_errors;
        double position_error_sum = 0.0;
        const auto& requested_samples = inputs.desired_task_path->samples;
        const double requested_start = requested_samples.front().t_s;
        const double requested_end = requested_samples.back().t_s;
        const double requested_span = requested_end - requested_start;
        for (const PathSample& requested : requested_samples) {
            const double u = requested_span > 0.0
                                 ? (requested.t_s - requested_start) / requested_span
                                 : 0.0;
            const Sample sample = At(
                trajectory, duration_s,
                inputs.task_start_time_s +
                    std::clamp(u, 0.0, 1.0) *
                        (duration_s - inputs.task_start_time_s));
            const gtsam::Pose3 actual_pose = ToolPoseInMount(model, sample.q);
            Eigen::Isometry3d actual = Eigen::Isometry3d::Identity();
            actual.linear() = actual_pose.rotation().matrix();
            actual.translation() = actual_pose.translation();
            const double position_error =
                (actual.translation() - requested.pose.translation()).norm();
            position_errors.push_back(position_error);
            position_error_sum += position_error;
            if (position_error > report.trace_max_position_m) {
                report.trace_max_position_m = position_error;
                report.trace_worst_position_u = u;
            }
            report.trace_max_orientation_rad = std::max(
                report.trace_max_orientation_rad,
                OrientationError(requested.pose.linear(), actual.linear()));
        }
        if (!position_errors.empty()) {
            report.trace_mean_position_m =
                position_error_sum / static_cast<double>(position_errors.size());
            std::sort(position_errors.begin(), position_errors.end());
            double sum_sq = 0.0;
            for (double error : position_errors) sum_sq += error * error;
            report.trace_rms_position_m =
                std::sqrt(sum_sq / static_cast<double>(position_errors.size()));
            const std::size_t index = static_cast<std::size_t>(
                std::ceil(0.95 * static_cast<double>(position_errors.size())) - 1.0);
            report.trace_p95_position_m =
                position_errors[std::min(index, position_errors.size() - 1)];
        }
    }
}

// ---- 3. Classification helper --------------------------------------------

PlanValidationReport Reject(PlanValidationReport report, const char* reason) {
    report.disposition = CandidateDisposition::kInvalid;
    report.failure_reason = reason;
    return report;
}

}  // namespace

PlanValidationReport MeasureConfigurationClearance(
    const PlannerModel& model,
    const Eigen::Matrix<double, 7, 1>& q_rad,
    const std::vector<NamedObstacleField>& obstacle_fields,
    double minimum_clearance_m)
{
    PlanValidationReport report;
    report.scene_valid = true;
    report.self_collision_valid = true;
    report.has_self_pairs = !kSelfCollisionPairs.empty();
    report.minimum_scene_clearance_m = std::numeric_limits<double>::infinity();
    report.minimum_self_clearance_m = std::numeric_limits<double>::infinity();
    UpdateConfigurationClearance(model, q_rad, 0.0, obstacle_fields,
                                 minimum_clearance_m, report);
    FinalizeClearance(report);
    return report;
}

PlanValidationReport ValidatePlan(const PlannerModel& model,
                                  const TrajectoryResult& trajectory,
                                  double duration_s,
                                  const PlanValidationInputs& inputs) {
    PlanValidationReport report;

    // 1. Basic trajectory sanity.
    if (!ValidTrajectoryShape(trajectory, duration_s, inputs)) {
        report.disposition = CandidateDisposition::kInvalid;
        report.failure_reason = "malformed_trajectory";
        return report;
    }

    const auto samples =
        SampleTrajectory(trajectory, duration_s, inputs.validation_dt_s);

    // 2. Measure everything once; no measurement decides anything.
    MeasureStartState(samples, inputs, report);
    MeasureJointLimits(samples, inputs, report);
    MeasureDynamics(samples, inputs, report);
    MeasureClearance(model, samples, inputs, report);
    MeasureTerminalError(model, samples.back(), inputs, report);
    MeasureTaskQuality(model, trajectory, duration_s, samples, inputs, report);

    // 3. Safety: hard rejection. (Start velocity over the effective limit is
    // preflighted in PlanSolver before any solve; it is not re-checked here.)
    if (!report.finite)
        return Reject(std::move(report), "non_finite_trajectory");
    if (!report.start_valid)
        return Reject(std::move(report), "start_state_mismatch");
    if (!report.joint_limits_valid)
        return Reject(std::move(report), "joint_position_limits");
    if (!report.scene_valid)
        return Reject(std::move(report), "scene_clearance");
    if (!report.self_collision_valid)
        return Reject(std::move(report), "self_collision");

    // 4. Valid geometry, but too fast: fix timing, don't reject the plan.
    if (report.max_velocity_ratio > 1.0 || report.max_acceleration_ratio > 1.0) {
        report.disposition = CandidateDisposition::kNeedsLongerDuration;
        report.failure_reason = "dynamic_limits_exceeded";
        return report;
    }

    // Cartesian task accuracy (terminal error, path deviation, task_valid)
    // is MEASURED above and reported, never used as a veto: the validator
    // answers "is this safe to send", the optimiser owns task quality.
    report.disposition = CandidateDisposition::kExecutable;
    report.executable = true;
    return report;
}
