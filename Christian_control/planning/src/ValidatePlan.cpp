#include "ValidatePlan.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kExactStartToleranceRad = 1e-12;

struct Sample { Eigen::Matrix<double, 7, 1> q, qdot; double t; };

Eigen::Isometry3d DesiredAt(const CartesianPath& path, double t) {
    if (path.samples.empty()) return Eigen::Isometry3d::Identity();
    if (t <= path.samples.front().t_s) return path.samples.front().pose;
    if (t >= path.samples.back().t_s) return path.samples.back().pose;
    std::size_t upper = 1;
    while (upper < path.samples.size() && path.samples[upper].t_s < t) ++upper;
    const PathSample& a = path.samples[upper - 1];
    const PathSample& b = path.samples[upper];
    const double u = (t - a.t_s) / (b.t_s - a.t_s);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = (1.0 - u) * a.pose.translation() + u * b.pose.translation();
    pose.linear() = Eigen::Quaterniond(a.pose.linear())
                        .slerp(u, Eigen::Quaterniond(b.pose.linear()))
                        .toRotationMatrix();
    return pose;
}

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
}

PlanValidationReport ValidatePlan(const PlannerModel& model,
                                  const TrajectoryResult& trajectory,
                                  double duration_s,
                                  const PlanValidationInputs& inputs) {
    PlanValidationReport report;
    if (trajectory.trajectory_pos.size() < 2 ||
        trajectory.trajectory_pos.size() != trajectory.trajectory_vel.size() ||
        !(duration_s > 0.0)) {
        report.failure_reason = "invalid trajectory dimensions or duration";
        return report;
    }
    if (!(inputs.validation_dt_s > 0.0) ||
        inputs.effective_velocity_rad_s.minCoeff() <= 0.0 ||
        inputs.effective_acceleration_rad_s2.minCoeff() <= 0.0) {
        report.failure_reason = "invalid validation step or dynamic limits";
        return report;
    }
    for (std::size_t i = 0; i < trajectory.trajectory_pos.size(); ++i) {
        if (trajectory.trajectory_pos[i].size() != 7 ||
            trajectory.trajectory_vel[i].size() != 7) {
            report.failure_reason = "trajectory state is not seven-dimensional";
            return report;
        }
    }
    const std::size_t count = static_cast<std::size_t>(duration_s / inputs.validation_dt_s);
    std::vector<Sample> samples;
    for (std::size_t i = 0; i <= count; ++i)
        samples.push_back(At(trajectory, duration_s,
                             std::min(duration_s, i * inputs.validation_dt_s)));
    if (samples.back().t < duration_s) samples.push_back(At(trajectory, duration_s, duration_s));
    report.finite = true;
    report.scene_valid = true;
    report.self_collision_valid = true;
    report.has_self_pairs = !kSelfCollisionPairs.empty();
    report.has_scene_pairs = inputs.obstacle_fields && !inputs.obstacle_fields->empty();
    report.joint_limits_valid = true;
    report.minimum_scene_clearance_m = std::numeric_limits<double>::infinity();
    report.minimum_self_clearance_m = std::numeric_limits<double>::infinity();
    Eigen::Matrix<double, 7, 1> max_qdot = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> max_qddot = Eigen::Matrix<double, 7, 1>::Zero();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Sample& s = samples[i];
        if (!s.q.allFinite() || !s.qdot.allFinite()) { report.finite = false; continue; }
        if (((s.q.array() < inputs.position_lower_rad.array()) ||
             (s.q.array() > inputs.position_upper_rad.array())).any())
            report.joint_limits_valid = false;
        max_qdot = max_qdot.cwiseMax(s.qdot.cwiseAbs());
        if (i > 0) {
            const double dt = s.t - samples[i - 1].t;
            if (dt > 0.0) max_qddot = max_qddot.cwiseMax(
                ((s.qdot - samples[i - 1].qdot) / dt).cwiseAbs());
        }
        if (inputs.obstacle_fields) {
            for (const auto& field : *inputs.obstacle_fields) {
                const auto centres = field.participating_arm->sphereCentersMat(gtsam::Vector(s.q));
                for (int local = 0; local < centres.cols(); ++local) {
                    const Eigen::Vector3d centre(centres(0, local), centres(1, local), centres(2, local));
                    const auto query = QueryStaticObstacle(field.geometry, centre,
                        field.participating_arm->sphere_radius(static_cast<std::size_t>(local)));
                    if (query.clearance_m < report.minimum_scene_clearance_m) {
                        report.minimum_scene_clearance_m = query.clearance_m;
                        report.worst_scene_object_id = field.id;
                        report.worst_scene_sphere_index = field.participating_sphere_indices[local];
                        report.worst_scene_time_s = s.t;
                    }
                }
            }
        }
        const auto centres = model.arm_model->sphereCentersMat(gtsam::Vector(s.q));
        for (const auto& pair : kSelfCollisionPairs) {
            const Eigen::Vector3d a(centres(0, pair.first_sphere_index), centres(1, pair.first_sphere_index), centres(2, pair.first_sphere_index));
            const Eigen::Vector3d b(centres(0, pair.second_sphere_index), centres(1, pair.second_sphere_index), centres(2, pair.second_sphere_index));
            const double clearance = (a - b).norm() -
                model.arm_model->sphere_radius(pair.first_sphere_index) -
                model.arm_model->sphere_radius(pair.second_sphere_index);
            if (clearance < report.minimum_self_clearance_m) {
                report.minimum_self_clearance_m = clearance;
                report.worst_self_first_sphere = pair.first_sphere_index;
                report.worst_self_second_sphere = pair.second_sphere_index;
                report.worst_self_time_s = s.t;
            }
            if (clearance < pair.minimum_surface_clearance_m)
                report.self_collision_valid = false;
        }
    }
    if (!std::isfinite(report.minimum_scene_clearance_m)) report.minimum_scene_clearance_m = 0.0;
    if (!std::isfinite(report.minimum_self_clearance_m)) report.minimum_self_clearance_m = 0.0;
    report.start_position_error_rad = (samples.front().q - inputs.measured_q_rad).cwiseAbs().maxCoeff();
    report.start_valid = report.start_position_error_rad <= kExactStartToleranceRad;
    if (inputs.measured_qdot_rad_s) {
        report.start_velocity_error_rad_s = (samples.front().qdot - *inputs.measured_qdot_rad_s).cwiseAbs().maxCoeff();
        report.start_valid = report.start_valid && report.start_velocity_error_rad_s <= kExactStartToleranceRad;
        if (((*inputs.measured_qdot_rad_s).cwiseAbs().array() >
             inputs.effective_velocity_rad_s.array()).any()) {
            report.failure_reason = "start_velocity_over_effective_limit";
            report.disposition = CandidateDisposition::kInvalid;
            return report;
        }
    }
    report.max_velocity_ratio = (max_qdot.array() / inputs.effective_velocity_rad_s.array()).maxCoeff();
    report.max_acceleration_ratio = (max_qddot.array() / inputs.effective_acceleration_rad_s2.array()).maxCoeff();
    report.scene_valid = !inputs.obstacle_fields || inputs.obstacle_fields->empty() ||
                         report.minimum_scene_clearance_m >= inputs.minimum_clearance_m;
    const gtsam::Pose3 final_pose = ToolPoseInMount(model, samples.back().q);
    report.requested_terminal_position_error_m =
        (final_pose.translation() - inputs.requested_terminal_mount.translation()).norm();
    report.requested_terminal_orientation_error_rad =
        gtsam::Rot3::Logmap(inputs.requested_terminal_mount.rotation().between(final_pose.rotation())).norm();
    const gtsam::Pose3& target = inputs.intended_status == PlanStatus::kGoalBlocked
                                     ? inputs.candidate_terminal_mount
                                     : inputs.requested_terminal_mount;
    report.terminal_position_error_m = (final_pose.translation() - target.translation()).norm();
    report.terminal_orientation_error_rad =
        gtsam::Rot3::Logmap(target.rotation().between(final_pose.rotation())).norm();
    report.terminal_position_shortfall_m = report.requested_terminal_position_error_m;
    report.terminal_orientation_shortfall_rad = report.requested_terminal_orientation_error_rad;
    if (inputs.desired_task_path && !inputs.desired_task_path->samples.empty()) {
        std::vector<double> position_errors;
        for (const Sample& sample : samples) {
            if (sample.t < inputs.task_start_time_s) continue;
            const Eigen::Isometry3d desired = DesiredAt(
                *inputs.desired_task_path,
                sample.t - inputs.task_start_time_s);
            const gtsam::Pose3 actual_pose = ToolPoseInMount(model, sample.q);
            Eigen::Isometry3d actual = Eigen::Isometry3d::Identity();
            actual.linear() = actual_pose.rotation().matrix();
            actual.translation() = actual_pose.translation();
            const double position_error =
                (actual.translation() - desired.translation()).norm();
            position_errors.push_back(position_error);
            report.trace_max_position_m =
                std::max(report.trace_max_position_m, position_error);
            report.trace_max_orientation_rad = std::max(
                report.trace_max_orientation_rad,
                OrientationError(desired.linear(), actual.linear()));
        }
        if (!position_errors.empty()) {
            std::sort(position_errors.begin(), position_errors.end());
            double sum_sq = 0.0;
            for (double error : position_errors) sum_sq += error * error;
            report.trace_rms_position_m =
                std::sqrt(sum_sq / static_cast<double>(position_errors.size()));
            const std::size_t index = static_cast<std::size_t>(
                std::ceil(0.95 * static_cast<double>(position_errors.size())) - 1.0);
            report.trace_p95_position_m = position_errors[std::min(index, position_errors.size() - 1)];
        }
    }
    const bool terminal_valid = report.terminal_position_error_m <= 0.001 &&
                                report.terminal_orientation_error_rad <= 0.01;
    const bool non_dynamic_valid = report.finite && report.start_valid &&
        report.scene_valid && report.self_collision_valid &&
        report.joint_limits_valid && terminal_valid;
    if (!non_dynamic_valid) {
        report.disposition = CandidateDisposition::kInvalid;
        if (!report.finite) report.failure_reason = "non_finite_trajectory";
        else if (!report.start_valid) report.failure_reason = "start_state_mismatch";
        else if (!report.scene_valid) report.failure_reason = "scene_clearance";
        else if (!report.self_collision_valid) report.failure_reason = "self_collision";
        else if (!report.joint_limits_valid) report.failure_reason = "joint_position_limits";
        else report.failure_reason = "terminal_pose_error";
    } else if (report.max_velocity_ratio > 1.0 || report.max_acceleration_ratio > 1.0) {
        report.disposition = CandidateDisposition::kNeedsLongerDuration;
        report.failure_reason = "dynamic_limits_exceeded";
    } else {
        report.disposition = CandidateDisposition::kExecutable;
        report.executable = true;
    }
    return report;
}
