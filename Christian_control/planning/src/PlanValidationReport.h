#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

enum class PlanStatus { kReached, kGoalBlocked, kFailed };
// kInvalid            unsafe / malformed / impossible to execute
// kNeedsLongerDuration geometry is safe, timing exceeds robot limits
// kExecutable          send it
// Task quality (terminal error, path deviation) is measured and reported
// but never a disposition: the validator gates safety, not perfection.
enum class CandidateDisposition {
    kInvalid,
    kNeedsLongerDuration,
    kExecutable
};

const char* PlanStatusName(PlanStatus status);
bool IsExecutable(PlanStatus status);

struct SceneViolationEvidence {
    std::string object_id;
    std::size_t sphere_index = 0;
    double time_s = 0.0;
    double clearance_m = 0.0;
    Eigen::Vector3d outward_normal_mount = Eigen::Vector3d::UnitX();
    Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
};

struct PlanValidationReport {
    bool executable = false;
    CandidateDisposition disposition = CandidateDisposition::kInvalid;
    std::string failure_reason;
    bool finite = false;
    bool start_valid = false;
    bool scene_valid = false;
    bool self_collision_valid = false;
    bool has_scene_pairs = false;
    bool has_self_pairs = false;
    bool joint_limits_valid = false;
    double start_position_error_rad = 0.0;
    double start_velocity_error_rad_s = 0.0;
    double minimum_scene_clearance_m = 0.0;
    std::vector<SceneViolationEvidence> first_scene_violations;
    std::string worst_scene_object_id;
    std::size_t worst_scene_sphere_index = 0;
    double worst_scene_time_s = 0.0;
    double minimum_self_clearance_m = 0.0;
    std::size_t worst_self_first_sphere = 0;
    std::size_t worst_self_second_sphere = 0;
    double worst_self_time_s = 0.0;
    Eigen::Matrix<double, 7, 1> maximum_abs_joint_velocity_rad_s =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> maximum_abs_joint_acceleration_rad_s2 =
        Eigen::Matrix<double, 7, 1>::Zero();
    double max_velocity_ratio = 0.0;
    double max_acceleration_ratio = 0.0;
    double terminal_position_error_m = 0.0;
    double terminal_orientation_error_rad = 0.0;
    double requested_terminal_position_error_m = 0.0;
    double requested_terminal_orientation_error_rad = 0.0;
    double trace_mean_position_m = 0.0;
    double trace_rms_position_m = 0.0;
    double trace_max_position_m = 0.0;
    double trace_p95_position_m = 0.0;
    double trace_worst_position_u = 0.0;
    double trace_max_orientation_rad = 0.0;
    // Dense executed-path fidelity: FK of EVERY validation sample in the
    // task phase against the requested path (piecewise-linear between its
    // samples). The anchor metrics above sample only where the pose priors
    // act and are DIAGNOSTIC; the dense figure is authoritative — a planned
    // Cartesian path means the continuously executed FK trajectory stays
    // within tolerance of that path, not merely intersects it at waypoints
    // (2026-08-23: 0.15 mm certified at anchors, 91.5 mm executed between).
    double trace_dense_max_position_m = 0.0;
    double trace_dense_worst_time_s = 0.0;
    // Quality flag, REPORT-ONLY (never gates execution): terminal pose
    // within tolerance and, for a traced path, the dense executed path
    // within tolerance of the requested path.
    bool task_valid = false;
    double integrated_joint_travel_rad = 0.0;
};
