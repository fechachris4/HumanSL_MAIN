#include "PlanDebugDump.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <sys/stat.h>
#include <sys/types.h>

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

// mkdir -p for one level, which is all a --debug-dir needs. An existing
// directory is success, not an error: re-running a plan into the same
// directory is the normal way to compare two solves.
std::optional<std::string> EnsureDirectory(const std::string& directory)
{
    if (::mkdir(directory.c_str(), 0755) == 0)
        return std::nullopt;
    struct stat info {};
    if (::stat(directory.c_str(), &info) == 0 && S_ISDIR(info.st_mode))
        return std::nullopt;
    return "cannot create debug directory '" + directory + "'";
}

std::optional<std::string> OpenCsv(const std::string& directory,
                                   const std::string& name,
                                   std::ofstream& file)
{
    if (const auto error = EnsureDirectory(directory))
        return error;
    const std::string path = directory + "/" + name;
    file.open(path);
    if (!file)
        return "cannot write '" + path + "'";
    file << std::setprecision(17);
    return std::nullopt;
}

std::string CsvQuoted(const std::string& value)
{
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (const char character : value) {
        if (character == '"')
            escaped.push_back('"');
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

std::string SerializeFactorCosts(const std::map<std::string, double>& costs)
{
    std::ostringstream serialized;
    serialized << std::setprecision(17);
    bool first = true;
    for (const auto& [key, cost] : costs) {
        if (!first)
            serialized << ";";
        first = false;
        serialized << key << "=" << cost;
    }
    return serialized.str();
}

}  // namespace

std::optional<std::string> WriteJointTrajectoryCsv(
    const std::string& directory, const TrajectoryResult& trajectory)
{
    std::ofstream file;
    if (const auto error = OpenCsv(directory, "joints.csv", file))
        return error;

    file << "t_s";
    for (int joint = 1; joint <= 7; ++joint)
        file << ",q" << joint << "_deg";
    for (int joint = 1; joint <= 7; ++joint)
        file << ",qd" << joint << "_deg_s";
    file << "\n";

    for (std::size_t sample = 0; sample < trajectory.trajectory_pos.size();
         ++sample) {
        const gtsam::Vector& position = trajectory.trajectory_pos[sample];
        file << static_cast<double>(sample) * trajectory.dt;
        for (int joint = 0; joint < 7; ++joint)
            file << "," << position(joint) * kRadToDeg;
        // A trajectory may carry fewer velocity samples than positions; a
        // missing one is written as an empty field rather than a zero, so a
        // gap in the data never reads as a genuine standstill.
        if (sample < trajectory.trajectory_vel.size()) {
            const gtsam::Vector& velocity = trajectory.trajectory_vel[sample];
            for (int joint = 0; joint < 7; ++joint)
                file << "," << velocity(joint) * kRadToDeg;
        } else {
            for (int joint = 0; joint < 7; ++joint)
                file << ",";
        }
        file << "\n";
    }
    return file ? std::nullopt
                : std::optional<std::string>("failed writing joints.csv");
}

std::optional<std::string> WriteJointLimitsCsv(const std::string& directory,
                                               const PlanJointLimits& limits)
{
    std::ofstream file;
    if (const auto error = OpenCsv(directory, "joint_limits.csv", file))
        return error;

    file << "joint,lower_deg,upper_deg,hardware_velocity_deg_s,effective_velocity_deg_s,"
            "hardware_acceleration_rad_s2,effective_acceleration_rad_s2\n";
    for (int joint = 0; joint < 7; ++joint) {
        file << (joint + 1) << "," << limits.lower_rad(joint) * kRadToDeg
             << "," << limits.upper_rad(joint) * kRadToDeg
             << "," << limits.hardware_velocity_rad_s(joint) * kRadToDeg
             << "," << limits.effective_velocity_rad_s(joint) * kRadToDeg
             << "," << limits.hardware_acceleration_rad_s2(joint)
             << "," << limits.effective_acceleration_rad_s2(joint) << "\n";
    }
    return file ? std::nullopt
                : std::optional<std::string>("failed writing joint_limits.csv");
}

std::optional<std::string> WriteCandidateAttemptsCsv(
    const std::string& directory,
    const std::vector<CandidateEvidence>& attempts,
    const std::optional<std::size_t>& selected_index)
{
    std::ofstream file;
    if (const auto error = OpenCsv(directory, "candidate_attempts.csv", file))
        return error;

    file << "attempt_index,selected,exact_actual_solve_attempts,"
            "shortened_actual_solve_attempts,request_actual_solve_attempts,"
            "exact_maximum_solve_attempts,shortened_maximum_solve_attempts,"
            "request_maximum_solve_attempts,stage,terminal_kind,target_source,"
            "target_ordinal,target_fraction,target_x_mount_m,target_y_mount_m,"
            "target_z_mount_m,target_qx_mount,target_qy_mount,target_qz_mount,"
            "target_qw_mount,blocker_id,terminal_branch,"
            "terminal_ik_stream_id,terminal_ik_attempt_count,"
            "terminal_ik_attempt_index,"
            "terminal_ik_position_residual_m,terminal_ik_orientation_residual_rad,"
            "terminal_ik_legal,terminal_ik_exact,requested_position_shortfall_m,"
            "requested_orientation_shortfall_rad,orientation_tier,"
            "duration_s,scene_collision_sigma,solve_time_s,"
            "optimizer_converged,optimizer_termination,optimizer_iterations,"
            "optimizer_max_iterations,"
            "optimizer_start_total_cost,optimizer_final_total_cost,"
            "optimizer_final_factor_costs,disposition,validation_failure,"
            "executable,finite,start_valid,scene_valid,self_collision_valid,"
            "joint_limits_valid,start_position_error_rad,start_velocity_error_rad_s,"
            "capped_clearance_m";
    for (int joint = 1; joint <= 7; ++joint)
        file << ",maximum_abs_qdot_j" << joint << "_rad_s";
    for (int joint = 1; joint <= 7; ++joint)
        file << ",maximum_abs_qddot_j" << joint << "_rad_s2";
    file << ",max_velocity_ratio,max_acceleration_ratio,"
            "first_scene_violation_object_id,first_scene_violation_sphere_index,"
            "first_scene_violation_time_s,first_scene_violation_clearance_m,"
            "worst_scene_object_id,worst_scene_sphere_index,worst_scene_time_s,"
            "minimum_scene_clearance_m,worst_self_first_sphere,"
            "worst_self_second_sphere,worst_self_time_s,minimum_self_clearance_m,"
            "terminal_position_error_m,terminal_orientation_error_rad,"
            "requested_terminal_position_error_m,"
            "requested_terminal_orientation_error_rad,"
            "trace_mean_position_m,trace_rms_position_m,trace_p95_position_m,"
            "trace_max_position_m,trace_worst_position_u,"
            "trace_max_orientation_rad,integrated_joint_travel_rad\n";
    const auto solve_count = [&](PlanStatus kind) {
        return static_cast<std::size_t>(std::count_if(
            attempts.begin(), attempts.end(), [&](const CandidateEvidence& attempt) {
                return attempt.terminal_kind == kind &&
                       attempt.stage == "route";
            }));
    };
    const std::size_t exact_actual = solve_count(PlanStatus::kReached);
    const std::size_t shortened_actual = solve_count(PlanStatus::kGoalBlocked);
    for (std::size_t index = 0; index < attempts.size(); ++index) {
        const CandidateEvidence& attempt = attempts[index];
        const PlanValidationReport& validation = attempt.validation;
        file << index << "," << (selected_index && *selected_index == index ? 1 : 0)
             << "," << exact_actual
             << "," << shortened_actual
             << "," << exact_actual + shortened_actual
             << ",27,27,54"
             << "," << CsvQuoted(attempt.stage)
             << "," << PlanStatusName(attempt.terminal_kind)
             << "," << CsvQuoted(attempt.target_source)
             << "," << attempt.target_ordinal
             << "," << attempt.target_fraction
             << "," << attempt.target_position_mount_m.x()
             << "," << attempt.target_position_mount_m.y()
             << "," << attempt.target_position_mount_m.z()
             << "," << attempt.target_orientation_mount.x()
             << "," << attempt.target_orientation_mount.y()
             << "," << attempt.target_orientation_mount.z()
             << "," << attempt.target_orientation_mount.w()
             << "," << CsvQuoted(attempt.blocker_id)
             << "," << attempt.terminal_branch
             << "," << attempt.terminal_ik_stream_id
             << "," << attempt.terminal_ik_attempt_count
             << "," << attempt.terminal_ik_attempt_index
             << "," << attempt.terminal_ik_position_residual_m
             << "," << attempt.terminal_ik_orientation_residual_rad
             << "," << (attempt.terminal_ik_legal ? 1 : 0)
             << "," << (attempt.terminal_ik_exact ? 1 : 0)
             << "," << attempt.requested_position_shortfall_m
             << "," << attempt.requested_orientation_shortfall_rad
             << "," << attempt.orientation_tier
             << "," << attempt.duration_s
             << "," << attempt.scene_collision_sigma
             << "," << attempt.solve_time_s
             << "," << (attempt.optimizer_converged ? 1 : 0)
             << "," << CsvQuoted(attempt.optimizer_termination)
             << "," << attempt.optimizer_iterations
             << "," << attempt.optimizer_max_iterations
             << "," << attempt.optimizer_start_total_cost
             << "," << attempt.optimizer_final_total_cost
             << "," << CsvQuoted(SerializeFactorCosts(
                            attempt.optimizer_final_factor_costs))
             << "," << CsvQuoted(attempt.disposition)
             << "," << CsvQuoted(validation.failure_reason)
             << "," << (validation.executable ? 1 : 0)
             << "," << (validation.finite ? 1 : 0)
             << "," << (validation.start_valid ? 1 : 0)
             << "," << (validation.scene_valid ? 1 : 0)
             << "," << (validation.self_collision_valid ? 1 : 0)
             << "," << (validation.joint_limits_valid ? 1 : 0)
             << "," << validation.start_position_error_rad
             << "," << validation.start_velocity_error_rad_s
             << "," << attempt.capped_clearance_m;
        for (int joint = 0; joint < 7; ++joint)
            file << "," << validation.maximum_abs_joint_velocity_rad_s(joint);
        for (int joint = 0; joint < 7; ++joint)
            file << "," << validation.maximum_abs_joint_acceleration_rad_s2(joint);
        const SceneViolationEvidence* first_violation =
            validation.first_scene_violations.empty()
                ? nullptr
                : &validation.first_scene_violations.front();
        file << "," << validation.max_velocity_ratio
             << "," << validation.max_acceleration_ratio
             << "," << CsvQuoted(first_violation ? first_violation->object_id : "")
             << "," << (first_violation ? first_violation->sphere_index : 0)
             << "," << (first_violation ? first_violation->time_s : 0.0)
             << "," << (first_violation ? first_violation->clearance_m : 0.0)
             << "," << CsvQuoted(validation.worst_scene_object_id)
             << "," << validation.worst_scene_sphere_index
             << "," << validation.worst_scene_time_s
             << "," << validation.minimum_scene_clearance_m
             << "," << validation.worst_self_first_sphere
             << "," << validation.worst_self_second_sphere
             << "," << validation.worst_self_time_s
             << "," << validation.minimum_self_clearance_m
             << "," << validation.terminal_position_error_m
             << "," << validation.terminal_orientation_error_rad
             << "," << validation.requested_terminal_position_error_m
             << "," << validation.requested_terminal_orientation_error_rad
             << "," << validation.trace_mean_position_m
             << "," << validation.trace_rms_position_m
             << "," << validation.trace_p95_position_m
             << "," << validation.trace_max_position_m
             << "," << validation.trace_worst_position_u
             << "," << validation.trace_max_orientation_rad
             << "," << validation.integrated_joint_travel_rad << "\n";
    }
    return file ? std::nullopt
                : std::optional<std::string>(
                      "failed writing candidate_attempts.csv");
}

namespace {

const char* StatusText(const PathIkSample& sample)
{
    if (sample.solved) return "solved";
    if (sample.interpolated) return "interpolated_seed";
    switch (sample.failure) {
    case PathIkFailure::kJointLimits: return "joint_limits";
    case PathIkFailure::kNoConvergence: return "no_convergence";
    case PathIkFailure::kNone: break;
    }
    return "failed";
}

}  // namespace

double JointLimitMarginRad(const Eigen::Matrix<double, 7, 1>& q_rad,
                           const PlanJointLimits& limits)
{
    double margin = std::numeric_limits<double>::infinity();
    for (int joint = 0; joint < 7; ++joint) {
        const double lower = limits.lower_rad(joint);
        const double upper = limits.upper_rad(joint);
        if (lower < -1e10 || upper > 1e10) continue;  // continuous joint
        margin = std::min({margin, q_rad(joint) - lower, upper - q_rad(joint)});
    }
    return margin;
}

std::string DescribeFailedRanges(const PathIkResult& walk)
{
    // "Failed" means an anchor whose bounded solve found nothing — never a
    // sample that was deliberately interpolated (failure == kNone there).
    const auto failed = [&](std::size_t index) {
        return !walk.samples[index].solved &&
               walk.samples[index].failure != PathIkFailure::kNone;
    };
    std::ostringstream text;
    bool first = true;
    for (std::size_t index = 0; index < walk.samples.size();) {
        if (!failed(index)) { ++index; continue; }
        std::size_t last = index;
        while (last + 1 < walk.samples.size() && failed(last + 1))
            ++last;
        if (!first) text << ", ";
        first = false;
        if (last == index) text << index;
        else text << index << "-" << last;
        index = last + 1;
    }
    return text.str();
}

std::optional<std::string> WritePathIkCsv(const std::string& directory,
                                          const CartesianPath& path_mount,
                                          const PathIkResult& walk,
                                          const PlanJointLimits& limits)
{
    std::ofstream file;
    if (const auto error = OpenCsv(directory, "path_ik.csv", file))
        return error;

    file << "sample,progress_pct,t_s,target_x_m,target_y_m,target_z_m,solved,"
            "status,position_residual_m,orientation_residual_rad,"
            "limit_margin_deg";
    for (int joint = 1; joint <= 7; ++joint)
        file << ",q" << joint << "_deg";
    file << "\n";

    const double count = walk.samples.size() > 1
                             ? static_cast<double>(walk.samples.size() - 1)
                             : 1.0;
    for (std::size_t sample = 0; sample < walk.samples.size(); ++sample) {
        const PathIkSample& solution = walk.samples[sample];
        file << sample << ","
             << 100.0 * static_cast<double>(sample) / count;
        // The walk and the path are the same length by construction, but a
        // truncated walk must not index past the path.
        if (sample < path_mount.samples.size()) {
            const PathSample& target = path_mount.samples[sample];
            const Eigen::Vector3d position = target.pose.translation();
            file << "," << target.t_s << "," << position.x() << ","
                 << position.y() << "," << position.z();
        } else {
            file << ",,,,";
        }
        const double margin = JointLimitMarginRad(solution.configuration, limits);
        file << "," << (solution.solved ? 1 : 0) << "," << StatusText(solution)
             << "," << solution.position_residual_m << ","
             << solution.orientation_residual_rad << ","
             << margin * kRadToDeg;
        for (int joint = 0; joint < 7; ++joint)
            file << "," << solution.configuration(joint) * kRadToDeg;
        file << "\n";
    }
    return file ? std::nullopt
                : std::optional<std::string>("failed writing path_ik.csv");
}

std::optional<std::string> WritePlanMetaCsv(const std::string& directory,
                                            const PlanDebugMeta& meta)
{
    std::ofstream file;
    if (const auto error = OpenCsv(directory, "meta.csv", file))
        return error;

    file << "key,value\n"
         << "arm,\"" << meta.arm << "\"\n"
         << "plan_kind,\"" << meta.plan_kind << "\"\n"
         << "status,\"" << PlanStatusName(meta.status) << "\"\n"
         << "failure_reason,\"" << meta.failure_reason << "\"\n"
         << "final_goal_error_m," << meta.final_goal_error_m << "\n"
         << "total_time_s," << meta.total_time_s << "\n";
    for (const auto& [key, value] : meta.extra)
        file << key << ",\"" << value << "\"\n";
    return file ? std::nullopt
                : std::optional<std::string>("failed writing meta.csv");
}
