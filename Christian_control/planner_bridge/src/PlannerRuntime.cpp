#include "PlannerRuntime.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace {

std::string Number(double value)
{
    std::ostringstream output;
    output << std::setprecision(17) << value;
    return output.str();
}

std::vector<std::string> ArgumentsForRequest(
    const PlanningRequest& request,
    const PlannerRuntimeConfig& config)
{
    const bool left = request.arm == PlanningArm::kLeft;
    std::vector<std::string> args = {
        "--arm", left ? "left" : "right",
        "--goal-file", config.goal_file,
        "--planner-config", config.planner_config_file,
        "--joint-limits", config.joint_limits_file,
        "--dh", left ? config.left_dh_file : config.right_dh_file,
        "--runs-root", config.runs_root,
        "--start-deg"};
    for (int joint = 0; joint < 7; ++joint)
        args.push_back(Number(request.q_rad(joint) * 180.0 / M_PI));

    const Eigen::Quaterniond world_q_mount(request.world_T_mount.linear());
    args.push_back("--world-mount-pose-m-quat");
    for (double value : request.world_T_mount.translation())
        args.push_back(Number(value));
    args.push_back(Number(world_q_mount.x()));
    args.push_back(Number(world_q_mount.y()));
    args.push_back(Number(world_q_mount.z()));
    args.push_back(Number(world_q_mount.w()));
    args.push_back("--vicon-sequence");
    args.push_back(std::to_string(request.vicon_sequence));
    args.push_back("--trajectory-id");
    args.push_back(std::to_string(request.request_id));
    return args;
}

}  // namespace

PlannerSolveResult SolveWorldTrajectoryForRequest(
    const PlanningRequest& request,
    const PlannerRuntimeConfig& config,
    std::ostream& diagnostics)
{
    return SolveWorldTrajectory(ArgumentsForRequest(request, config),
                                diagnostics);
}
