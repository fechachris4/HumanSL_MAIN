#include <cassert>
#include <cstdio>
#include "PlanSolver.h"

int main(int argc, char** argv) {
    assert(argc == 3 && "usage: test_plan_solver <dh_tool.yaml> <joint_limits.yaml>");
    const PlannerModel model = LoadPlannerModel(argv[1]);
    PlanRequest request;
    request.q_start_rad = Eigen::Matrix<double, 7, 1>::Zero();
    const Eigen::Vector3d start = ToolPositionInBaseLink(model, request.q_start_rad);
    request.goal_position_m = start + Eigen::Vector3d(0.15, 0.10, -0.10);
    const PlanOutcome outcome = SolveToPosition(model, request, argv[2]);
    assert(outcome.ok && "solve must succeed in an empty world");
    assert(!outcome.result.trajectory_pos.empty());
    // First support state is the start; last reaches the goal.
    assert((outcome.result.trajectory_pos.front() -
            gtsam::Vector(request.q_start_rad)).norm() < 1e-3);
    std::printf("final goal error: %.1f mm, %lld ms solve\n",
                outcome.final_goal_error_m * 1000.0,
                static_cast<long long>(outcome.result.optimization_duration.count()));
    assert(outcome.final_goal_error_m < 0.03 && "goal within 3 cm");
    return 0;
}
