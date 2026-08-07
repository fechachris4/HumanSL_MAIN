// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <cstdio>
#include "PinocchioKinematicsAdapter.h"
#include "PlanSolver.h"

int main(int argc, char** argv) {
    assert(argc == 4 &&
           "usage: test_plan_solver <dh_tool.yaml> <joint_limits.yaml> <planner.yaml>");
    const PlannerModel model = LoadPlannerModel(argv[1]);
    // The checked-in tuning, not a hand-built one: this test then covers
    // the file the real runs use, so a config edit that breaks solving is
    // caught here rather than on hardware.
    const PlannerConfig config = LoadPlannerConfig(argv[3]);
    PlanRequest request;
    request.q_start_rad = Eigen::Matrix<double, 7, 1>::Zero();
    const Eigen::Vector3d start = ToolPositionInMount(model, request.q_start_rad);
    // (0.15, 0.10, -0.10) was chosen in base_link and is proven solvable
    // there (test_bridge_main cites it). The planner now works in mount, so
    // the offset is rotated by the mounting transform to keep the PHYSICAL
    // displacement identical — writing the same triple in mount would be a
    // different move, rotated ~69 deg, and would quietly invalidate that
    // provenance while still compiling and probably still passing.
    const Eigen::Vector3d offset_base(0.15, 0.10, -0.10);
    request.goal_position_m =
        start + pinocchio_kinematics_adapter::MountFromBase(false).linear() * offset_base;
    const PlanOutcome outcome = SolveToPosition(model, request, argv[2], config);
    assert(outcome.ok && "solve must succeed in an empty world");
    assert(!outcome.result.trajectory_pos.empty());
    // First support state is the start; last reaches the goal.
    assert((outcome.result.trajectory_pos.front() -
            gtsam::Vector(request.q_start_rad)).norm() < 1e-3);
    // Planned duration is reported because it is what the controller
    // follows, and it is set from planner.yaml's motion.* — so this line
    // also shows the loaded tuning reaching the solve.
    std::printf("final goal error: %.1f mm, %.2f s planned, %lld ms solve\n",
                outcome.final_goal_error_m * 1000.0, outcome.total_time_sec,
                static_cast<long long>(outcome.result.optimization_duration.count()));
    assert(outcome.final_goal_error_m < 0.03 && "goal within 3 cm");
    return 0;
}
