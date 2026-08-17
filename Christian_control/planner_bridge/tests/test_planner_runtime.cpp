// Direct typed planner-result characterization. This test is deliberately
// written before the runtime API exists; the first build must fail because
// the typed entry point has not yet been added.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include "PlannerRuntime.h"

int main(int argc, char** argv)
{
    assert(argc == 5 &&
           "usage: test_planner_runtime <planner> <limits> <dh_tool> <dh_flange>");

    const std::filesystem::path goal_path =
        std::filesystem::current_path() / "planner_runtime_goal.yaml";
    {
        std::ofstream goal(goal_path);
        goal << "right:\n"
             << "  frame: mount\n"
             << "  goal: [0.15, -1.158774, 0.497919]\n";
    }

    PlanningRequest request;
    request.request_id = 91;
    request.arm = PlanningArm::kRight;
    request.vicon_sequence = 42;
    request.vicon_frame_number = 7;
    request.world_T_mount = Eigen::Isometry3d::Identity();
    request.q_rad.setZero();

    PlannerRuntimeConfig config;
    config.goal_file = goal_path.string();
    config.planner_config_file = argv[1];
    config.joint_limits_file = argv[2];
    config.right_dh_file = argv[3];
    config.left_dh_file = argv[4];
    config.runs_root = ".";

    std::ostringstream diagnostics;
    PlannerSolveResult result =
        SolveWorldTrajectoryForRequest(request, config, diagnostics);
    assert(result.exit_code == 0);
    assert(result.trajectory);
    const WorldCartesianTrajectory& trajectory = *result.trajectory;
    assert(trajectory.trajectory_id == request.request_id);
    assert(trajectory.planner_vicon_sequence == request.vicon_sequence);
    assert(trajectory.points.size() >= 2);
    assert(trajectory.points.front().t_from_start_s == 0.0);
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
        const auto& point = trajectory.points[i];
        assert(std::isfinite(point.t_from_start_s));
        assert(point.position_world_m.allFinite());
        assert(point.orientation_world.coeffs().allFinite());
        assert(point.linear_velocity_world_m_s.allFinite());
        assert(point.angular_velocity_world_rad_s.allFinite());
        if (i > 0)
            assert(point.t_from_start_s >
                   trajectory.points[i - 1].t_from_start_s);
    }
    const auto& final = trajectory.points.back();
    assert(final.arrival_eligible);
    assert(final.linear_velocity_world_m_s.isZero(0.0));
    assert(final.angular_velocity_world_rad_s.isZero(0.0));

    // Both-arm production starts one worker per arm. The inherited planner
    // temporarily redirects process-global std::cout, so concurrent typed
    // solves must remain well-defined and complete independently.
    PlanningRequest other = request;
    other.request_id = 92;
    other.vicon_sequence = 43;
    PlannerSolveResult first_concurrent;
    PlannerSolveResult second_concurrent;
    std::thread first_thread([&] {
        std::ostringstream output;
        first_concurrent =
            SolveWorldTrajectoryForRequest(request, config, output);
    });
    std::thread second_thread([&] {
        std::ostringstream output;
        second_concurrent =
            SolveWorldTrajectoryForRequest(other, config, output);
    });
    first_thread.join();
    second_thread.join();
    assert(first_concurrent.exit_code == 0 && first_concurrent.trajectory);
    assert(second_concurrent.exit_code == 0 && second_concurrent.trajectory);
    assert(first_concurrent.trajectory->trajectory_id == request.request_id);
    assert(second_concurrent.trajectory->trajectory_id == other.request_id);

    std::filesystem::remove(goal_path);
    std::cout << "typed planner runtime characterization passed\n";
    return 0;
}
