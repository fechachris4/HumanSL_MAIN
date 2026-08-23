#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "CartesianPath.h"
#include "PlanSolver.h"
#include "PlannerConfig.h"
#include "PlannerModel.h"
#include "PlannerRuntime.h"
#include "StartState.h"

namespace {
int failures = 0;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        std::printf("FAIL: %s\n", message.c_str());
        ++failures;
    }
}

double MaxAbs(const Eigen::Matrix<double, 7, 1>& value) {
    return value.cwiseAbs().maxCoeff();
}

Eigen::Matrix<double, 7, 1> MeasuredQ() {
    const Eigen::Matrix<double, 7, 1> q_deg =
        (Eigen::Matrix<double, 7, 1>() <<
            94.868255615234375, 103.06673431396484, 336.72296142578125,
            7.3997693061828613, 350.69995117187494, 354.3056640625,
            184.83963012695315).finished();
    Eigen::Matrix<double, 7, 1> q_rad = q_deg * M_PI / 180.0;
    for (int joint = 0; joint < 7; ++joint)
        q_rad(joint) = WrapToPrincipalRad(q_rad(joint));
    return q_rad;
}

Eigen::Matrix<double, 7, 1> MeasuredQdot() {
    return (Eigen::Matrix<double, 7, 1>() <<
        0.001, -0.002, 0.003, -0.004, 0.005, -0.006, 0.007).finished();
}
}  // namespace

int main(int argc, char** argv) {
    Check(argc == 3, "usage: test_planner_start_state dh_tool.yaml dh_flange.yaml");
    if (argc != 3) return 1;

    const PlannerModel model = LoadPlannerModel(argv[1], /*has_tool=*/true);
    PlannerConfig config = LoadPlannerConfig("../config/planner.yaml");
    config.scene.clear();  // keep this start-state characterization collision-free
    const std::string joint_limits = "../config/joint_limits.yaml";
    const Eigen::Matrix<double, 7, 1> q_plan = MeasuredQ();
    const Eigen::Matrix<double, 7, 1> qdot_meas = MeasuredQdot();
    const Eigen::Vector3d start_position = ToolPositionInMount(model, q_plan);
    const gtsam::Pose3 start_pose = ToolPoseInMount(model, q_plan);
    Eigen::Matrix<double, 7, 1> q_goal = q_plan;
    q_goal(0) += 0.05;
    const gtsam::Pose3 goal_pose = ToolPoseInMount(model, q_goal);

    PlanRequest point_request;
    point_request.q_start_rad = q_plan;
    point_request.qdot_start_rad_s = qdot_meas;
    point_request.goal_position_m = goal_pose.translation();
    point_request.goal_rotation = goal_pose.rotation().matrix();
    const PlanOutcome point =
        SolveToPosition(model, point_request, joint_limits, config);
    Check(IsExecutable(point.status) && point.trajectory,
          "point plan is executable");
    if (IsExecutable(point.status) && point.trajectory) {
        Check(MaxAbs(point.trajectory->trajectory_pos.front() - q_plan) < 1e-12,
              "point q0 equals canonical measured q");
        Check(MaxAbs(point.trajectory->trajectory_vel.front() - qdot_meas) < 1e-12,
              "point qdot0 equals measured qdot");
        Check(point.terminal_candidate.has_value(),
              "point records selected terminal IK candidate");
        if (point.terminal_candidate) {
            Check(MaxAbs(point.trajectory->trajectory_pos.back() -
                         point.terminal_candidate->configuration) < 1e-12,
                  "point qN equals selected terminal configuration");
            Check(MaxAbs(point.terminal_candidate->configuration - q_plan) > 1e-3,
                  "point terminal differs from measured start");
        }
        Check(point.trajectory->start_costs.count("StartPosEquality") == 1,
              "point graph contains position equality");
        Check(point.trajectory->start_costs.count("StartVelEquality") == 1,
              "point graph contains velocity equality");
        Check(point.trajectory->start_costs.count("TerminalPosEquality") == 1,
              "point graph contains terminal equality");
        Check(point.trajectory->start_costs.count("PoseFactor") == 0,
              "point graph has no terminal workspace pose factor");
    }

    PlanRequest offline_point = point_request;
    offline_point.qdot_start_rad_s.reset();
    const PlanOutcome offline =
        SolveToPosition(model, offline_point, joint_limits, config);
    Check(IsExecutable(offline.status) && offline.trajectory,
          "offline point plan is executable");
    if (IsExecutable(offline.status) && offline.trajectory) {
        Check(offline.trajectory->start_costs.count("StartVelEquality") == 0,
              "offline point has no start velocity equality");
        Check(offline.trajectory->start_costs.count("StartVelPrior") == 0,
              "offline point has no zero start prior");
    }

    CircleSpec circle;
    circle.centre_m = start_position - Eigen::Vector3d(0.005, 0.0, 0.0);
    circle.radius_m = 0.005;
    circle.normal = Eigen::Vector3d::UnitZ();
    circle.samples = 8;
    circle.duration_s = 12.0;
    circle.orientation = OrientationPolicy::kFixed;
    const Eigen::Vector3d zyx = start_pose.rotation().matrix().eulerAngles(2, 1, 0);
    circle.fixed_rpy_rad = Eigen::Vector3d(zyx.z(), zyx.y(), zyx.x());
    const CartesianPath path = GenerateCircle(circle);
    const Eigen::Matrix<double, 7, 1> path_qdot = qdot_meas;

    const PathPlanOutcome traced = SolveAlongPath(
        model, path, q_plan, path_qdot, joint_limits, config);
    Check(IsExecutable(traced.status) && traced.trajectory,
          "traced plan is executable");
    if (IsExecutable(traced.status) && traced.trajectory) {
        Check(MaxAbs(traced.trajectory->trajectory_pos.front() - q_plan) < 1e-12,
              "traced q0 equals canonical measured q");
        Check(MaxAbs(traced.trajectory->trajectory_vel.front() - path_qdot) < 1e-12,
              "traced qdot0 equals measured qdot");
        Check(traced.trajectory->start_costs.count("StartPosEquality") == 1,
              "traced graph contains position equality");
        Check(traced.trajectory->start_costs.count("StartVelEquality") == 1,
              "traced graph contains velocity equality");
        Check(traced.terminal_candidate.has_value(),
              "traced records selected terminal IK candidate");
        if (traced.terminal_candidate)
            Check(MaxAbs(traced.trajectory->trajectory_pos.back() -
                         traced.terminal_candidate->configuration) < 1e-12,
                  "traced qN equals selected terminal configuration");
        Check(traced.trajectory->start_costs.count("TerminalPosEquality") == 1,
              "traced graph contains terminal equality");
    }

    const PathPlanOutcome offline_path = SolveAlongPath(
        model, path, q_plan, std::nullopt, joint_limits, config);
    Check(IsExecutable(offline_path.status) && offline_path.trajectory,
          "offline traced plan is executable");
    if (IsExecutable(offline_path.status) && offline_path.trajectory) {
        Check(offline_path.trajectory->start_costs.count("StartVelEquality") == 0,
              "offline traced path has no start velocity equality");
        Check(offline_path.trajectory->start_costs.count("StartVelPrior") == 0,
              "offline traced path has no zero start prior");
    }

    PlanningRequest live;
    live.request_id = 1;
    live.arm = PlanningArm::kRight;
    live.vicon_sequence = 1;
    live.receive_steady_s = 1.0;
    live.age_s = 0.001;
    live.q_rad = q_plan;
    live.qdot_rad_s = qdot_meas;
    live.goal.command_id = 1;
    live.goal.kind = GoalKind::kPoint;
    live.goal.point_m = start_position;
    Check(!ValidatePlanningRequest(live),
          "live request with measured velocity validates");

    PlannerRuntimeConfig runtime;
    runtime.planner_config_file = "../config/planner.yaml";
    runtime.joint_limits_file = joint_limits;
    runtime.right_dh_file = argv[1];
    runtime.left_dh_file = argv[2];
    runtime.runs_root = ".";
    const std::string transport_config = "/tmp/planner_start_state_no_scene.yaml";
    {
        std::ifstream source("../config/planner.yaml");
        std::ofstream destination(transport_config);
        std::string line;
        while (std::getline(source, line)) {
            if (line.find("enabled: true") != std::string::npos)
                line.replace(line.find("true"), 4, "false");
            destination << line << '\n';
        }
    }
    runtime.planner_config_file = transport_config;
    std::ostringstream diagnostics;
    const PlannerSolveResult transported =
        SolvePlanForRequest(live, runtime, diagnostics);
    Check(IsExecutable(transported.status) && transported.trajectory,
          "known-good typed transport is executable");
    std::remove(transport_config.c_str());

    if (failures == 0)
        std::puts("test_planner_start_state: all assertions passed");
    return failures == 0 ? 0 : 1;
}
