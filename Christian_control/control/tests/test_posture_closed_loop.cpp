//
// Closed-loop evidence for slice 3 of planner/controller redundancy
// agreement: the SAME posture-bearing trajectory is executed twice through
// the full ArmExecutionCore (measure -> reference -> law -> clamp ->
// integration) against an ideal plant (measured = last commanded, one
// cycle late — exactly the reactive position-integration architecture),
// once with posture attraction ON and once OFF.
//
// What it must show, simulation-grade:
//   1. task tracking stays tight in BOTH runs — the posture term never
//      buys posture at the price of the TCP (the 2026-08-05 stall
//      equilibrium is the failure mode this pins);
//   2. with the term ON the executed joint path ends near the planned
//      terminal configuration; with it OFF the null-space component of
//      the plan is simply lost (DLS least-norm re-decides redundancy) —
//      ON must recover a clear majority of what OFF loses.
//
// The trajectory is built from a joint-space plan q_plan(t) via the same
// kinematic model the controller measures with, so the Cartesian
// reference and the posture are consistent by construction — exactly the
// planner's own relationship between its FK exit and its joint states.
//

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>

#include "ExecutionCore.h"
#include "ExecutionConfig.h"
#include "Kinematics.h"
#include "RobotModel.h"

namespace {

int failures = 0;

void Check(bool condition, const char* what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kPlanDurationS = 2.0;
constexpr double kHoldTailS = 1.0;
constexpr double kSampleDtS = 0.02;

// Smoothstep 0..1 with zero end-slopes, so the plan starts and ends at
// rest and the terminal point is honestly stationary.
double Smooth(double u) { return u * u * (3.0 - 2.0 * u); }

Eigen::Matrix<double, 7, 1> PlanConfiguration(
    const Eigen::Matrix<double, 7, 1>& q0_rad, double t_s)
{
    // A deliberate mix: bounded joints (2, 4, 6 -> indices 1, 3, 5) and
    // continuous joints (3, 5 -> indices 2, 4) all move, so the plan has a
    // genuine null-space component the task alone cannot reproduce.
    Eigen::Matrix<double, 7, 1> delta_rad = Eigen::Matrix<double, 7, 1>::Zero();
    delta_rad(1) = 8.0 * kDegToRad;
    delta_rad(2) = -10.0 * kDegToRad;
    delta_rad(3) = 6.0 * kDegToRad;
    delta_rad(4) = 12.0 * kDegToRad;
    delta_rad(5) = -5.0 * kDegToRad;
    const double u = std::clamp(t_s / kPlanDurationS, 0.0, 1.0);
    return q0_rad + Smooth(u) * delta_rad;
}

std::unique_ptr<WorldCartesianTrajectory> BuildPostureTrajectory(
    DualArmKinematics& model, const Eigen::Matrix<double, 7, 1>& q0_rad)
{
    auto trajectory = std::make_unique<WorldCartesianTrajectory>();
    trajectory->trajectory_id = 1;
    trajectory->planner_vicon_sequence = 1;
    const int samples = static_cast<int>(kPlanDurationS / kSampleDtS) + 1;
    trajectory->points.reserve(samples);
    for (int i = 0; i < samples; ++i) {
        const double t = i * kSampleDtS;
        const Eigen::Matrix<double, 7, 1> q = PlanConfiguration(q0_rad, t);
        const Pose pose = model.ToolPoseInMount(Arm::kRight, q);
        WorldCartesianTrajectoryPoint point;
        point.t_from_start_s = t;
        point.position_world_m = pose.position; // identity world_T_mount
        point.orientation_world = Eigen::Quaterniond(pose.rotation);
        point.has_posture = true;
        point.posture_rad = q;
        if (!trajectory->points.empty()) {
            const auto& previous = trajectory->points.back();
            point.linear_velocity_world_m_s =
                (point.position_world_m - previous.position_world_m) /
                kSampleDtS;
            if (previous.orientation_world.coeffs().dot(
                    point.orientation_world.coeffs()) < 0.0)
                point.orientation_world.coeffs() *= -1.0;
        }
        trajectory->points.push_back(point);
    }
    trajectory->points.back().linear_velocity_world_m_s.setZero();
    trajectory->points.back().angular_velocity_world_rad_s.setZero();
    trajectory->points.back().arrival_eligible = true;
    return trajectory;
}

ArmExecutionInput InputAt(const JointVector& measured_deg)
{
    ArmExecutionInput input;
    input.dt_s = config::kControlDtS;
    input.measured_position_deg = measured_deg;
    input.world.mount_valid = true;
    input.world.mount_position_m.setZero();
    input.world.mount_quat_xyzw[0] = 0.0;
    input.world.mount_quat_xyzw[1] = 0.0;
    input.world.mount_quat_xyzw[2] = 0.0;
    input.world.mount_quat_xyzw[3] = 1.0;
    input.world.sequence = 1;
    input.world.age_s = 0.0;
    return input;
}

struct RunMetrics {
    double max_task_error_m = 0.0;
    double final_task_error_m = 0.0;
    // Max over joints of |wrap(q_executed - q_plan_terminal)| at run end.
    double final_posture_error_rad = 0.0;
};

RunMetrics RunPlan(bool posture_enabled)
{
    RobotModel robot_model(GEN3_DUAL_URDF_PATH);
    DualArmKinematics model(robot_model, Arm::kRight,
                            config::kLeftNominalRad,
                            config::kRightBaseFrame,
                            config::kRightEndEffectorFrame);
    CartesianTrajectoryMailbox mailbox;
    ExecutionConfig configuration = ProductionExecutionConfig();
    configuration.gains.posture_enabled = posture_enabled;
    configuration.gains.posture_gain_s_inv = config::kPostureGain;
    ArmExecutionCore core(model, configuration, mailbox,
                          config::kControlDtS);

    const JointVector q0_deg{10, 20, 30, 40, 50, 60, 70};
    Eigen::Matrix<double, 7, 1> q0_rad;
    for (int j = 0; j < 7; ++j) q0_rad(j) = q0_deg[j] * kDegToRad;

    mailbox.Publish(BuildPostureTrajectory(model, q0_rad));
    core.Seed(q0_deg, JointVector{});

    RunMetrics metrics;
    JointVector measured_deg = q0_deg;
    const int steps = static_cast<int>(
        (kPlanDurationS + kHoldTailS) / config::kControlDtS);
    ArmExecutionResult result;
    bool activated = false;
    for (int i = 0; i < steps; ++i) {
        result = core.Step(InputAt(measured_deg));
        const ExecutionStopDecision stop =
            core.ResolveStop(result.commanded_deg, AdapterHealth{});
        Check(!stop.following_error && !stop.nonfinite_stop &&
                  !stop.overrun_stop,
              "no stop fires during the closed-loop run");
        activated = activated || result.controller_status.cartesian_traj_activated;
        const double task_error =
            (result.reference.ee_pose_world.position_m -
             result.measured.ee_pose_world.position_m).norm();
        metrics.max_task_error_m = std::max(metrics.max_task_error_m, task_error);
        metrics.final_task_error_m = task_error;
        // Ideal plant: next cycle measures this cycle's command.
        measured_deg = result.commanded_deg;
    }
    Check(activated, "the posture trajectory activated");

    const Eigen::Matrix<double, 7, 1> q_terminal =
        PlanConfiguration(q0_rad, kPlanDurationS);
    for (int j = 0; j < 7; ++j) {
        const double error = std::abs(std::remainder(
            result.state.q_rad(j) - q_terminal(j), 2.0 * M_PI));
        metrics.final_posture_error_rad =
            std::max(metrics.final_posture_error_rad, error);
    }
    return metrics;
}

} // namespace

int main()
{
    const RunMetrics off = RunPlan(false);
    const RunMetrics on = RunPlan(true);

    std::printf("task error   max/final (mm): off %.3f/%.3f, on %.3f/%.3f\n",
                off.max_task_error_m * 1e3, off.final_task_error_m * 1e3,
                on.max_task_error_m * 1e3, on.final_task_error_m * 1e3);
    std::printf("terminal posture error (deg): off %.3f, on %.3f\n",
                off.final_posture_error_rad / kDegToRad,
                on.final_posture_error_rad / kDegToRad);

    // 1. The task is never sacrificed: tight tracking with the term ON,
    //    and no stall — the run settles onto the target.
    Check(on.max_task_error_m < 0.010, "posture ON keeps task error under 10 mm");
    Check(on.final_task_error_m < 0.002, "posture ON settles within 2 mm (no stall)");
    Check(off.final_task_error_m < 0.002, "baseline settles within 2 mm");

    // 2. The planned redundancy is recovered: OFF loses a measurable null
    //    component; ON must end at least twice as close to the planned
    //    terminal configuration, and close in absolute terms.
    Check(off.final_posture_error_rad > 0.5 * kDegToRad,
          "the plan has a genuine null component the task alone cannot fix");
    Check(on.final_posture_error_rad < 0.5 * off.final_posture_error_rad,
          "posture ON recovers a clear majority of the planned posture");
    // Measured 2026-08-23 at kPostureGain = 1.0: 1.42 deg terminal error
    // against 9.21 deg open-loop (gain sweep in the commit message); the
    // threshold carries margin over that measurement, not ambition.
    Check(on.final_posture_error_rad < 2.0 * kDegToRad,
          "posture ON ends within 2 deg of the planned configuration");

    if (failures == 0)
        std::puts("test_posture_closed_loop: all assertions passed");
    return failures == 0 ? 0 : 1;
}
