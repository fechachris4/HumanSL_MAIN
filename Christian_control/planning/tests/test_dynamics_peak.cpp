//
// MeasureDynamics peak location — when a plan is rejected for exceeding a
// dynamic limit, the report must say WHICH joint and WHEN, not just the
// worst ratio. (2026-08-24: a traced-circle repair loop diverged on a
// duration-invariant acceleration spike, and the evidence could not say
// where the spike was.) Pure validator behaviour on a synthetic
// trajectory: no solver, no robot.
//

#include <cmath>
#include <cstdio>
#include <string>

#include "PlanSolver.h"
#include "PlannerModel.h"
#include "StartState.h"
#include "ValidatePlan.h"

namespace {
int failures = 0;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        std::printf("FAIL: %s\n", message.c_str());
        ++failures;
    }
}

// A joint configuration known to be self-collision-clear (the same measured
// pose test_planner_start_state characterises).
Eigen::Matrix<double, 7, 1> ValidQ() {
    Eigen::Matrix<double, 7, 1> q_deg;
    q_deg << 94.868, 103.067, 336.723, 7.400, 350.700, 354.306, 184.840;
    Eigen::Matrix<double, 7, 1> q_rad = q_deg * M_PI / 180.0;
    for (int joint = 0; joint < 7; ++joint)
        q_rad(joint) = WrapToPrincipalRad(q_rad(joint));
    return q_rad;
}
}  // namespace

int main(int argc, char** argv) {
    Check(argc == 2, "usage: test_dynamics_peak dh_flange.yaml");
    if (argc != 2) return 1;
    const PlannerModel model = LoadPlannerModel(argv[1], /*has_tool=*/false);

    // Synthetic 2 s trajectory at 1 ms: all joints rest except joint 4
    // (index 3), whose velocity is a triangular bump centred at t = 1.5 s,
    // 0.2 s wide, peaking at 0.3 rad/s. Peak acceleration is the ramp
    // slope, 3 rad/s^2, present only on joint 4 around the bump.
    const double duration_s = 2.0;
    const Eigen::Matrix<double, 7, 1> q = ValidQ();
    TrajectoryResult trajectory;
    trajectory.dt = 1e-3;
    const std::size_t count = 2001;
    for (std::size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) * trajectory.dt;
        Eigen::Matrix<double, 7, 1> qdot = Eigen::Matrix<double, 7, 1>::Zero();
        const double offset = std::abs(t - 1.5);
        if (offset < 0.1)
            qdot(3) = 0.3 * (1.0 - offset / 0.1);
        trajectory.trajectory_pos.push_back(gtsam::Vector(q));
        trajectory.trajectory_vel.push_back(gtsam::Vector(qdot));
    }

    PlanValidationInputs inputs;
    inputs.measured_q_rad = q;
    inputs.measured_qdot_rad_s = Eigen::Matrix<double, 7, 1>::Zero();
    inputs.position_lower_rad = Eigen::Matrix<double, 7, 1>::Constant(-10.0);
    inputs.position_upper_rad = Eigen::Matrix<double, 7, 1>::Constant(10.0);
    inputs.effective_velocity_rad_s = Eigen::Matrix<double, 7, 1>::Constant(1.0);
    inputs.effective_acceleration_rad_s2 =
        Eigen::Matrix<double, 7, 1>::Constant(1.0);
    inputs.requested_terminal_mount = ToolPoseInMount(model, q);
    inputs.candidate_terminal_mount = inputs.requested_terminal_mount;
    inputs.intended_status = PlanStatus::kReached;
    inputs.validation_dt_s = 0.004;

    const PlanValidationReport report =
        ValidatePlan(model, trajectory, duration_s, inputs);

    Check(report.disposition == CandidateDisposition::kNeedsLongerDuration,
          "acceleration 3x over limit asks for a longer duration");
    Check(std::abs(report.max_velocity_ratio - 0.3) < 0.05,
          "velocity ratio reports the 0.3 rad/s bump");
    Check(report.max_acceleration_ratio > 2.0,
          "acceleration ratio reports the 3 rad/s^2 ramp");

    Check(report.peak_velocity_joint == 4,
          "peak velocity joint is joint 4 (got " +
              std::to_string(report.peak_velocity_joint) + ")");
    Check(std::abs(report.peak_velocity_time_s - 1.5) < 0.02,
          "peak velocity time is the bump centre (got " +
              std::to_string(report.peak_velocity_time_s) + ")");
    Check(report.peak_acceleration_joint == 4,
          "peak acceleration joint is joint 4 (got " +
              std::to_string(report.peak_acceleration_joint) + ")");
    Check(report.peak_acceleration_time_s > 1.35 &&
              report.peak_acceleration_time_s < 1.65,
          "peak acceleration time lies on the bump ramps (got " +
              std::to_string(report.peak_acceleration_time_s) + ")");

    if (failures == 0) std::printf("test_dynamics_peak: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
