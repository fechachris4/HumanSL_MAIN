//
// Hardware-free tests for TrajectoryPlayback, including a closed-loop
// replay of the full pipeline the Runner executes per cycle:
//
//   measured state -> DesiredVelocity -> per-joint clamp ->
//   PositionIntegration::Apply -> commanded setpoint -> (simulated plant)
//
// The simulated plant is the position servo's idealization: the measured
// position each cycle is the PREVIOUS cycle's commanded setpoint (one
// exchange of transport delay), matching how the real loop consumes the
// previous Refresh reply. Wrap variants store the measurement in [0,360)
// exactly as the hardware reports it.
//
// No robot, no Pinocchio. Returns nonzero on the first failure.
//

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "actuation/PositionIntegration.h"
#include "control/TrajectoryPlayback.h"

namespace
{
    int failures = 0;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    const JointVector kClipDegS = {79.6, 79.6, 79.6, 79.6, 69.9, 69.9, 69.9};

    // Quintic move on joints 2 and 4, rest elsewhere — built directly (the
    // loader has its own tests).
    Trajectory MakeTrajectory(double amplitude_deg = 10.0,
                              double duration_s = 2.0, double dt = 0.001,
                              double base_deg = 20.0)
    {
        Trajectory t;
        t.dt_s = dt;
        const int n = static_cast<int>(duration_s / dt) + 1;
        for (int i = 0; i < n; ++i)
        {
            const double s = (i * dt) / duration_s;
            const double shape =
                10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s;
            const double dshape =
                (30 * s * s - 60 * s * s * s + 30 * s * s * s * s) / duration_s;
            Eigen::Matrix<double, 7, 1> q =
                Eigen::Matrix<double, 7, 1>::Constant(base_deg);
            Eigen::Matrix<double, 7, 1> qd = Eigen::Matrix<double, 7, 1>::Zero();
            q[1] += amplitude_deg * shape;
            q[3] -= 0.5 * amplitude_deg * shape;
            qd[1] = amplitude_deg * dshape;
            qd[3] = -0.5 * amplitude_deg * dshape;
            t.pos_deg.push_back(q);
            t.vel_deg_s.push_back(qd);
        }
        return t;
    }

    struct SimResult {
        double max_cmd_vs_ref_deg = 0.0;  // |integrated command - reference|
        double max_meas_vs_ref_deg = 0.0; // |measured - reference| (wrapped)
        double max_step_deg = 0.0;        // largest per-cycle command step
        int done_edges = 0;
        int refused_edges = 0;
        int final_state = 0;
        JointVector final_cmd_deg{};
        Eigen::Matrix<double, 7, 1> final_ref_deg =
            Eigen::Matrix<double, 7, 1>::Zero();
    };

    // Run the pipeline for `cycles` cycles. `dt_of_cycle` supplies each
    // cycle's measured dt (the Runner clamps at 2x nominal upstream —
    // callers model that by never returning more than 0.002).
    // `start_offset_deg` displaces the plant from the trajectory start.
    // `wrap_measurement` reports the plant position in [0,360) like the
    // hardware does.
    template <typename DtFn>
    SimResult Simulate(const Trajectory& trajectory, PlaybackSettings settings,
                       int cycles, DtFn dt_of_cycle,
                       double start_offset_deg = 0.0,
                       bool wrap_measurement = false)
    {
        SimResult r;
        TrajectoryPlayback controller(trajectory, settings);
        PositionIntegration actuation;

        auto wrap = [&](double deg)
        {
            if (!wrap_measurement)
                return deg;
            double w = std::fmod(deg, 360.0);
            return w < 0.0 ? w + 360.0 : w;
        };

        // Plant truth, continuous degrees. Measured = wrap(plant).
        Eigen::Matrix<double, 7, 1> plant_deg = trajectory.pos_deg.front();
        plant_deg.array() += start_offset_deg;

        RobotState state;
        for (int j = 0; j < 7; ++j)
            state.q_rad[j] = wrap(plant_deg[j]) * kDegToRad;
        state.qdot_rad_s.setZero();

        actuation.Prepare(state); // T4: integrator seeded from measurement
        controller.Reset(state);  // T5

        JointVector cmd_deg{};
        JointVector cmd_vel{};
        for (int j = 0; j < 7; ++j)
            cmd_deg[j] = state.q_rad[j] * kRadToDeg;
        JointVector prev_cmd = cmd_deg;

        double t_ref = 0.0;
        for (int cycle = 0; cycle < cycles; ++cycle)
        {
            const double dt = dt_of_cycle(cycle);
            ControllerStatus status;
            Eigen::Matrix<double, 7, 1> qdot =
                controller.DesiredVelocity(state, dt, status);
            for (int j = 0; j < 7; ++j)
                qdot[j] = std::clamp(qdot[j] * kRadToDeg, -kClipDegS[j],
                                     kClipDegS[j]) *
                          kDegToRad;
            actuation.Apply(qdot, state, dt, cmd_deg, cmd_vel);

            r.done_edges += status.playback_done_edge ? 1 : 0;
            r.refused_edges += status.playback_refused_edge ? 1 : 0;
            r.final_state = status.playback_state;
            r.final_ref_deg = status.q_ref_deg;

            t_ref = std::min(t_ref + dt, TrajectoryDurationS(trajectory));
            const Eigen::Matrix<double, 7, 1> q_ref =
                SamplePositionDeg(trajectory, t_ref);
            for (int j = 0; j < 7; ++j)
            {
                r.max_cmd_vs_ref_deg =
                    std::max(r.max_cmd_vs_ref_deg,
                             std::abs(std::remainder(cmd_deg[j] - q_ref[j], 360.0)));
                r.max_step_deg = std::max(
                    r.max_step_deg, std::abs(cmd_deg[j] - prev_cmd[j]));
            }
            prev_cmd = cmd_deg;

            // Plant: the servo reaches this cycle's setpoint by the next
            // measurement (one-cycle transport delay, like the real loop).
            for (int j = 0; j < 7; ++j)
            {
                plant_deg[j] = cmd_deg[j];
                state.q_rad[j] = wrap(plant_deg[j]) * kDegToRad;
                r.max_meas_vs_ref_deg = std::max(
                    r.max_meas_vs_ref_deg,
                    std::abs(std::remainder(plant_deg[j] - q_ref[j], 360.0)));
            }
        }
        r.final_cmd_deg = cmd_deg;
        return r;
    }

    const PlaybackSettings kDefaults{0.5, 0.2};

    void TestNominalTracking()
    {
        const Trajectory t = MakeTrajectory();
        // 2 s of motion + 1 s of hold at 1 kHz nominal dt.
        const SimResult r =
            Simulate(t, kDefaults, 3000, [](int) { return 0.001; });
        Check(r.max_cmd_vs_ref_deg < 0.05,
              "nominal: command tracks the reference (max " +
                  std::to_string(r.max_cmd_vs_ref_deg) + " deg)");
        Check(r.done_edges == 1, "nominal: exactly one completion edge");
        Check(r.refused_edges == 0, "nominal: no refusal");
        Check(r.final_state == 2, "nominal: final state is done-holding");
        for (int j = 0; j < 7; ++j)
            Check(std::abs(r.final_cmd_deg[j] - t.pos_deg.back()[j]) < 0.01,
                  "nominal: final command = final reference, joint " +
                      std::to_string(j + 1));
        // No command step may exceed what the clamp allows in one cycle.
        Check(r.max_step_deg <= 79.6 * 0.001 + 1e-9,
              "nominal: every command step within clip*dt");
    }

    void TestTimingJitter()
    {
        const Trajectory t = MakeTrajectory();
        // dt pattern modeled on the 2026-07-31 run evidence: mostly ~1 ms,
        // frequent 1.5 ms, occasional 2 ms (the Runner's clamp ceiling).
        const auto jitter = [](int cycle)
        {
            if (cycle % 97 == 0) return 0.002;
            if (cycle % 13 == 0) return 0.0015;
            if (cycle % 7 == 0) return 0.0008;
            return 0.001;
        };
        const SimResult r = Simulate(t, kDefaults, 3000, jitter);
        Check(r.max_cmd_vs_ref_deg < 0.05,
              "jitter: command still tracks the reference (max " +
                  std::to_string(r.max_cmd_vs_ref_deg) + " deg)");
        Check(r.done_edges == 1, "jitter: exactly one completion edge");
        Check(r.max_step_deg <= 79.6 * 0.002 + 1e-9,
              "jitter: steps bounded by clip * dt even on slow cycles");
    }

    void TestStartOffsetWithinGate()
    {
        const Trajectory t = MakeTrajectory();
        // 0.15 deg offset: inside the 0.2 deg gate. The kp = 0.5 term must
        // absorb it smoothly (time constant 2 s): no step, converged after
        // the trajectory plus a few seconds of hold.
        const SimResult r =
            Simulate(t, kDefaults, 8000, [](int) { return 0.001; }, 0.15);
        Check(r.refused_edges == 0, "offset: plays (inside the gate)");
        Check(r.max_step_deg <= 79.6 * 0.001 + 1e-9,
              "offset: absorbed without a command jump");
        double final_err = 0.0;
        for (int j = 0; j < 7; ++j)
            final_err = std::max(final_err,
                                 std::abs(r.final_cmd_deg[j] - t.pos_deg.back()[j]));
        Check(final_err < 0.01,
              "offset: converged onto the final reference (residual " +
                  std::to_string(final_err) + " deg)");
    }

    void TestStartMismatchRefusal()
    {
        const Trajectory t = MakeTrajectory();
        // 0.5 deg offset: beyond the 0.2 deg gate — must refuse and never move.
        const SimResult r =
            Simulate(t, kDefaults, 1000, [](int) { return 0.001; }, 0.5);
        Check(r.refused_edges == 1, "mismatch: exactly one refusal edge");
        Check(r.final_state == 3, "mismatch: state stays refused");
        Check(r.done_edges == 0, "mismatch: never completes");
        Check(r.max_step_deg < 1e-12, "mismatch: command never moves");
        // The reference telemetry still points at the trajectory start.
        Check((r.final_ref_deg - t.pos_deg.front()).norm() < 1e-9,
              "mismatch: telemetry reports the un-reached start");
    }

    void TestWrappedFeedback()
    {
        // Trajectory on the continuous branch around -130 deg while the
        // plant reports [0,360) (i.e. ~230 deg): the wrapped comparison
        // must see a match, play normally, and never command a 360 jump.
        const Trajectory t = MakeTrajectory(8.0, 2.0, 0.001, -130.0);
        const SimResult r = Simulate(t, kDefaults, 3000,
                                     [](int) { return 0.001; }, 0.0,
                                     /*wrap_measurement=*/true);
        Check(r.refused_edges == 0, "wrap: gate passes across the branch cut");
        Check(r.done_edges == 1, "wrap: completes");
        Check(r.max_cmd_vs_ref_deg < 0.05,
              "wrap: tracks (max " + std::to_string(r.max_cmd_vs_ref_deg) +
                  " deg)");
        Check(r.max_step_deg <= 79.6 * 0.001 + 1e-9,
              "wrap: no phantom 360-degree step");
    }

    void TestHoldRejectsDisturbance()
    {
        const Trajectory t = MakeTrajectory();
        TrajectoryPlayback controller(t, kDefaults);
        PositionIntegration actuation;
        RobotState state;
        for (int j = 0; j < 7; ++j)
            state.q_rad[j] = t.pos_deg.front()[j] * kDegToRad;
        state.qdot_rad_s.setZero();
        actuation.Prepare(state);
        controller.Reset(state);

        JointVector cmd{}, vel{};
        ControllerStatus status;
        // Play to completion.
        for (int cycle = 0; cycle < 2500; ++cycle)
        {
            Eigen::Matrix<double, 7, 1> qdot =
                controller.DesiredVelocity(state, 0.001, status);
            actuation.Apply(qdot, state, 0.001, cmd, vel);
            for (int j = 0; j < 7; ++j)
                state.q_rad[j] = cmd[j] * kDegToRad;
        }
        Check(status.playback_state == 2, "hold: completed before disturbance");
        // Push the measured position 0.05 deg off the final reference: the
        // kp term must command a restoring velocity toward the reference.
        state.q_rad[1] += 0.05 * kDegToRad;
        Eigen::Matrix<double, 7, 1> qdot =
            controller.DesiredVelocity(state, 0.001, status);
        Check(qdot[1] < -1e-6, "hold: restoring velocity opposes the push");
        Check(std::abs(qdot[1] * kRadToDeg + 0.5 * 0.05) < 1e-6,
              "hold: restoring velocity = kp * error exactly");
    }

    void TestTwoRowTrajectory()
    {
        // Degenerate but legal: 2 samples, zero motion. Must complete on
        // the first cycle past dt and hold.
        Trajectory t;
        t.dt_s = 0.001;
        t.pos_deg.assign(2, Eigen::Matrix<double, 7, 1>::Constant(15.0));
        t.vel_deg_s.assign(2, Eigen::Matrix<double, 7, 1>::Zero());
        const SimResult r =
            Simulate(t, kDefaults, 10, [](int) { return 0.001; });
        Check(r.done_edges == 1, "two-row: completes once");
        Check(r.max_step_deg < 1e-9, "two-row: zero motion commanded");
    }
} // namespace

int main()
{
    TestNominalTracking();
    TestTimingJitter();
    TestStartOffsetWithinGate();
    TestStartMismatchRefusal();
    TestWrappedFeedback();
    TestHoldRejectsDisturbance();
    TestTwoRowTrajectory();

    if (failures == 0)
    {
        std::cout << "test_playback: all tests passed\n";
        return 0;
    }
    std::cout << "test_playback: " << failures << " FAILURES\n";
    return 1;
}
