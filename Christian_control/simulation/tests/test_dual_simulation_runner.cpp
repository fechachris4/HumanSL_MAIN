//
// DualSimulationRunner tests (Plan 02 Task 5): the deterministic dual-arm
// coordinator that steps two frozen execution cores against the MuJoCo
// plant on the same 2 ms control tick.
//
// What is under test, per the accepted shape proposal and plan Step 1:
//   - both cores consume the SAME tick (t_s) and the SAME ideal Mount
//     state (pose bit-equal to the prescription, sequence = tick + 1,
//     age 0, exact zero twist) on every cycle;
//   - both commands are computed BEFORE physics advances: cycle N's
//     measured joint state is the previous exchange's reply, and
//     simulated time reads (N + 1) * 0.002 s only in next_feedback;
//   - the cores are seeded from CONTINUOUS degrees (home joint 4 is
//     -130 deg; a wrapped 230 deg seed would command through the +/-147.8
//     deg range and is the pre-registered failure this check catches);
//   - either arm's stop verdict latches the shared stop and prevents BOTH
//     commands from advancing (Step throws after the latch);
//   - Reset clears references and integrators and requires a fresh Start
//     (a re-run after Reset reproduces the first run bit-for-bit);
//   - repeated runs are byte-identical (nothing reads wall-clock time);
//   - the ideal-mode dual hold: no stop, overrun_count == 0 and
//     world_fresh == true on every cycle (pre-registered prediction 6),
//     no contact over the run (the accepted analysis correction: an
//     inter-arm geom contact would fake a Mount-coupling error), and the
//     TCP stays within the centimetre-scale servo-droop bound of the
//     accepted packet (prediction 5) around its captured world hold pose;
//   - the SCRIPTED Mount path (Plan 03 Task 1 wiring, gated 2026-08-17):
//     kStatic ignores the amplitude fields, and under translation,
//     rotation and combined motion the sample both cores receive AND the
//     pose the plant holds are SampleMountMotion(t_k) — see
//     TestScriptedMountWiring for why each check cannot pass vacuously.
//
// Evidence class: unit test / simulation (generic position-servo plant;
// kp/kv are declared generic mechanics, not Kinova parameters).
//

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "DualSimulationRunner.h"
#include "MountMotion.h"
#include "SimulationConfig.h"
#include "StopPriority.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::printf("FAIL: %s\n", what.c_str());
            ++failures;
        }
    }

    constexpr double kControlDt = 0.002; // s
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    // A deliberately non-trivial stationary Mount pose so the world
    // composition (quaternion wire order, rotation into world) is
    // exercised, not just the identity.
    CartesianPose TestMountPose()
    {
        CartesianPose pose;
        pose.position_m = Eigen::Vector3d(0.05, -0.10, 0.80);
        pose.rotation =
            Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
        return pose;
    }

    SimulationConfig BaseConfig()
    {
        SimulationConfig config;
        config.model_xml_path = SIM_MODEL_XML_PATH;
        config.urdf_path = SIM_URDF_PATH;
        config.control_dt_s = kControlDt;
        config.physics_substeps = 1;
        config.world_T_mount_at_zero = TestMountPose();
        return config;
    }

    // Home keyframe continuous degrees straight from the loaded model's
    // key_qpos (not through the feedback path under test).
    void ReadHomeDegrees(const DualSimulationRunner& runner,
                         JointVector& right_deg, JointVector& left_deg)
    {
        const mjModel& model = runner.Backend().Model();
        const DualModelContract& ids = runner.Backend().Contract();
        for (int i = 0; i < 7; ++i) {
            right_deg[i] =
                model.key_qpos[ids.right.joint_qpos_adr[i]] * kRadToDeg;
            left_deg[i] =
                model.key_qpos[ids.left.joint_qpos_adr[i]] * kRadToDeg;
        }
    }

    Eigen::Vector3d SitePosition(const DualSimulationRunner& runner,
                                 int site_id)
    {
        const mjData& data = runner.Backend().Data();
        return Eigen::Vector3d(data.site_xpos[3 * site_id + 0],
                               data.site_xpos[3 * site_id + 1],
                               data.site_xpos[3 * site_id + 2]);
    }

    bool StopRequested(const ExecutionStopDecision& decision)
    {
        return decision.priority.reason != StopPriorityReason::kNone ||
               decision.nonfinite_stop || decision.overrun_stop;
    }

    // ---------------------------------------------------------------
    // Guard ordering, seeding, per-cycle contract, reset
    // ---------------------------------------------------------------
    void TestLifecycleAndCycleContract()
    {
        const SimulationConfig config = BaseConfig();
        DualSimulationRunner runner(config);

        // Step before Start is the accepted API-ordering guard.
        bool threw = false;
        try {
            runner.Step();
        } catch (const std::logic_error&) {
            threw = true;
        }
        Check(threw, "Step before Start throws std::logic_error");

        const SimArmFeedback seed = runner.Start();

        // Start twice without Reset is rejected.
        threw = false;
        try {
            runner.Start();
        } catch (const std::logic_error&) {
            threw = true;
        }
        Check(threw, "Start twice without Reset throws std::logic_error");

        // The seed feedback is the home keyframe, in continuous degrees.
        JointVector home_right{}, home_left{};
        ReadHomeDegrees(runner, home_right, home_left);
        for (int i = 0; i < 7; ++i) {
            Check(std::abs(seed.right.continuous_position_deg[i] -
                           home_right[i]) < 1e-9,
                  "seed right joint " + std::to_string(i + 1) +
                      " continuous == home keyframe");
            Check(std::abs(seed.left.continuous_position_deg[i] -
                           home_left[i]) < 1e-9,
                  "seed left joint " + std::to_string(i + 1) +
                      " continuous == home keyframe");
        }
        // Home joint 4 is -130 deg: the wrapped wire reports 230 deg.
        Check(std::abs(home_right[3] - (-130.0)) < 1e-6,
              "home keyframe joint 4 is -130 deg (test precondition)");
        Check(std::abs(seed.right.measured_position_deg[3] - 230.0) < 1e-6,
              "wrapped wire reports joint 4 as 230 deg (test precondition)");

        SimArmFeedback previous = seed;
        std::vector<JointVector> first_run_right_commands;
        for (long tick = 0; tick < 5; ++tick) {
            const DualSimulationCycle cycle = runner.Step();
            const std::string at = "cycle " + std::to_string(tick);

            // Continuous seed evidence (binding correction): the first
            // integrated command frame stays near -130 deg on joint 4. A
            // wrapped 230 deg seed would command ~230 deg (4.01 rad)
            // against the +/-147.8 deg range.
            if (tick == 0) {
                Check(std::abs(cycle.right.commanded_deg[3] - (-130.0)) < 5.0,
                      "first right command joint 4 near -130 deg "
                      "(continuous seed, not wrapped)");
                Check(std::abs(cycle.left.commanded_deg[3] - (-130.0)) < 5.0,
                      "first left command joint 4 near -130 deg "
                      "(continuous seed, not wrapped)");
            }

            // Same tick for both cores.
            const double expected_t = static_cast<double>(tick) * kControlDt;
            Check(cycle.right.state.t_s == expected_t &&
                      cycle.left.state.t_s == expected_t,
                  at + ": both cores consume t_s = tick * dt");

            // Same ideal Mount state, exact every tick: sequence tick + 1
            // (sequence 0 means "never arrived" — State.h:61), fresh, pose
            // bit-equal to the prescription, exact zero twist.
            const std::uint64_t expected_seq =
                static_cast<std::uint64_t>(tick) + 1;
            Check(cycle.right.state.world_sequence == expected_seq &&
                      cycle.left.state.world_sequence == expected_seq,
                  at + ": world sequence = tick + 1 on both arms");
            Check(cycle.right.state.world_fresh &&
                      cycle.left.state.world_fresh,
                  at + ": world_fresh on both arms");
            Check(cycle.right.state.world_p_mountseg ==
                          config.world_T_mount_at_zero.position_m &&
                      cycle.left.state.world_p_mountseg ==
                          config.world_T_mount_at_zero.position_m,
                  at + ": Mount position exact (bit-equal) on both arms");
            Check((cycle.right.state.world_R_mountseg -
                   config.world_T_mount_at_zero.rotation)
                          .cwiseAbs()
                          .maxCoeff() < 1e-12,
                  at + ": Mount rotation exact to quaternion round-trip");
            Check(cycle.right.state.world_mount_twist_valid &&
                      cycle.right.state.world_v_mountseg_m_s.isZero(0.0) &&
                      cycle.right.state.world_w_mountseg_rad_s.isZero(0.0) &&
                      cycle.left.state.world_mount_twist_valid &&
                      cycle.left.state.world_v_mountseg_m_s.isZero(0.0) &&
                      cycle.left.state.world_w_mountseg_rad_s.isZero(0.0),
                  at + ": Mount twist valid and exactly zero");

            // Commands computed before physics: this cycle's measured
            // state is the PREVIOUS exchange's wrapped reply (bit-equal
            // through the same deg -> rad conversion the core compiles),
            // and simulated time has advanced only in next_feedback.
            bool measured_matches = true;
            for (int i = 0; i < 7; ++i) {
                if (cycle.right.state.q_rad[i] !=
                        previous.right.measured_position_deg[i] * kDegToRad ||
                    cycle.left.state.q_rad[i] !=
                        previous.left.measured_position_deg[i] * kDegToRad)
                    measured_matches = false;
            }
            Check(measured_matches,
                  at + ": measured state is the previous exchange reply");
            Check(std::abs(cycle.next_feedback.simulation_time_s -
                           static_cast<double>(tick + 1) * kControlDt) <
                      1e-12,
                  at + ": physics advanced exactly one tick after commands");

            // Ideal-mode invariants (pre-registered prediction 6).
            Check(cycle.right.overrun_count == 0 &&
                      cycle.left.overrun_count == 0,
                  at + ": overrun_count == 0");
            Check(!cycle.shared_stop, at + ": no stop in the ideal hold");

            first_run_right_commands.push_back(cycle.right.commanded_deg);
            previous = cycle.next_feedback;
        }

        // Reset requires a fresh Start, and a re-run reproduces the first
        // run bit-for-bit (references and integrators fully cleared).
        runner.Reset();
        threw = false;
        try {
            runner.Step();
        } catch (const std::logic_error&) {
            threw = true;
        }
        Check(threw, "Step after Reset (before fresh Start) throws");

        runner.Start();
        for (long tick = 0; tick < 5; ++tick) {
            const DualSimulationCycle cycle = runner.Step();
            Check(cycle.right.state.world_sequence ==
                      static_cast<std::uint64_t>(tick) + 1,
                  "post-reset cycle " + std::to_string(tick) +
                      ": sequence restarts at 1");
            Check(cycle.right.commanded_deg ==
                      first_run_right_commands[static_cast<size_t>(tick)],
                  "post-reset cycle " + std::to_string(tick) +
                      ": command frame bit-identical to the first run");
        }
    }

    // ---------------------------------------------------------------
    // Shared stop: one arm's verdict prevents both from advancing
    // ---------------------------------------------------------------
    void TestSharedStopLatch()
    {
        SimulationConfig config = BaseConfig();
        // Deterministic single-arm stop, now from the plant's own
        // physics: with the mocap Mount the arms are gravity-loaded, so
        // they sag away from the commanded posture the moment stepping
        // starts (a few 1e-3 deg within the first 2 ms tick, settling at
        // ~0.09-0.23 deg per joint). A 1e-4 deg following-error limit on
        // the RIGHT arm only (production left limit stays 3 deg, far
        // above the sag) therefore trips on cycle 1 — right stops, left
        // stays healthy. The superseded free-falling plant tracked to
        // float noise and needed an injected Mount twist to produce this
        // stop at all.
        config.right_execution.following_error_limit_deg = 1e-4;

        DualSimulationRunner runner(config);
        runner.Start();
        const DualSimulationCycle cycle = runner.Step();

        Check(cycle.right_stop.priority.reason ==
                  StopPriorityReason::kFollowingError,
              "right arm stops on following error");
        Check(!StopRequested(cycle.left_stop),
              "left arm is healthy on the same cycle");
        Check(cycle.shared_stop, "either arm's stop latches the shared stop");

        bool threw = false;
        try {
            runner.Step();
        } catch (const std::logic_error&) {
            threw = true;
        }
        Check(threw,
              "after the shared stop, neither arm's command can advance "
              "(Step throws)");

        // Reset clears the latch; a fresh Start runs again.
        runner.Reset();
        runner.Start();
        const DualSimulationCycle after = runner.Step();
        Check(after.right_stop.priority.reason ==
                  StopPriorityReason::kFollowingError,
              "the injected config survives Reset (same stop reproduces)");
    }

    // ---------------------------------------------------------------
    // Determinism: two identical runners, byte-identical traces
    // ---------------------------------------------------------------
    void TestDeterminism()
    {
        const SimulationConfig config = BaseConfig();
        DualSimulationRunner a(config);
        DualSimulationRunner b(config);
        a.Start();
        b.Start();
        for (long tick = 0; tick < 20; ++tick) {
            const DualSimulationCycle ca = a.Step();
            const DualSimulationCycle cb = b.Step();
            Check(ca.right.commanded_deg == cb.right.commanded_deg &&
                      ca.left.commanded_deg == cb.left.commanded_deg,
                  "determinism: commands identical at tick " +
                      std::to_string(tick));
            Check(ca.next_feedback.right.continuous_position_deg ==
                          cb.next_feedback.right.continuous_position_deg &&
                      ca.next_feedback.left.continuous_position_deg ==
                          cb.next_feedback.left.continuous_position_deg,
                  "determinism: feedback identical at tick " +
                      std::to_string(tick));
        }
    }

    // ---------------------------------------------------------------
    // Ideal-mode dual hold (0.5 s, 2 substeps)
    // ---------------------------------------------------------------
    void TestIdealDualHold()
    {
        SimulationConfig config = BaseConfig();
        config.physics_substeps = 2;
        DualSimulationRunner runner(config);
        runner.Start();

        // Sim-truth reference (contract section 4: simulator truth is
        // used only for simulation evaluation, never fed to the cores):
        // the TCP site world positions at the seed.
        const int right_site = runner.Backend().Contract().right.tcp_site_id;
        const int left_site = runner.Backend().Contract().left.tcp_site_id;
        const Eigen::Vector3d right_tcp_seed =
            SitePosition(runner, right_site);
        const Eigen::Vector3d left_tcp_seed = SitePosition(runner, left_site);

        const long cycles = 250; // 0.5 s at 500 Hz
        double max_est_error_m = 0.0;   // core estimate vs its reference
        double max_truth_drift_m = 0.0; // MuJoCo site vs seed position
        int max_ncon = 0;
        bool invariants_ok = true;
        bool stopped = false;
        for (long tick = 0; tick < cycles; ++tick) {
            const DualSimulationCycle cycle = runner.Step();
            if (cycle.shared_stop) {
                stopped = true;
                break;
            }
            if (cycle.right.overrun_count != 0 ||
                cycle.left.overrun_count != 0 ||
                !cycle.right.state.world_fresh ||
                !cycle.left.state.world_fresh || cycle.right.nonfinite ||
                cycle.left.nonfinite)
                invariants_ok = false;

            // The accepted analysis correction: contacts (inter-arm above
            // all) would produce cross-arm disturbance indistinguishable
            // from a Mount-representation error; the chosen falsifier is
            // ncon == 0 over the whole run.
            max_ncon = std::max(max_ncon,
                                static_cast<int>(runner.Backend().Data().ncon));

            const double right_est =
                (cycle.right.measured.ee_pose_world.position_m -
                 cycle.right.reference.ee_pose_world.position_m)
                    .norm();
            const double left_est =
                (cycle.left.measured.ee_pose_world.position_m -
                 cycle.left.reference.ee_pose_world.position_m)
                    .norm();
            max_est_error_m =
                std::max({max_est_error_m, right_est, left_est});

            const double right_truth =
                (SitePosition(runner, right_site) - right_tcp_seed).norm();
            const double left_truth =
                (SitePosition(runner, left_site) - left_tcp_seed).norm();
            max_truth_drift_m =
                std::max({max_truth_drift_m, right_truth, left_truth});
        }

        Check(!stopped, "dual hold: no stop over the run");
        Check(invariants_ok,
              "dual hold: overrun_count == 0, world_fresh, finite on every "
              "cycle (pre-registered prediction 6)");
        Check(max_ncon == 0,
              "dual hold: ncon == 0 over the whole run (chosen contact "
              "falsifier)");
        // Pre-registered prediction 5: transient servo-droop sag of
        // centimetre scale, converging under the outer loop — 0.05 m is
        // the sanity gate derived from that prediction, and the measured
        // bound is reported, not tuned. The mocap Mount rework made this
        // sag real: 0.000186 m on the free-falling freejoint plant,
        // 0.002295 m with gravity loading the arms (2026-08-17), inside
        // the 1-4 mm band the rework's analysis predicted.
        Check(max_est_error_m < 0.05,
              "dual hold: estimated TCP error within the centimetre-scale "
              "droop bound");
        Check(max_truth_drift_m < 0.05,
              "dual hold: sim-truth TCP drift within the same bound");
        std::printf("dual hold: max estimated TCP error %.6f m, "
                    "max sim-truth TCP drift %.6f m, max ncon %d\n",
                    max_est_error_m, max_truth_drift_m, max_ncon);
    }

    // ---------------------------------------------------------------
    // Scripted Mount motion through the runner (Plan 03 Task 1 wiring)
    // ---------------------------------------------------------------
    // Step() reads config.mount_motion, but every check above runs the
    // default kStatic Mount, so nothing registered stepped physics with a
    // Mount pose that CHANGES between ticks. This is that gate. Task 3
    // Step 1 still owns the full scenario test (fixed world TCP
    // references, per-arm-body carry, realistic sensing); what is fixed
    // here is that the wiring itself is no longer ungated.
    //
    // The oracle is not the code under test: SampleMountMotion is
    // recomputed here from the same config and time, and it is itself
    // gated independently by ctest mount_motion against Rodrigues and
    // rotation-log oracles. Three things are checked, none of which can
    // pass vacuously:
    //   (a) kStatic ignores the amplitude fields — a kStatic run with
    //       10x amplitudes reproduces the default static command trace
    //       BIT for bit ("disabling Mount motion reproduces the static
    //       trace");
    //   (b) the Mount state both cores receive at tick k is the sample
    //       at t_k = k*dt (position bit-equal; rotation to the
    //       quaternion round-trip; twist bit-equal, since ideal sensing
    //       copies it);
    //   (c) the pose the PLANT holds after tick k is that same pose, not
    //       tick k-1's. A one-tick-late Mount write is off by |v|*dt,
    //       6.3e-5 m at these defaults — seven orders above the 1e-12
    //       tolerance used, so the ordering is falsified, not assumed.
    // A motion-magnitude accumulator guards the whole loop against the
    // failure mode where the Mount silently never moves and every
    // difference is trivially zero.
    void TestScriptedMountWiring()
    {
        // (a) The static trace, from the default (kStatic) config.
        std::vector<JointVector> static_commands;
        {
            DualSimulationRunner runner(BaseConfig());
            runner.Start();
            for (long tick = 0; tick < 50; ++tick)
                static_commands.push_back(runner.Step().right.commanded_deg);
        }
        {
            SimulationConfig config = BaseConfig();
            config.mount_motion.kind = MountMotionKind::kStatic;
            config.mount_motion.translation_amplitude_m = 0.5; // 10x
            config.mount_motion.rotation_amplitude_rad = 1.0;  // 10x
            DualSimulationRunner runner(config);
            runner.Start();
            bool identical = true;
            for (long tick = 0; tick < 50; ++tick)
                identical =
                    identical && runner.Step().right.commanded_deg ==
                                     static_commands[static_cast<size_t>(tick)];
            Check(identical,
                  "kStatic ignores the amplitude fields: static command "
                  "trace reproduced bit-for-bit");
        }

        // (b) and (c), for every moving kind, at the shipped defaults
        // (0.05 m along +X, 0.1 rad about +Z, 0.1 Hz, phase 0).
        struct Case {
            MountMotionKind kind;
            const char* name;
        };
        const Case cases[] = {{MountMotionKind::kTranslation, "translation"},
                              {MountMotionKind::kRotation, "rotation"},
                              {MountMotionKind::kCombined, "combined"}};

        for (const Case& scenario : cases) {
            SimulationConfig config = BaseConfig();
            config.mount_motion.kind = scenario.kind;
            DualSimulationRunner runner(config);
            runner.Start();

            const DualModelContract& ids = runner.Backend().Contract();
            const int right_site = ids.right.tcp_site_id;
            const int left_site = ids.left.tcp_site_id;
            const Eigen::Vector3d right_tcp_seed =
                SitePosition(runner, right_site);
            const Eigen::Vector3d left_tcp_seed =
                SitePosition(runner, left_site);

            const long cycles = 250; // 0.5 s at 500 Hz
            double max_sample_pos_m = 0.0;
            double max_sample_rot = 0.0;
            double max_sample_twist = 0.0;
            double max_plant_pos_m = 0.0;
            double max_plant_quat = 0.0;
            double motion_magnitude = 0.0; // vacuity guard
            double max_tcp_drift_m = 0.0;
            bool both_arms_same_sample = true;
            bool stopped = false;
            for (long tick = 0; tick < cycles; ++tick) {
                const DualSimulationCycle cycle = runner.Step();
                if (cycle.shared_stop) {
                    stopped = true;
                    break;
                }
                const MountTruth truth = SampleMountMotion(
                    config.mount_motion, static_cast<double>(tick) * kControlDt,
                    config.world_T_mount_at_zero);

                // (b) what the cores were given.
                max_sample_pos_m =
                    std::max(max_sample_pos_m,
                             (cycle.right.state.world_p_mountseg -
                              truth.world_T_mount.position_m)
                                 .cwiseAbs()
                                 .maxCoeff());
                max_sample_rot = std::max(
                    max_sample_rot, (cycle.right.state.world_R_mountseg -
                                     truth.world_T_mount.rotation)
                                        .cwiseAbs()
                                        .maxCoeff());
                max_sample_twist = std::max(
                    {max_sample_twist,
                     (cycle.right.state.world_v_mountseg_m_s -
                      truth.world_V_mount.linear_m_s)
                         .cwiseAbs()
                         .maxCoeff(),
                     (cycle.right.state.world_w_mountseg_rad_s -
                      truth.world_V_mount.angular_rad_s)
                         .cwiseAbs()
                         .maxCoeff()});
                if (cycle.left.state.world_p_mountseg !=
                        cycle.right.state.world_p_mountseg ||
                    cycle.left.state.world_v_mountseg_m_s !=
                        cycle.right.state.world_v_mountseg_m_s ||
                    cycle.left.state.world_w_mountseg_rad_s !=
                        cycle.right.state.world_w_mountseg_rad_s ||
                    !cycle.left.state.world_mount_twist_valid ||
                    !cycle.right.state.world_mount_twist_valid)
                    both_arms_same_sample = false;

                // (c) what the plant holds after the exchange.
                const double* mocap_pos =
                    runner.Backend().Data().mocap_pos + 3 * ids.mount_mocap_id;
                const double* mocap_quat =
                    runner.Backend().Data().mocap_quat + 4 * ids.mount_mocap_id;
                max_plant_pos_m = std::max(
                    max_plant_pos_m,
                    (Eigen::Vector3d(mocap_pos[0], mocap_pos[1], mocap_pos[2]) -
                     truth.world_T_mount.position_m)
                        .cwiseAbs()
                        .maxCoeff());
                const Eigen::Quaterniond expected(truth.world_T_mount.rotation);
                const Eigen::Vector4d observed_quat(mocap_quat[0], mocap_quat[1],
                                                    mocap_quat[2],
                                                    mocap_quat[3]);
                const Eigen::Vector4d expected_quat(
                    expected.w(), expected.x(), expected.y(), expected.z());
                max_plant_quat =
                    std::max(max_plant_quat,
                             std::min((observed_quat - expected_quat).norm(),
                                      (observed_quat + expected_quat).norm()));

                // Vacuity guard: the Mount the cores saw must actually
                // leave the anchor. If the wiring silently held it still,
                // every difference above would be zero and this stays
                // zero too.
                motion_magnitude = std::max(
                    motion_magnitude,
                    (cycle.right.state.world_p_mountseg -
                     config.world_T_mount_at_zero.position_m)
                            .norm() +
                        (cycle.right.state.world_R_mountseg -
                         config.world_T_mount_at_zero.rotation)
                            .norm());

                // The point of the whole rig: the world hold keeps the
                // TCP world-fixed while the base moves under it.
                max_tcp_drift_m = std::max(
                    {max_tcp_drift_m,
                     (SitePosition(runner, right_site) - right_tcp_seed).norm(),
                     (SitePosition(runner, left_site) - left_tcp_seed).norm()});
            }

            const std::string at = std::string("moving Mount (") +
                                   scenario.name + ")";
            Check(!stopped, at + ": no stop over the run");
            Check(motion_magnitude > 1e-3,
                  at + ": the Mount actually left the anchor (vacuity guard)");
            Check(max_sample_pos_m == 0.0,
                  at + ": Mount position both cores saw is the sample, "
                       "bit-equal");
            Check(max_sample_rot < 1e-12,
                  at + ": Mount rotation both cores saw is the sample, to "
                       "the quaternion round-trip");
            Check(max_sample_twist == 0.0,
                  at + ": Mount twist both cores saw is the sample's, "
                       "bit-equal");
            Check(both_arms_same_sample,
                  at + ": both arms received the SAME Mount sample, valid "
                       "twist");
            Check(max_plant_pos_m == 0.0,
                  at + ": the plant holds THIS tick's Mount position, "
                       "bit-equal (a one-tick-late write is 6.3e-5 m off)");
            Check(max_plant_quat < 1e-12,
                  at + ": the plant holds THIS tick's Mount rotation");
            Check(max_tcp_drift_m < 0.05,
                  at + ": world hold keeps the TCP within the same "
                       "centimetre-scale bound as the static hold");
            std::printf("%s: max sample pose err %.3e m / %.3e, twist err "
                        "%.3e, plant pose err %.3e m / %.3e quat, Mount "
                        "excursion %.4f, max TCP drift %.6f m\n",
                        at.c_str(), max_sample_pos_m, max_sample_rot,
                        max_sample_twist, max_plant_pos_m, max_plant_quat,
                        motion_magnitude, max_tcp_drift_m);
        }
    }
} // namespace

int main()
{
    try {
        TestLifecycleAndCycleContract();
        TestSharedStopLatch();
        TestDeterminism();
        TestIdealDualHold();
        TestScriptedMountWiring();
    } catch (const std::exception& failure) {
        std::printf("FAIL: unexpected exception: %s\n", failure.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("test_dual_simulation_runner: all checks passed\n");
        return 0;
    }
    std::printf("test_dual_simulation_runner: %d check(s) failed\n", failures);
    return 1;
}
