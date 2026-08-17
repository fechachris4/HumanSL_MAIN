//
// Moving-Mount world-hold evidence (Plan 03 Task 3 Step 1, ideal sensing).
//
// THE PHYSICAL QUESTION. The wearer is the disturbance: the arm exists to
// keep its end effector still in Vicon world W while the backpack Mount
// translates and rotates under it. Everything registered before this file
// checked that the pipeline was WIRED correctly — that both cores see one
// coherent Mount sample per tick, that the plant holds this tick's pose
// and not the last one. None of it asked whether the resulting motion is
// the right motion. This file asks that.
//
// WHAT IS MEASURED, AND WHY IT IS INDEPENDENT OF THE CONTROLLER. Every
// number below is simulator truth (contract section 4): the MuJoCo TCP
// site position and the Mount pose read back from the plant's own mocap
// row. None of it comes from the core's Pinocchio estimate of where it
// thinks its TCP is, so a controller that were confidently wrong about
// its own kinematics could not pass by agreeing with itself.
//
// THE TWO NUMBERS, AND WHY BOTH ARE ASSERTED. For a TCP at world position
// p_W_E and a Mount at world_T_mount = (R, p_W_M), write the same point in
// the Mount frame:
//
//     p_M_E(t) = R(t)^T (p_W_E(t) - p_W_M(t))
//
// A world hold that works keeps p_W_E fixed, so p_M_E must MOVE, by
// exactly the Mount's own excursion (with the sign flipped). An arm that
// merely hangs off the base and does nothing shows the opposite pair:
// p_M_E constant, p_W_E swinging by the Mount's amplitude. The two failure
// modes are distinguished only by asserting BOTH, which is why each
// scenario checks
//
//   (1) max |p_W_E(t) - p_W_E(hold)| <= a bound derived below, and
//   (2) max |p_M_E(t) - p_M_E(0)| within that same bound of the excursion
//       the geometry predicts, and
//   (3) that the Mount-frame displacement is OPPOSITE the Mount's own
//       (a direction check, not a magnitude one: cos <= -0.99 for the
//       translation case, and a signed angle equal to -theta(t) for the
//       rotation case).
//
// WHERE THE BOUNDS COME FROM (accepted analysis packet, written before the
// runs; §5 and prediction 2/3). The hold reference is a fixed world pose
// with zero twist, so compensation is pure feedback and the quasi-steady
// world error obeys Kp * e ~ (base-induced TCP speed) / alpha, with
// Kp = kKpCartesian = 10 1/s and the damped-least-squares attenuation
// alpha = sigma^2/(sigma^2 + lambda^2) >= 0.90 in the positional
// directions at lambda = kDlsLambda = 0.1. On top of that sits the static
// gravity droop the mocap Mount rework made real and MEASURED BEFORE this
// file existed: 2.3 mm (README, "How the Mount is represented"). So
//
//   translation, A = 0.05 m at f = 0.1 Hz:
//     v_peak = A 2 pi f = 0.0314 m/s -> 3.5 mm, + 2.3 mm droop = 5.8 mm
//   rotation, A = 0.1 rad at f = 0.1 Hz, worst lever arm 0.739 m:
//     |omega| r = 0.0464 m/s        -> 5.2 mm, + 2.3 mm droop = 7.5 mm
//
// The gates are twice those predictions (12 mm and 15 mm), because the
// packet's magnitudes are honest to an order of magnitude only: sigma
// varies with configuration and the 6-D damped inverse couples the
// rotational and translational channels. The measured values are REPORTED
// by every run, never used to set the gate.
//
// WHAT THESE CHECKS CATCH, MEASURED (mutants compiled outside the
// repository against the same archives, 2026-08-17 — a test asserting its
// own author's work is not evidence on its own):
//
//   Mutation A, "the cores are told the Mount is static" (the sample is
//   built from SampleMountMotion(0) while the plant still moves): 13
//   checks fail. The world-hold error becomes 48.9 mm under translation
//   and 71.7 mm under rotation — the amplitude and the lever-arm chord,
//   i.e. the arm is simply carried by the base — while the Mount-frame
//   excursion collapses to 2.3 mm (the static droop, i.e. the TCP no
//   longer moves through the Mount frame at all) and the right/left
//   excursion ratio collapses from 1.134 to 1.027. That is the failure
//   mode this file exists to reject, and every one of the three
//   assertion families rejects it.
//
//   Mutation B, "the Mount twist is zeroed in the sample" (pose still
//   exact): NOT CAUGHT, stated here rather than papered over. The world
//   error grows from 3.70 to 5.12 mm (translation) and 4.82 to 7.11 mm
//   (rotation) — 1.38x and 1.48x, which is exactly the 1.3-1.5x the
//   accepted packet's negative control predicted for the Kd damping term
//   flipping from damping to disturbance — and both stay inside gates set
//   at twice the predicted peak. So these scenarios gate the POSE half of
//   the base-motion compensation, not the twist half. The packet
//   deliberately specified that negative control as a statement and not a
//   runtime switch, so no twist-disabling flag was added to production
//   code to make it gateable; the coupling that makes the twist right —
//   that it is the exact derivative of the pose being prescribed, from
//   one SampleMountMotion call — is gated instead by ctest mount_motion
//   (central finite differences of the pose) and ctest
//   dual_simulation_runner (the sample both cores receive is that call's
//   result, bit for bit).
//
// Evidence class: unit test / simulation, ideal sensing (exact pose and
// exact pose-derivative every 2 ms). It bounds the execution mathematics;
// it says nothing about Vicon, flex or the real servos.
//

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "DualSimulationRunner.h"
#include "MountMotion.h"
#include "SimulationConfig.h"

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

    // Pre-registered gates (derivation in the file header). Declared here,
    // above every run, so they are visibly independent of any result.
    constexpr double kStaticHoldBoundM = 0.010;      // 1-4 mm predicted
    constexpr double kTranslationHoldBoundM = 0.012; // 5.8 mm predicted
    constexpr double kRotationHoldBoundM = 0.015;    // 7.5 mm predicted

    SimulationConfig BaseConfig()
    {
        SimulationConfig config;
        config.model_xml_path = SIM_MODEL_XML_PATH;
        config.urdf_path = SIM_URDF_PATH;
        config.control_dt_s = kControlDt;
        config.physics_substeps = 1;
        // The identity anchor, exactly as humansl_sim runs it, so these
        // numbers and the CLI summary's are the same numbers.
        return config;
    }

    Eigen::Vector3d SitePosition(const DualSimulationRunner& runner,
                                 int site_id)
    {
        const mjData& data = runner.Backend().Data();
        return Eigen::Vector3d(data.site_xpos[3 * site_id + 0],
                               data.site_xpos[3 * site_id + 1],
                               data.site_xpos[3 * site_id + 2]);
    }

    // The Mount pose the PLANT is holding, from its own mocap row
    // (MuJoCo's w,x,y,z order). Deliberately read back from the plant
    // rather than recomputed: if the Mount write ever landed a tick late,
    // the Mount-frame numbers below would be computed in the frame the
    // physics actually used, and the excursion checks would see it.
    CartesianPose PlantMountPose(const DualSimulationRunner& runner)
    {
        const mjData& data = runner.Backend().Data();
        const int mocap_id = runner.Backend().Contract().mount_mocap_id;
        const double* position = data.mocap_pos + 3 * mocap_id;
        const double* quaternion = data.mocap_quat + 4 * mocap_id;
        CartesianPose pose;
        pose.position_m =
            Eigen::Vector3d(position[0], position[1], position[2]);
        pose.rotation = Eigen::Quaterniond(quaternion[0], quaternion[1],
                                           quaternion[2], quaternion[3])
                            .toRotationMatrix();
        return pose;
    }

    Eigen::Vector3d InMountFrame(const CartesianPose& world_T_mount,
                                 const Eigen::Vector3d& world_p)
    {
        return world_T_mount.rotation.transpose() *
               (world_p - world_T_mount.position_m);
    }

    // One arm's per-tick truth trace. Plain vectors: each scenario asks a
    // different question of the same trace, and the questions are clearer
    // written out than folded into accumulators.
    struct ArmTrace {
        Eigen::Vector3d world_tcp_seed;
        Eigen::Vector3d mount_tcp_seed;
        std::vector<Eigen::Vector3d> world_tcp;    // sim truth, per tick
        std::vector<Eigen::Vector3d> mount_tcp;    // p_M_E, per tick
    };

    struct RunTrace {
        ArmTrace right;
        ArmTrace left;
        CartesianPose mount_seed;
        std::vector<CartesianPose> mount_truth; // SampleMountMotion(t_k)
        bool stopped = false;
        bool invariants_ok = true;
        int max_ncon = 0;
    };

    // A run that stopped early leaves a short (possibly empty) trace, and
    // every check below would then be measuring nothing. The stop itself
    // is already reported as a failure; this keeps the arithmetic that
    // follows it from reading past the end of the trace.
    bool HasTrace(const RunTrace& trace, const char* what)
    {
        if (!trace.mount_truth.empty())
            return true;
        Check(false, std::string(what) + ": the run produced no cycles");
        return false;
    }

    // Steps the runner for `cycles` ticks and records simulator truth.
    RunTrace Run(const SimulationConfig& config, long cycles)
    {
        DualSimulationRunner runner(config);
        runner.Start();

        const DualModelContract& ids = runner.Backend().Contract();
        const int right_site = ids.right.tcp_site_id;
        const int left_site = ids.left.tcp_site_id;

        RunTrace trace;
        trace.mount_seed = PlantMountPose(runner);
        trace.right.world_tcp_seed = SitePosition(runner, right_site);
        trace.left.world_tcp_seed = SitePosition(runner, left_site);
        trace.right.mount_tcp_seed =
            InMountFrame(trace.mount_seed, trace.right.world_tcp_seed);
        trace.left.mount_tcp_seed =
            InMountFrame(trace.mount_seed, trace.left.world_tcp_seed);

        for (long tick = 0; tick < cycles; ++tick) {
            const DualSimulationCycle cycle = runner.Step();
            if (cycle.shared_stop) {
                trace.stopped = true;
                break;
            }
            if (cycle.right.overrun_count != 0 ||
                cycle.left.overrun_count != 0 ||
                !cycle.right.state.world_fresh ||
                !cycle.left.state.world_fresh || cycle.right.nonfinite ||
                cycle.left.nonfinite)
                trace.invariants_ok = false;
            trace.max_ncon =
                std::max(trace.max_ncon,
                         static_cast<int>(runner.Backend().Data().ncon));

            const CartesianPose mount_now = PlantMountPose(runner);
            const Eigen::Vector3d right_world =
                SitePosition(runner, right_site);
            const Eigen::Vector3d left_world = SitePosition(runner, left_site);
            trace.right.world_tcp.push_back(right_world);
            trace.left.world_tcp.push_back(left_world);
            trace.right.mount_tcp.push_back(
                InMountFrame(mount_now, right_world));
            trace.left.mount_tcp.push_back(InMountFrame(mount_now, left_world));
            trace.mount_truth.push_back(
                SampleMountMotion(config.mount_motion,
                                  static_cast<double>(tick) * kControlDt,
                                  config.world_T_mount_at_zero)
                    .world_T_mount);
        }
        return trace;
    }

    double MaxWorldHoldErrorM(const ArmTrace& arm)
    {
        double worst = 0.0;
        for (const Eigen::Vector3d& p : arm.world_tcp)
            worst = std::max(worst, (p - arm.world_tcp_seed).norm());
        return worst;
    }

    double MaxMountFrameExcursionM(const ArmTrace& arm)
    {
        double worst = 0.0;
        for (const Eigen::Vector3d& p : arm.mount_tcp)
            worst = std::max(worst, (p - arm.mount_tcp_seed).norm());
        return worst;
    }

    // ---------------------------------------------------------------
    // (a) Static regression: motion = static reproduces a bounded hold
    // ---------------------------------------------------------------
    // The base case the moving ones are read against. It is a regression
    // in the strict sense: the bound is the accepted packet's predicted
    // 1-4 mm gravity-droop transient, and the measured value existed
    // before this file (2.255 mm right / 1.732 mm left over the 2 s
    // acceptance run, README).
    void TestStaticHold()
    {
        SimulationConfig config = BaseConfig();
        config.mount_motion.kind = MountMotionKind::kStatic;
        const RunTrace trace = Run(config, 500); // 1.0 s

        Check(!trace.stopped, "static hold: no stop over the run");
        if (!HasTrace(trace, "static hold"))
            return;
        Check(trace.invariants_ok,
              "static hold: overrun_count == 0, world_fresh and finite on "
              "every cycle");
        Check(trace.max_ncon == 0, "static hold: ncon == 0 over the run");

        const double right = MaxWorldHoldErrorM(trace.right);
        const double left = MaxWorldHoldErrorM(trace.left);
        Check(right < kStaticHoldBoundM,
              "static hold: right world-hold TCP error within the droop "
              "bound");
        Check(left < kStaticHoldBoundM,
              "static hold: left world-hold TCP error within the droop "
              "bound");
        // With the Mount still, the Mount frame IS the world frame up to
        // a fixed transform, so the two excursions must coincide exactly.
        // (This is the statement the moving cases falsify.)
        Check(std::abs(MaxMountFrameExcursionM(trace.right) - right) < 1e-12 &&
                  std::abs(MaxMountFrameExcursionM(trace.left) - left) < 1e-12,
              "static hold: Mount-frame excursion equals the world error "
              "when the Mount does not move");
        std::printf("static hold: max world-hold TCP error %.6f m right, "
                    "%.6f m left (gate %.3f m)\n",
                    right, left, kStaticHoldBoundM);
    }

    // ---------------------------------------------------------------
    // (b) Translation hold
    // ---------------------------------------------------------------
    // Half a period (5 s at 0.1 Hz) covers s(t) = 0 -> A -> 0 with the
    // velocity peaking at both ends, so the run contains both the largest
    // displacement (which the excursion check needs) and the largest
    // base-induced TCP speed (which the world-error check needs).
    void TestTranslationHold()
    {
        SimulationConfig config = BaseConfig();
        config.mount_motion.kind = MountMotionKind::kTranslation;
        config.mount_motion.translation_axis_world = Eigen::Vector3d::UnitX();
        const double amplitude_m = config.mount_motion.translation_amplitude_m;
        const RunTrace trace = Run(config, 2500); // 5.0 s = half a period

        Check(!trace.stopped, "translation hold: no stop over the run");
        if (!HasTrace(trace, "translation hold"))
            return;
        Check(trace.invariants_ok,
              "translation hold: overrun_count == 0, world_fresh and finite "
              "on every cycle");
        Check(trace.max_ncon == 0,
              "translation hold: ncon == 0 over the run");

        // The Mount really did travel its amplitude (vacuity guard: every
        // check below is trivially satisfied by a Mount that never moved).
        double mount_excursion_m = 0.0;
        for (const CartesianPose& pose : trace.mount_truth)
            mount_excursion_m =
                std::max(mount_excursion_m,
                         (pose.position_m - trace.mount_seed.position_m).norm());
        Check(std::abs(mount_excursion_m - amplitude_m) < 1e-6,
              "translation hold: the Mount travelled its full amplitude");

        const ArmTrace* arms[2] = {&trace.right, &trace.left};
        const char* names[2] = {"right", "left"};
        for (int side = 0; side < 2; ++side) {
            const ArmTrace& arm = *arms[side];
            const std::string at =
                std::string("translation hold (") + names[side] + ")";

            // (1) The world hold held.
            const double world_error_m = MaxWorldHoldErrorM(arm);
            Check(world_error_m < kTranslationHoldBoundM,
                  at + ": max world-hold TCP error within the predicted "
                       "bound");

            // (2) The compensation actually happened: the TCP moved
            // through the Mount frame by the Mount's own amplitude. A
            // controller that ignored the base would leave this near 0.
            const double excursion_m = MaxMountFrameExcursionM(arm);
            Check(std::abs(excursion_m - amplitude_m) <
                      kTranslationHoldBoundM,
                  at + ": Mount-frame TCP excursion equals the Mount "
                       "amplitude to within the hold error");

            // (3) ... and in the OPPOSITE direction. The Mount rotation
            // is constant here, so the Mount-frame displacement maps to
            // world by the fixed seed rotation; the two vectors must be
            // antiparallel. Only ticks past half amplitude are scored:
            // near s(t) = 0 the direction of a millimetre-scale vector is
            // dominated by the droop, and asking for its sign would be
            // asking a question the physics does not answer.
            double worst_cos = -1.0;
            int scored = 0;
            for (size_t k = 0; k < arm.mount_tcp.size(); ++k) {
                const Eigen::Vector3d mount_step =
                    trace.mount_truth[k].position_m -
                    trace.mount_seed.position_m;
                if (mount_step.norm() < 0.5 * amplitude_m)
                    continue;
                const Eigen::Vector3d tcp_step =
                    trace.mount_seed.rotation *
                    (arm.mount_tcp[k] - arm.mount_tcp_seed);
                worst_cos =
                    std::max(worst_cos, tcp_step.normalized().dot(
                                            mount_step.normalized()));
                ++scored;
            }
            Check(scored > 100,
                  at + ": the phase check scored a meaningful number of "
                       "ticks");
            Check(worst_cos < -0.99,
                  at + ": the Mount-frame TCP displacement is opposite the "
                       "Mount's own displacement");
            std::printf("%s: world-hold error %.6f m (gate %.3f), "
                        "Mount-frame excursion %.6f m vs Mount amplitude "
                        "%.6f m, worst direction cos %.5f over %d ticks\n",
                        at.c_str(), world_error_m, kTranslationHoldBoundM,
                        excursion_m, amplitude_m, worst_cos, scored);
        }
    }

    // ---------------------------------------------------------------
    // (c) Rotation hold — the lever-arm case
    // ---------------------------------------------------------------
    // The Mount origin does not move at all, so every metre the TCP has
    // to travel comes from omega x (p_W_E - p_W_M). The predicted
    // Mount-frame excursion is a chord, 2 r_perp sin(theta_max / 2), with
    // r_perp the TCP's distance from the rotation axis THROUGH THE MOUNT
    // ORIGIN — measured here from the seed geometry rather than assumed,
    // and cross-checked against the accepted packet's 0.739 m (right) and
    // 0.650 m (left).
    void TestRotationHold()
    {
        SimulationConfig config = BaseConfig();
        config.mount_motion.kind = MountMotionKind::kRotation;
        config.mount_motion.rotation_axis_world = Eigen::Vector3d::UnitZ();
        const double amplitude_rad = config.mount_motion.rotation_amplitude_rad;
        const Eigen::Vector3d axis_world =
            config.mount_motion.rotation_axis_world.normalized();
        const RunTrace trace = Run(config, 2500); // 5.0 s = half a period

        Check(!trace.stopped, "rotation hold: no stop over the run");
        if (!HasTrace(trace, "rotation hold"))
            return;
        Check(trace.invariants_ok,
              "rotation hold: overrun_count == 0, world_fresh and finite on "
              "every cycle");
        Check(trace.max_ncon == 0, "rotation hold: ncon == 0 over the run");

        double mount_angle_rad = 0.0;
        for (const CartesianPose& pose : trace.mount_truth)
            mount_angle_rad = std::max(
                mount_angle_rad,
                Eigen::AngleAxisd(trace.mount_seed.rotation.transpose() *
                                  pose.rotation)
                    .angle());
        Check(std::abs(mount_angle_rad - amplitude_rad) < 1e-6,
              "rotation hold: the Mount turned through its full amplitude");
        Check((trace.mount_truth.back().position_m -
               trace.mount_seed.position_m)
                      .norm() == 0.0,
              "rotation hold: the Mount origin did not translate (the "
              "rotation is about an axis through it)");

        // The rotation axis in the Mount frame: the Mount-frame TCP must
        // turn about THIS by -theta(t).
        const Eigen::Vector3d axis_mount =
            trace.mount_seed.rotation.transpose() * axis_world;

        double lever_arm_m[2] = {0.0, 0.0};
        double excursion_m[2] = {0.0, 0.0};
        double world_error_m[2] = {0.0, 0.0};
        const ArmTrace* arms[2] = {&trace.right, &trace.left};
        const char* names[2] = {"right", "left"};
        for (int side = 0; side < 2; ++side) {
            const ArmTrace& arm = *arms[side];
            const std::string at =
                std::string("rotation hold (") + names[side] + ")";

            const Eigen::Vector3d r_mount =
                arm.mount_tcp_seed; // Mount origin to TCP, Mount axes
            const Eigen::Vector3d r_perp =
                r_mount - r_mount.dot(axis_mount) * axis_mount;
            lever_arm_m[side] = r_perp.norm();

            // (1) The world hold held, against a bound whose lever-arm
            // term is this arm's own measured r_perp.
            world_error_m[side] = MaxWorldHoldErrorM(arm);
            Check(world_error_m[side] < kRotationHoldBoundM,
                  at + ": max world-hold TCP error within the predicted "
                       "bound");

            // (2) The chord the geometry demands.
            excursion_m[side] = MaxMountFrameExcursionM(arm);
            const double predicted_chord_m =
                2.0 * lever_arm_m[side] * std::sin(0.5 * amplitude_rad);
            Check(std::abs(excursion_m[side] - predicted_chord_m) <
                      kRotationHoldBoundM,
                  at + ": Mount-frame TCP excursion equals the lever-arm "
                       "chord to within the hold error");

            // (3) ... swept the opposite way: the Mount-frame TCP turns
            // about the axis by exactly -theta(t). The angle tolerance is
            // the position bound divided by the lever arm — the same
            // error, expressed as the angle it subtends.
            const double angle_tolerance_rad =
                kRotationHoldBoundM / lever_arm_m[side];
            double worst_angle_residual_rad = 0.0;
            int scored = 0;
            for (size_t k = 0; k < arm.mount_tcp.size(); ++k) {
                const double theta_rad =
                    Eigen::AngleAxisd(trace.mount_seed.rotation.transpose() *
                                      trace.mount_truth[k].rotation)
                        .angle();
                if (theta_rad < 0.5 * amplitude_rad)
                    continue;
                // Signed angle from the seed radius to the current one,
                // about axis_mount (atan2 of the cross and dot products).
                const Eigen::Vector3d now = arm.mount_tcp[k];
                const Eigen::Vector3d now_perp =
                    now - now.dot(axis_mount) * axis_mount;
                const double swept_rad = std::atan2(
                    axis_mount.dot(r_perp.cross(now_perp)),
                    r_perp.dot(now_perp));
                worst_angle_residual_rad =
                    std::max(worst_angle_residual_rad,
                             std::abs(swept_rad + theta_rad));
                ++scored;
            }
            Check(scored > 100,
                  at + ": the swept-angle check scored a meaningful number "
                       "of ticks");
            Check(worst_angle_residual_rad < angle_tolerance_rad,
                  at + ": the Mount-frame TCP swept -theta(t), opposite the "
                       "Mount's own rotation");
            std::printf("%s: lever arm %.4f m, world-hold error %.6f m "
                        "(gate %.3f), Mount-frame excursion %.6f m vs chord "
                        "%.6f m, worst swept-angle residual %.5f rad (tol "
                        "%.5f) over %d ticks\n",
                        at.c_str(), lever_arm_m[side], world_error_m[side],
                        kRotationHoldBoundM, excursion_m[side],
                        predicted_chord_m, worst_angle_residual_rad,
                        angle_tolerance_rad, scored);
        }

        // The packet's left/right asymmetry, which a translation scenario
        // cannot show at all: the two arms sit at different distances from
        // the rotation axis, so the same Mount rotation demands different
        // TCP motion of each. The EXCURSION ratio is asserted because it
        // is a kinematic identity (it must equal the lever-arm ratio); the
        // ERROR ratio is only reported, because it also depends on each
        // arm's Jacobian conditioning, which the packet bounds to an order
        // of magnitude and not better.
        Check(std::abs(lever_arm_m[0] - 0.739) < 0.005 &&
                  std::abs(lever_arm_m[1] - 0.650) < 0.005,
              "rotation hold: measured lever arms match the accepted "
              "packet's 0.739 m / 0.650 m");
        const double excursion_ratio = excursion_m[0] / excursion_m[1];
        const double lever_ratio = lever_arm_m[0] / lever_arm_m[1];
        Check(std::abs(excursion_ratio - lever_ratio) < 0.02,
              "rotation hold: the right/left excursion ratio is the "
              "lever-arm ratio (the asymmetry is real, not an artifact)");
        std::printf("rotation hold: excursion ratio right/left %.4f vs "
                    "lever-arm ratio %.4f; world-error ratio %.4f "
                    "(reported, not gated)\n",
                    excursion_ratio, lever_ratio,
                    world_error_m[0] / world_error_m[1]);
    }
} // namespace

int main()
{
    try {
        TestStaticHold();
        TestTranslationHold();
        TestRotationHold();
    } catch (const std::exception& failure) {
        std::printf("FAIL: unexpected exception: %s\n", failure.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("test_moving_mount_control: all checks passed\n");
        return 0;
    }
    std::printf("test_moving_mount_control: %d check(s) failed\n", failures);
    return 1;
}
