//
// humansl_sim — the ideal-mode entry point (Plan 02 Task 5; scripted-Mount
// CLI from Plan 03 Task 3 Step 4). Owns everything the coordinator must
// not: argv parsing, the run length, the run loop, all terminal output and
// the run-provenance summary, and the optional passive viewer. The
// coordinator's Step() stays free of I/O.
//
// Usage:
//   humansl_sim [--headless] [--viewer] [--duration-s <s>]
//               [--physics-substeps <n>]
//               [--motion static|translation|rotation|combined]
//               [--motion-axis x|y|z | --motion-axis <x> <y> <z>]
//               [--amplitude-m <m>] [--amplitude-rad <rad>]
//               [--frequency-hz <hz>] [--phase-rad <rad>]
//
// Headless is the default and the gate; --viewer requires a build with
// -DHUMANSL_SIM_VIEWER=ON (vendored GLFW + system X11/GL) and renders
// read-only between control ticks, never altering simulation or control
// time.
//
// Run length. A headless run must say how long it is: it produces a
// number, and a number needs a stated horizon. Under the viewer the run
// is being watched rather than measured, so an absent or zero
// --duration-s means "run until the window closes"; SIGINT (Ctrl-C) ends
// any run cleanly at a tick boundary and still prints the summary.
//
// The Mount motion is scripted and analytic (src/MountMotion.h), so there
// is no seed and no random state anywhere in this program: the same
// command line produces a byte-identical summary, which ctest `sim_cli`
// checks by running the binary twice.
//
// Exit status: 0 = the run ended without a stop and without contact
// (including an operator interrupt or a closed viewer window); 1 = a stop
// verdict or a contact ended the run early, or the model/configuration was
// rejected at startup; 2 = bad command line.
//
// Evidence class of everything this prints: simulation, on generic
// position servos (kp/kv from the mechanics source are declared generic,
// not Kinova parameters — model/model_provenance.yaml).
//

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "DualSimulationRunner.h"
#include "MountMotion.h"
#include "SimulationConfig.h"
#include "StopPriority.h"

#ifdef HUMANSL_SIM_VIEWER
#include "SimViewer.h"
#endif

namespace
{
    // Set from the SIGINT handler only; read by the run loop. The handler
    // does nothing else, which is the only thing it is allowed to do.
    volatile std::sig_atomic_t g_interrupted = 0;

    extern "C" void OnInterrupt(int) { g_interrupted = 1; }

    const char* StopReasonName(StopPriorityReason reason)
    {
        switch (reason) {
        case StopPriorityReason::kNone: return "none";
        case StopPriorityReason::kRobotFault: return "robot fault";
        case StopPriorityReason::kFollowingError: return "following error";
        case StopPriorityReason::kLeftLowLevel: return "left low-level";
        case StopPriorityReason::kJointLimitWarning:
            return "joint limit warning";
        case StopPriorityReason::kStaleFeedback: return "stale feedback";
        }
        return "unknown";
    }

    std::string DescribeStop(const ExecutionStopDecision& decision)
    {
        if (decision.priority.reason != StopPriorityReason::kNone)
            return StopReasonName(decision.priority.reason);
        if (decision.nonfinite_stop)
            return "non-finite command counter";
        if (decision.overrun_stop)
            return "overrun counter";
        return "none";
    }

    const char* MotionKindName(MountMotionKind kind)
    {
        switch (kind) {
        case MountMotionKind::kStatic: return "static";
        case MountMotionKind::kTranslation: return "translation";
        case MountMotionKind::kRotation: return "rotation";
        case MountMotionKind::kCombined: return "combined";
        }
        return "unknown";
    }

    bool UsesTranslation(MountMotionKind kind)
    {
        return kind == MountMotionKind::kTranslation ||
               kind == MountMotionKind::kCombined;
    }

    bool UsesRotation(MountMotionKind kind)
    {
        return kind == MountMotionKind::kRotation ||
               kind == MountMotionKind::kCombined;
    }

    // The base-motion (fictitious) acceleration the mocap Mount does NOT
    // apply to the links, for THIS run's resolved motion.
    //
    // A mocap body is teleported, so MuJoCo sees zero Mount velocity: the
    // transport terms that a truly accelerating base would impose on every
    // link — m a_base, the Euler term alpha x r, and the centrifugal term
    // omega x (omega x r) — are absent from the plant's dynamics. Their
    // size relative to gravity is what says whether the omission is a
    // footnote or the dominant physics of the run, so it must be computed
    // from the flags actually in force, never quoted from the defaults.
    //
    // With s(t) = A sin(w t + phi), w = 2 pi f:  |s''|max = A w^2 and
    // |s'|max = A w. Hence, per active channel and at lever arm r,
    //   translation: A_t w^2
    //   rotation:    A_r w^2 r  (Euler)  +  (A_r w)^2 r  (centrifugal)
    // The three peak at different instants, so their sum is an upper
    // bound, and r is the FULL distance from the Mount rather than the
    // perpendicular one, which keeps the rotation terms an upper bound too.
    double PeakUnmodelledBaseAccel(const MountMotionConfig& motion,
                                   double lever_arm_m)
    {
        const double w = 2.0 * M_PI * motion.frequency_hz;
        double accel = 0.0;
        if (UsesTranslation(motion.kind))
            accel += std::abs(motion.translation_amplitude_m) * w * w;
        if (UsesRotation(motion.kind)) {
            const double amplitude = std::abs(motion.rotation_amplitude_rad);
            accel += amplitude * w * w * lever_arm_m;
            accel += amplitude * amplitude * w * w * lever_arm_m;
        }
        return accel;
    }

    Eigen::Vector3d SitePosition(const mjData& data, int site_id)
    {
        return Eigen::Vector3d(data.site_xpos[3 * site_id + 0],
                               data.site_xpos[3 * site_id + 1],
                               data.site_xpos[3 * site_id + 2]);
    }

    // The Mount pose the PLANT is holding, read back from its own mocap
    // rows (MuJoCo's w,x,y,z quaternion order — ModelContract.h). Used
    // only to express sim-truth TCP positions in the Mount frame for the
    // summary; the controller never sees any of this (contract section 4).
    CartesianPose PlantMountPose(const mjData& data, int mocap_id)
    {
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

    // p_M_E: a world TCP position expressed in the Mount frame. Under a
    // perfect world hold with a moving Mount this is what MOVES, by the
    // Mount's own excursion, while the world position stays put.
    Eigen::Vector3d InMountFrame(const CartesianPose& world_T_mount,
                                 const Eigen::Vector3d& world_p)
    {
        return world_T_mount.rotation.transpose() *
               (world_p - world_T_mount.position_m);
    }

    // --- argv parsing -------------------------------------------------
    // Whole-token numeric parsing: "--amplitude-m 0.05x" and
    // "--amplitude-m none" are command-line errors, not a silent 0.0. A
    // silently-zeroed amplitude would produce a stationary run wearing a
    // "translation" label in its own summary, which is the one CLI
    // mistake that fabricates evidence rather than just failing.
    bool ParseDouble(const char* text, double& out)
    {
        char* end = nullptr;
        const double value = std::strtod(text, &end);
        if (end == text || *end != '\0' || !std::isfinite(value))
            return false;
        out = value;
        return true;
    }

    bool ParseInt(const char* text, int& out)
    {
        char* end = nullptr;
        const long value = std::strtol(text, &end, 10);
        if (end == text || *end != '\0')
            return false;
        out = static_cast<int>(value);
        return true;
    }

    int Usage(const char* argv0, const std::string& problem)
    {
        if (!problem.empty())
            std::fprintf(stderr, "humansl_sim: %s\n", problem.c_str());
        std::fprintf(
            stderr,
            "usage: %s [--headless] [--viewer] [--duration-s <s>]\n"
            "          [--physics-substeps <n>]\n"
            "          [--motion static|translation|rotation|combined]\n"
            "          [--motion-axis x|y|z | --motion-axis <x> <y> <z>]\n"
            "          [--amplitude-m <m>] [--amplitude-rad <rad>]\n"
            "          [--frequency-hz <hz>] [--phase-rad <rad>]\n"
            "\n"
            "  --duration-s   headless: required to be positive (default "
            "2 s if omitted).\n"
            "                 --viewer: omit it or pass 0 to run until the "
            "window closes.\n"
            "  --motion-axis  applies to the channels --motion selects; "
            "with 'combined'\n"
            "                 it sets BOTH axes (the preamble prints what "
            "was used).\n",
            argv0);
        return 2;
    }

    // Per-arm accumulators over the run — class A diagnostics (contract
    // section 10); the run's pass criterion is only "no stop, no contact".
    //
    // "est" is the core's own estimate of its TCP against the world pose
    // it captured as the hold; "world hold error" and "Mount-frame
    // excursion" are simulator truth (the MuJoCo TCP site), which is why
    // they can be trusted as independent evidence that the compensation
    // happened: under a moving Mount a correct world hold makes the first
    // small and the second large, and dropping the base-motion term makes
    // them swap.
    struct ArmRunStats {
        double max_est_position_error_m = 0.0;
        double max_est_orientation_error_rad = 0.0;
        double max_world_hold_error_m = 0.0;     // sim truth vs seed pose
        double max_mount_frame_excursion_m = 0.0; // sim truth, in Mount frame
    };

    void Accumulate(ArmRunStats& stats, const ArmExecutionResult& result,
                    const Eigen::Vector3d& world_tcp_now,
                    const Eigen::Vector3d& world_tcp_seed,
                    const Eigen::Vector3d& mount_tcp_now,
                    const Eigen::Vector3d& mount_tcp_seed)
    {
        stats.max_est_position_error_m = std::max(
            stats.max_est_position_error_m,
            (result.measured.ee_pose_world.position_m -
             result.reference.ee_pose_world.position_m)
                .norm());
        if (std::isfinite(result.controller_status.rot_error_rad))
            stats.max_est_orientation_error_rad =
                std::max(stats.max_est_orientation_error_rad,
                         result.controller_status.rot_error_rad);
        stats.max_world_hold_error_m =
            std::max(stats.max_world_hold_error_m,
                     (world_tcp_now - world_tcp_seed).norm());
        stats.max_mount_frame_excursion_m =
            std::max(stats.max_mount_frame_excursion_m,
                     (mount_tcp_now - mount_tcp_seed).norm());
    }

    void PrintPreamble(const SimulationConfig& config, double control_dt_s,
                       double timestep_s, long cycles, bool unbounded,
                       double lever_arm_m, double gravity_m_s2)
    {
        const MountMotionConfig& motion = config.mount_motion;
        std::printf("humansl_sim run configuration (evidence class: "
                    "simulation)\n");
        std::printf("  model: %s\n", config.model_xml_path.c_str());
        std::printf("  kinematic authority (URDF): %s\n",
                    config.urdf_path.c_str());
        std::printf("  control tick: %.6f s (500 Hz), physics substeps: %d "
                    "(timestep %.6f s)\n",
                    control_dt_s, config.physics_substeps, timestep_s);
        if (unbounded)
            std::printf("  run length: until the viewer window closes or "
                        "SIGINT\n");
        else
            std::printf("  run length: %ld cycles (%.3f s)\n", cycles,
                        static_cast<double>(cycles) * control_dt_s);

        const Eigen::Vector3d& anchor =
            config.world_T_mount_at_zero.position_m;
        std::printf("  Mount anchor (zero-displacement pose, world m): "
                    "(%.4f, %.4f, %.4f)\n",
                    anchor.x(), anchor.y(), anchor.z());
        std::printf("  Mount motion: %s\n", MotionKindName(motion.kind));
        if (UsesTranslation(motion.kind))
            std::printf("    translation: %.4f m along world axis "
                        "(%.3f, %.3f, %.3f) [normalized before use]\n",
                        motion.translation_amplitude_m,
                        motion.translation_axis_world.x(),
                        motion.translation_axis_world.y(),
                        motion.translation_axis_world.z());
        else
            std::printf("    translation: inactive (this kind does not "
                        "consult the translation amplitude or axis)\n");
        if (UsesRotation(motion.kind))
            std::printf("    rotation: %.4f rad about world axis "
                        "(%.3f, %.3f, %.3f) through the Mount's own "
                        "origin\n",
                        motion.rotation_amplitude_rad,
                        motion.rotation_axis_world.x(),
                        motion.rotation_axis_world.y(),
                        motion.rotation_axis_world.z());
        else
            std::printf("    rotation: inactive (this kind does not consult "
                        "the rotation amplitude or axis)\n");
        if (motion.kind == MountMotionKind::kStatic)
            std::printf("    the Mount stands at the anchor; its twist is "
                        "the exact derivative of a constant pose: zero\n");
        else
            std::printf("    s(t) = A sin(2 pi f t + phi), f = %.4f Hz, "
                        "phi = %.4f rad; A is the PEAK displacement and "
                        "phi != 0 starts the run displaced by A sin(phi)\n",
                        motion.frequency_hz, motion.phase_rad);

        std::printf("  sensing: IDEAL — the exact Mount pose and the exact "
                    "derivative of that pose, every 2 ms tick, age 0. Not "
                    "100 Hz Vicon, no latency, no noise, no dropout.\n");
        std::printf("  plant: generic position servos (kp/kv are generic "
                    "mechanics, not a Kinova servo model)\n");
        std::printf("  Mount representation: a world-welded MOCAP body. The "
                    "arms hang from it under full gravity and are carried "
                    "wherever it goes, but it has infinite effective mass — "
                    "no reaction force reaches the wearer, and because the "
                    "physics sees no Mount velocity, the base-motion "
                    "fictitious forces on the links are not modelled. See "
                    "README, \"Known representation limits\".\n");
        // The size of that omission is a property of THIS run's flags, so
        // it is computed here rather than quoted: a scenario that leaves
        // the defaults changes it by f^2, and a stale figure in the
        // preamble is a wrong quantity sitting inside the evidence.
        const double unmodelled_accel =
            PeakUnmodelledBaseAccel(motion, lever_arm_m);
        const double fraction_of_g =
            gravity_m_s2 > 0.0 ? unmodelled_accel / gravity_m_s2 : 0.0;
        if (unmodelled_accel == 0.0)
            std::printf("    size of that omission on THIS run: exactly "
                        "zero — the Mount does not accelerate.\n");
        else
            std::printf("    size of that omission on THIS run: at most "
                        "%.4f m/s^2, %.2f%% of g (upper bound A_t w^2 + "
                        "A_r w^2 r (1 + A_r), inactive channels "
                        "contributing zero, with w = 2 pi f = %.4f rad/s, "
                        "r = %.4f m the farther TCP's distance from the "
                        "Mount at the seed, g = %.3f m/s^2 from the model; "
                        "it grows with f^2).\n",
                        unmodelled_accel, 100.0 * fraction_of_g,
                        2.0 * M_PI * motion.frequency_hz, lever_arm_m,
                        gravity_m_s2);
        // A tenth of g is where the omitted term stops being small beside
        // the gravity load the plant does model. The threshold only
        // changes how loudly the same number is read; nothing is rejected
        // and no run is refused.
        if (fraction_of_g >= 0.10)
            std::printf("    READ THIS RUN WITH CARE: at %.0f%% of g the "
                        "unmodelled base-motion term is a leading part of "
                        "the physics, not a footnote. The numbers below "
                        "still bound the execution mathematics, but they "
                        "are not a claim about how the real arms would "
                        "load at this amplitude and frequency.\n",
                        100.0 * fraction_of_g);
        std::printf("  reference: world hold — each core captures its TCP "
                    "world pose on the first fresh sample and regulates to "
                    "it. No trajectory is published.\n");
        std::printf("  what the errors below are NOT: they are simulation "
                    "on a generic-servo plant with exact sensing and exact "
                    "calibration, so they bound the execution mathematics, "
                    "never the real rig's Vicon, flex or servo response.\n");
        std::fflush(stdout);
    }
} // namespace

int main(int argc, char** argv)
{
    SimulationConfig config;
    config.model_xml_path = SIM_MODEL_XML_PATH;
    config.urdf_path = SIM_URDF_PATH;

    double duration_s = 2.0; // headless default when --duration-s is absent
    bool duration_given = false;
    Eigen::Vector3d motion_axis = Eigen::Vector3d::Zero();
    bool motion_axis_given = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--headless") {
            config.headless = true;
        } else if (arg == "--viewer") {
            config.headless = false;
        } else if (arg == "--duration-s" && i + 1 < argc) {
            if (!ParseDouble(argv[++i], duration_s))
                return Usage(argv[0], "--duration-s needs a finite number");
            duration_given = true;
        } else if (arg == "--physics-substeps" && i + 1 < argc) {
            if (!ParseInt(argv[++i], config.physics_substeps))
                return Usage(argv[0], "--physics-substeps needs an integer");
        } else if (arg == "--motion" && i + 1 < argc) {
            const std::string kind = argv[++i];
            if (kind == "static")
                config.mount_motion.kind = MountMotionKind::kStatic;
            else if (kind == "translation")
                config.mount_motion.kind = MountMotionKind::kTranslation;
            else if (kind == "rotation")
                config.mount_motion.kind = MountMotionKind::kRotation;
            else if (kind == "combined")
                config.mount_motion.kind = MountMotionKind::kCombined;
            else
                return Usage(argv[0], "--motion must be one of static, "
                                      "translation, rotation, combined");
        } else if (arg == "--motion-axis" && i + 1 < argc) {
            const std::string first = argv[++i];
            if (first == "x")
                motion_axis = Eigen::Vector3d::UnitX();
            else if (first == "y")
                motion_axis = Eigen::Vector3d::UnitY();
            else if (first == "z")
                motion_axis = Eigen::Vector3d::UnitZ();
            else {
                double x = 0.0, y = 0.0, z = 0.0;
                if (i + 2 >= argc || !ParseDouble(first.c_str(), x) ||
                    !ParseDouble(argv[i + 1], y) ||
                    !ParseDouble(argv[i + 2], z))
                    return Usage(argv[0],
                                 "--motion-axis needs x, y, z or three "
                                 "finite numbers");
                i += 2;
                motion_axis = Eigen::Vector3d(x, y, z);
            }
            motion_axis_given = true;
        } else if (arg == "--amplitude-m" && i + 1 < argc) {
            if (!ParseDouble(argv[++i],
                             config.mount_motion.translation_amplitude_m))
                return Usage(argv[0], "--amplitude-m needs a finite number");
        } else if (arg == "--amplitude-rad" && i + 1 < argc) {
            if (!ParseDouble(argv[++i],
                             config.mount_motion.rotation_amplitude_rad))
                return Usage(argv[0],
                             "--amplitude-rad needs a finite number");
        } else if (arg == "--frequency-hz" && i + 1 < argc) {
            if (!ParseDouble(argv[++i], config.mount_motion.frequency_hz))
                return Usage(argv[0],
                             "--frequency-hz needs a finite number");
        } else if (arg == "--phase-rad" && i + 1 < argc) {
            if (!ParseDouble(argv[++i], config.mount_motion.phase_rad))
                return Usage(argv[0], "--phase-rad needs a finite number");
        } else {
            return Usage(argv[0], "unrecognised argument: " + arg);
        }
    }

    // One axis flag, applied to whichever channels the chosen kind uses;
    // for `combined` that is both, which the preamble prints so the run
    // never has to be guessed at from the command line alone. Inactive
    // channels keep their defaults and are reported as inactive rather
    // than rejected — one place (the preamble) states what the run
    // actually does, instead of a branch per unused combination.
    if (motion_axis_given) {
        if (UsesTranslation(config.mount_motion.kind))
            config.mount_motion.translation_axis_world = motion_axis;
        if (UsesRotation(config.mount_motion.kind))
            config.mount_motion.rotation_axis_world = motion_axis;
    }

    // The motion is validated here, at the argv boundary, by the same
    // function SampleMountMotion uses — a bad command line exits 2 rather
    // than surfacing as a runtime exception from inside Start().
    try {
        ValidateMountMotionConfig(config.mount_motion);
    } catch (const std::exception& rejected) {
        return Usage(argv[0], rejected.what());
    }

#ifndef HUMANSL_SIM_VIEWER
    if (!config.headless)
        return Usage(argv[0],
                     "built without the viewer; reconfigure with "
                     "-DHUMANSL_SIM_VIEWER=ON or run --headless");
#endif

    // Run length. Watching (viewer) may be open-ended; measuring
    // (headless) must state its horizon.
    const bool unbounded =
        !config.headless && (!duration_given || duration_s == 0.0);
    if (!unbounded && !(duration_s > 0.0))
        return Usage(argv[0],
                     "a headless run needs a positive --duration-s (only "
                     "--viewer may run unbounded, with --duration-s 0 or "
                     "omitted)");
    const long cycles =
        unbounded ? 0 : std::lround(duration_s / config.control_dt_s);
    if (!unbounded && cycles <= 0)
        return Usage(argv[0], "--duration-s is shorter than one 0.002 s "
                              "control tick");

    std::signal(SIGINT, OnInterrupt);

    try {
        DualSimulationRunner runner(config);
        runner.Start();

        const DualModelContract& ids = runner.Backend().Contract();
        const mjData& seed_data = runner.Backend().Data();
        // Simulator truth at the seed: the world TCP positions the world
        // hold is about to be captured at, and the same points expressed
        // in the Mount frame (contract section 4 — evaluation only).
        const Eigen::Vector3d right_tcp_seed =
            SitePosition(seed_data, ids.right.tcp_site_id);
        const Eigen::Vector3d left_tcp_seed =
            SitePosition(seed_data, ids.left.tcp_site_id);
        const CartesianPose mount_seed =
            PlantMountPose(seed_data, ids.mount_mocap_id);
        const Eigen::Vector3d right_tcp_seed_mount =
            InMountFrame(mount_seed, right_tcp_seed);
        const Eigen::Vector3d left_tcp_seed_mount =
            InMountFrame(mount_seed, left_tcp_seed);

        // The preamble is printed after the seed because one of its
        // numbers is measured rather than configured: the lever arm the
        // Mount's rotation acts through, taken as the farther TCP's
        // distance from the Mount in the seeded posture (sim truth, the
        // same MuJoCo sites the summary reads). Gravity likewise comes
        // from the model rather than from a literal.
        const double lever_arm_m =
            std::max((right_tcp_seed - mount_seed.position_m).norm(),
                     (left_tcp_seed - mount_seed.position_m).norm());
        const double gravity_m_s2 =
            Eigen::Map<const Eigen::Vector3d>(runner.Backend().Model().opt.gravity)
                .norm();
        PrintPreamble(config, config.control_dt_s,
                      runner.Backend().timestep_s(), cycles, unbounded,
                      lever_arm_m, gravity_m_s2);

#ifdef HUMANSL_SIM_VIEWER
        std::unique_ptr<SimViewer> viewer;
        if (!config.headless)
            viewer = std::make_unique<SimViewer>(runner.Backend().Model());
#endif

        ArmRunStats right_stats, left_stats;
        long completed = 0;
        int max_ncon = 0;
        bool all_world_fresh = true;
        int max_overrun_count = 0;
        double max_mount_translation_m = 0.0;
        double max_mount_rotation_rad = 0.0;
        std::string stop_cause;
        bool interrupted = false;
        bool window_closed = false;

        for (long tick = 0; unbounded || tick < cycles; ++tick) {
            if (g_interrupted) {
                interrupted = true;
                break;
            }
            const DualSimulationCycle cycle = runner.Step();
            ++completed;

            const mjData& data = runner.Backend().Data();
            const CartesianPose mount_now =
                PlantMountPose(data, ids.mount_mocap_id);
            Accumulate(right_stats, cycle.right,
                       SitePosition(data, ids.right.tcp_site_id),
                       right_tcp_seed,
                       InMountFrame(mount_now,
                                    SitePosition(data, ids.right.tcp_site_id)),
                       right_tcp_seed_mount);
            Accumulate(left_stats, cycle.left,
                       SitePosition(data, ids.left.tcp_site_id), left_tcp_seed,
                       InMountFrame(mount_now,
                                    SitePosition(data, ids.left.tcp_site_id)),
                       left_tcp_seed_mount);
            max_mount_translation_m =
                std::max(max_mount_translation_m,
                         (mount_now.position_m - mount_seed.position_m).norm());
            max_mount_rotation_rad = std::max(
                max_mount_rotation_rad,
                Eigen::AngleAxisd(mount_seed.rotation.transpose() *
                                  mount_now.rotation)
                    .angle());
            max_ncon = std::max(max_ncon, static_cast<int>(data.ncon));
            all_world_fresh = all_world_fresh &&
                              cycle.right.state.world_fresh &&
                              cycle.left.state.world_fresh;
            max_overrun_count =
                std::max({max_overrun_count, cycle.right.overrun_count,
                          cycle.left.overrun_count});

            if (cycle.shared_stop) {
                stop_cause = "right: " + DescribeStop(cycle.right_stop) +
                             ", left: " + DescribeStop(cycle.left_stop);
                break;
            }

#ifdef HUMANSL_SIM_VIEWER
            // Render read-only roughly at display rate; a lagging or
            // closed window never alters simulation/control time. Closing
            // it ends an unbounded run (there is nothing left to watch)
            // and lets a bounded one finish its stated length headless.
            if (viewer && tick % 8 == 0) {
                if (!viewer->Render(runner.Backend().Model(),
                                    runner.Backend().Data())) {
                    viewer.reset();
                    window_closed = true;
                    if (unbounded)
                        break;
                }
            }
#endif
        }

        // Run provenance and class-A diagnostics. The hold reference each
        // core regulates to is the TCP world pose captured on the first
        // fresh tick (world hold, contract section 7); "est" errors are
        // the core's own estimate against that reference, while the world
        // hold error and Mount-frame excursion are simulator truth (the
        // MuJoCo TCP site, contract section 4) and are therefore
        // independent of the controller's own kinematics.
        std::printf("\nhumansl_sim run summary (evidence class: "
                    "simulation)\n");
        std::printf("  Mount motion: %s\n",
                    MotionKindName(config.mount_motion.kind));
        if (unbounded)
            std::printf("  cycles completed: %ld (unbounded run)\n",
                        completed);
        else
            std::printf("  cycles completed: %ld of %ld\n", completed,
                        cycles);
        std::printf("  ideal sensing: world_fresh on every cycle: %s; max "
                    "overrun_count: %d\n",
                    all_world_fresh ? "yes" : "NO", max_overrun_count);
        std::printf("  Mount excursion from the anchor: %.6f m, %.6f rad "
                    "(sim truth, read back from the plant)\n",
                    max_mount_translation_m, max_mount_rotation_rad);
        std::printf("  right: max world-hold TCP error %.6f m (sim truth), "
                    "Mount-frame excursion %.6f m, max est TCP error "
                    "%.6f m, max est orientation error %.6f rad\n",
                    right_stats.max_world_hold_error_m,
                    right_stats.max_mount_frame_excursion_m,
                    right_stats.max_est_position_error_m,
                    right_stats.max_est_orientation_error_rad);
        std::printf("  left:  max world-hold TCP error %.6f m (sim truth), "
                    "Mount-frame excursion %.6f m, max est TCP error "
                    "%.6f m, max est orientation error %.6f rad\n",
                    left_stats.max_world_hold_error_m,
                    left_stats.max_mount_frame_excursion_m,
                    left_stats.max_est_position_error_m,
                    left_stats.max_est_orientation_error_rad);
        std::printf("  reading those two: under a moving Mount a working "
                    "world hold keeps the world-hold error small while the "
                    "Mount-frame excursion tracks the Mount's own "
                    "excursion; an arm merely carried by the base would "
                    "show the opposite pair.\n");
        std::printf("  max contacts in any cycle (ncon): %d\n", max_ncon);

        if (!stop_cause.empty()) {
            std::printf("  STOPPED early: %s\n", stop_cause.c_str());
            return 1;
        }
        if (max_ncon > 0) {
            // Declared run pass criterion (plan Task 5 Step 5): the
            // ideal-mode hold must finish without contact — a contact
            // would disturb the arms in a way indistinguishable from a
            // Mount-representation error.
            std::printf("  CONTACT observed during the run\n");
            return 1;
        }
        if (interrupted) {
            std::printf("  ended on SIGINT at a tick boundary; no stop and "
                        "no contact\n");
            return 0;
        }
        if (window_closed && unbounded) {
            std::printf("  ended when the viewer window closed; no stop and "
                        "no contact\n");
            return 0;
        }
        std::printf("  completed with no stop and no contact\n");
        return 0;
    } catch (const std::exception& failure) {
        std::fprintf(stderr, "humansl_sim: %s\n", failure.what());
        return 1;
    }
}
