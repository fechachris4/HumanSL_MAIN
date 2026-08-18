//
// probe_path_reachability — OFFLINE: can the arm actually trace this path?
//
// NEVER CONNECTS TO A ROBOT. It loads the URDF-derived model and solves
// inverse kinematics at each sample, nothing more. Safe to run with both
// arms powered off.
//
// Why this exists. A single goal that fails is obvious — the planner says
// so. A PATH that fails is not: it can be reachable for three quarters of
// its length and impossible for the rest, and the planner will happily
// optimise a trajectory that quietly cuts the corner. On 2026-08-06 a
// left-arm goal whose POSITION was reachable turned out to be impossible at
// the requested ORIENTATION, because it needed joint 6 about 38 deg past
// its stop. A circle sweeps far more workspace than a point, so that
// failure mode is the rule rather than the exception. Establish
// reachability before spending a solve on it.
//
// Continuation seeding: each sample is solved from the PREVIOUS sample's
// solution rather than from scratch. A redundant arm has a continuous
// family of solutions for most poses, and independent solves are free to
// jump between branches — the elbow flips, and the joint trajectory
// contains a discontinuity the Cartesian path never asked for. Seeding from
// the neighbour keeps the walk on one branch. Where a sample fails, the
// last successful solution stays the seed, so one bad patch does not
// scatter everything after it.
//
//   ./probe_path_reachability --arm left --dh <yaml> \
//        --centre X Y Z --radius R --normal NX NY NZ \
//        [--frame mount|right_base|left_base] [--samples N]
//        [--orientation fixed|radial] [--rpy R P Y]
//        [--start-deg J1..J7]
//

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "CartesianPath.h"
#include "Config.h"
#include "PathFrames.h"
#include "PathIk.h"
#include "PlannerModel.h"
#include "analytical_ik.h"

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

[[noreturn]] void Usage(const std::string& error) {
    if (!error.empty()) std::fprintf(stderr, "error: %s\n\n", error.c_str());
    std::fprintf(stderr,
        "usage: probe_path_reachability --arm right|left --dh <params.yaml>\n"
        "         --centre X Y Z --radius R [--normal NX NY NZ]\n"
        "         [--frame mount|right_base|left_base] [--samples N]\n"
        "         [--orientation fixed|radial] [--rpy ROLL PITCH YAW (deg)]\n"
        "         [--start-deg J1..J7]\n"
        "         [--tolerance-mm T] [--tolerance-deg T]\n"
        "\n"
        "Solves IK at every sample of the circle and reports which are\n"
        "reachable. Never connects to a robot.\n"
        "\n"
        "--tolerance-mm/--tolerance-deg (default 2 mm / 2 deg) are how close\n"
        "the tool must actually get for a sample to count as ON the path.\n"
        "They are NOT the solver's own acceptance test, which passes at\n"
        "40 mm / 8.6 deg because it was tuned to SEED a point-to-point\n"
        "optimiser that then corrects the endpoint. Nothing corrects the\n"
        "middle of a path, so certifying a path against that threshold would\n"
        "call a 38 mm-wide smear a circle.\n");
    std::exit(2);
}

double ParseDouble(const std::string& text) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value))
        Usage("not a finite number: '" + text + "'");
    return value;
}

config::ReferenceFrame FrameFromName(const std::string& name) {
    for (int i = 0; i < 4; ++i)
        if (name == config::kReferenceFrameNames[i])
            return static_cast<config::ReferenceFrame>(i);
    Usage("unknown frame '" + name + "'");
}

}  // namespace

int main(int argc, char** argv) {
    std::string arm, dh_path;
    CircleSpec spec;
    spec.samples = 36;
    spec.duration_s = 1.0;  // unused by the probe; any positive value is fine
    spec.normal = Eigen::Vector3d::UnitZ();
    bool have_centre = false, have_radius = false;
    Eigen::Matrix<double, 7, 1> seed = Eigen::Matrix<double, 7, 1>::Zero();
    double tolerance_mm = 2.0, tolerance_deg = 2.0;

    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        const auto next = [&]() -> std::string {
            if (++i >= argc) Usage("missing value after " + flag);
            return argv[i];
        };
        if (flag == "--arm") arm = next();
        else if (flag == "--dh") dh_path = next();
        else if (flag == "--frame") spec.frame = FrameFromName(next());
        else if (flag == "--radius") { spec.radius_m = ParseDouble(next()); have_radius = true; }
        else if (flag == "--samples") spec.samples = static_cast<int>(ParseDouble(next()));
        else if (flag == "--centre" || flag == "--center") {
            for (int a = 0; a < 3; ++a) spec.centre_m(a) = ParseDouble(next());
            have_centre = true;
        } else if (flag == "--normal") {
            for (int a = 0; a < 3; ++a) spec.normal(a) = ParseDouble(next());
        } else if (flag == "--rpy") {
            for (int a = 0; a < 3; ++a) spec.fixed_rpy_rad(a) = ParseDouble(next()) / kRadToDeg;
        } else if (flag == "--orientation") {
            const std::string value = next();
            if (value == "fixed") spec.orientation = OrientationPolicy::kFixed;
            else if (value == "radial") spec.orientation = OrientationPolicy::kRadialInward;
            else Usage("--orientation must be 'fixed' or 'radial'");
        } else if (flag == "--tolerance-mm") {
            tolerance_mm = ParseDouble(next());
        } else if (flag == "--tolerance-deg") {
            tolerance_deg = ParseDouble(next());
        } else if (flag == "--start-deg") {
            for (int j = 0; j < 7; ++j) seed(j) = ParseDouble(next()) / kRadToDeg;
        } else Usage("unrecognized flag '" + flag + "'");
    }
    if (arm != "right" && arm != "left") Usage("--arm must be 'right' or 'left'");
    if (dh_path.empty()) Usage("--dh is required");
    if (!have_centre || !have_radius) Usage("--centre and --radius are required");

    const bool left_arm = arm == "left";
    // has_tool mirrors LoadPlannerModel's contract: tool <-> right chain,
    // bare flange <-> left chain.
    // The standalone probe has no Vicon input; identity makes its WORLD
    // coincide with Mount while exercising the same world-aware model path.
    const Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
    const PlannerModel model =
        LoadPlannerModel(dh_path, /*has_tool=*/!left_arm, world_T_mount);
    const Eigen::Matrix4d base_transform = model.base_pose.matrix();

    CartesianPath path;
    try {
        path = PathToWorld(GenerateCircle(spec), world_T_mount);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }

    std::printf("arm %s, frame %s -> world, radius %.4f m, %d samples\n",
                arm.c_str(), config::kReferenceFrameNames[static_cast<int>(spec.frame)],
                spec.radius_m, spec.samples);
    std::printf("on-path tolerance: %.2f mm / %.2f deg\n\n", tolerance_mm, tolerance_deg);
    std::printf("%-5s %-28s %-9s %-11s %-11s %s\n",
                "i", "position in world (m)", "verdict", "pos err(mm)", "rot err(deg)",
                "joints outside limits");

    int solved_count = 0, on_path_count = 0;
    double worst_pos_mm = 0.0, worst_rot_deg = 0.0;
    std::vector<bool> reachable;
    reachable.reserve(path.samples.size());

    // Ask the solver to converge to the accuracy being certified, rather
    // than to its default 20 mm early exit — otherwise the residuals report
    // where the solver gave up, not where the arm can reach, and every
    // sample of a reachable circle looks 8-19 mm off it. A tenth of the
    // tolerance leaves headroom so the verdict is about the arm.
    analytical_ik::IKTolerance tolerance;
    tolerance.converge_position_m = tolerance_mm / 1000.0 * 0.1;
    tolerance.converge_orientation_rad = tolerance_deg / kRadToDeg * 0.1;
    tolerance.accept_position_m = tolerance_mm / 1000.0;
    tolerance.accept_orientation_rad = tolerance_deg / kRadToDeg;

    // ONE implementation of the continuation walk, shared with the planner
    // (PathIk.h). The probe used to carry its own copy; two copies of "how
    // the arm walks a path" would eventually disagree, and the probe would
    // then certify something the planner does not do.
    PathIkArm probe_arm;
    probe_arm.base_transform = base_transform;
    probe_arm.end_effector_frame = model.end_effector_frame;
    probe_arm.left_arm = model.left_arm;

    const PathIkResult walk =
        SolvePathIk(path, probe_arm, seed, tolerance, /*closed=*/true);

    for (std::size_t i = 0; i < path.samples.size(); ++i) {
        const Eigen::Isometry3d& pose = path.samples[i].pose;
        const PathIkSample& sample = walk.samples[i];

        std::string limit_note;
        for (int j = 0; j < 7; ++j) {
            const double lo = analytical_ik::KinovaGen3Params::JOINT_LIMITS_LOWER[j];
            const double hi = analytical_ik::KinovaGen3Params::JOINT_LIMITS_UPPER[j];
            if (lo < -1e10 || hi > 1e10) continue;  // continuous joint
            const double q = std::remainder(sample.configuration(j), 2.0 * M_PI);
            if (q < lo || q > hi) {
                char buffer[64];
                std::snprintf(buffer, sizeof buffer, "j%d by %.1f deg ", j + 1,
                              (q < lo ? lo - q : q - hi) * kRadToDeg);
                limit_note += buffer;
            }
        }

        const double pos_mm = sample.position_residual_m * 1000.0;
        const double rot_deg = sample.orientation_residual_rad * kRadToDeg;
        // Three verdicts, not two. "solved" is the solver's own loose test;
        // "on path" is whether the tool would actually be where the path
        // asked. A sample can be the first without being the second, and
        // for a path only the second one means anything.
        const bool on_path =
            sample.solved && pos_mm <= tolerance_mm && rot_deg <= tolerance_deg;
        const char* verdict = !sample.solved ? "NO"
                              : on_path      ? "on-path"
                                             : "loose";

        const Eigen::Vector3d p = pose.translation();
        std::printf("%-5zu %8.4f %8.4f %8.4f      %-9s %-11.2f %-11.3f %s\n",
                    i, p.x(), p.y(), p.z(), verdict, pos_mm, rot_deg,
                    limit_note.empty() ? "-" : limit_note.c_str());

        reachable.push_back(on_path);
        if (on_path) ++on_path_count;
        if (sample.solved) ++solved_count;
        worst_pos_mm = std::max(worst_pos_mm, pos_mm);
        worst_rot_deg = std::max(worst_rot_deg, rot_deg);
    }

    const std::size_t total = path.samples.size();
    std::printf("\n%d/%zu samples ON PATH (within %.2f mm / %.2f deg)\n",
                on_path_count, total, tolerance_mm, tolerance_deg);
    std::printf("%d/%zu samples solved at all (the solver's own loose test)\n",
                solved_count, total);
    std::printf("worst residual over the path: %.2f mm / %.3f deg\n",
                worst_pos_mm, worst_rot_deg);

    // Continuation diagnostics. A large joint step between neighbouring
    // samples means the solver flipped to a different IK branch despite the
    // seeding — the tool moved a few millimetres while the arm swung. That
    // would execute as a lurch, so it matters even when every sample is
    // individually on-path.
    std::printf("largest joint step between neighbouring samples: %.2f deg%s\n",
                walk.maximum_joint_step_rad * kRadToDeg,
                walk.maximum_joint_step_rad * kRadToDeg > 30.0
                    ? "  <-- LARGE: likely an IK branch flip, expect a lurch"
                    : "");
    // The tool pose closes exactly (the generator repeats the first point),
    // but the redundant elbow need not return with it.
    std::printf("closure drift (first vs last configuration): %.2f deg%s\n",
                walk.closure_drift_rad * kRadToDeg,
                walk.closure_drift_rad * kRadToDeg > 5.0
                    ? "  <-- a second lap would not repeat the first"
                    : "");

    // Worst-case posture quality along the path: how close the arm comes to
    // a joint stop, and to a singularity. Both are warnings rather than
    // failures — a path can be traced from a poor posture, it is just
    // fragile there.
    double worst_margin_deg = 1e9, worst_sigma = 1e9;
    for (const PathIkSample& s : walk.samples) {
        if (!s.solved) continue;
        worst_margin_deg =
            std::min(worst_margin_deg, s.minimum_joint_limit_margin_rad * kRadToDeg);
        worst_sigma = std::min(worst_sigma, s.minimum_manipulability);
    }
    if (worst_margin_deg < 1e8)
        std::printf("tightest joint-limit margin: %.1f deg; smallest sigma_min: %.4f\n",
                    worst_margin_deg, worst_sigma);

    // Contiguous off-path runs matter more than the raw count: scattered
    // failures usually mean the solver, one long gap means the geometry.
    for (std::size_t i = 0; i < reachable.size();) {
        if (reachable[i]) { ++i; continue; }
        std::size_t end = i;
        while (end < reachable.size() && !reachable[end]) ++end;
        std::printf("  OFF-PATH arc: samples %zu..%zu (%zu of %zu, %.0f%% of the path)\n",
                    i, end - 1, end - i, total,
                    100.0 * static_cast<double>(end - i) / static_cast<double>(total));
        i = end;
    }

    if (on_path_count == static_cast<int>(total)) {
        std::printf("  VERDICT: the whole path is traceable within tolerance\n");
        return 0;
    }
    if (solved_count == static_cast<int>(total)) {
        // The distinction that matters most, and the one a yes/no report
        // would have hidden: the arm can get near every sample but not
        // accurately. Usually the requested orientation is only achievable
        // at part of the path, or the path is at the edge of the workspace.
        std::printf("  VERDICT: every sample is roughly reachable, but %d are off the\n"
                    "  requested path by more than the tolerance. The arm would trace\n"
                    "  a distorted shape, not this one. Try a smaller radius, a\n"
                    "  different centre, or --orientation radial.\n",
                    static_cast<int>(total) - on_path_count);
        return 0;
    }
    std::printf("  VERDICT: part of this path is not reachable at all.\n");
    return 0;
}
