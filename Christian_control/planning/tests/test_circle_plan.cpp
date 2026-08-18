// End-to-end: a circle is planned and the exact final dense GPMP2 artefact
// that feeds world-Cartesian projection is validated against the requested
// geometry.
//
// It also pins the two circle-specific decompositions. A single distance
// hides WHICH way a trace is wrong, and out-of-plane drift has a different
// cause from a wrong radius.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>

#include "CartesianPath.h"
#include "PathFrames.h"
#include "PlanSolver.h"
#include "PlannerConfig.h"
#include "PlannerModel.h"

int main(int argc, char** argv) {
    assert(argc == 4 &&
           "usage: test_circle_plan <dh_flange.yaml> <joint_limits.yaml> <planner.yaml>");
    const Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
    const PlannerModel model =
        LoadPlannerModel(argv[1], /*has_tool=*/false, world_T_mount);
    const PlannerConfig config = LoadPlannerConfig(argv[3]);

    // A circle measured reachable by probe_path_reachability (2026-08-07):
    // 13/13 samples on path at 0.11 mm, from this start configuration.
    CircleSpec spec;
    spec.frame = config::ReferenceFrame::kLeftBase;
    spec.centre_m = Eigen::Vector3d(0.0239, -0.563535, 0.0235506);
    spec.radius_m = 0.03;
    spec.normal = Eigen::Vector3d::UnitZ();
    spec.duration_s = 12.0;
    spec.orientation = OrientationPolicy::kFixed;
    spec.fixed_rpy_rad =
        Eigen::Vector3d(12.3, -63.0, 107.3) * (M_PI / 180.0);
    spec.samples = CircleSamplesForChordError(
        spec.radius_m, config.path_following.max_chord_error_m);
    assert(spec.samples >= 8 && "chord-error sampling must produce a usable count");

    const CartesianPath task_path =
        PathToWorld(GenerateCircle(spec), world_T_mount);

    Eigen::Matrix<double, 7, 1> start;
    start << -26.83, -113.52, 92.11, 109.07, 81.18, -24.70, -138.06;
    start *= M_PI / 180.0;

    ValidationInputs validation;
    validation.circle_applicable = true;
    validation.circle_centre =
        PoseToWorld(Eigen::Isometry3d(Eigen::Translation3d(spec.centre_m)),
                    spec.frame, world_T_mount).translation();
    validation.circle_normal =
        PoseToWorld(Eigen::Isometry3d(Eigen::Quaterniond::Identity()), spec.frame,
                    world_T_mount)
            .linear() * spec.normal.normalized();
    validation.circle_radius_m = spec.radius_m;

    const PathPlanOutcome plan =
        SolveAlongPath(model, task_path, start, std::nullopt, argv[2], config,
                       validation);
    assert(plan.ok && "planning a reachable circle must succeed");
    assert(plan.result.trajectory_pos.size() ==
               plan.result.trajectory_vel.size() &&
           plan.result.trajectory_pos.size() > 1000 &&
           "the outcome must retain every final dense state, not the old "
           "1,000-point wire subset");
    std::printf("%s", plan.report.Summary().c_str());

    // --- the dense result begins at the measured configuration. The
    // world-Cartesian controller applies its own pose-continuity splice.
    const double start_error =
        (plan.result.trajectory_pos.front() - start).cwiseAbs().maxCoeff();
    assert(start_error < 2.0 * M_PI / 180.0 &&
           "the trajectory must start within the 2 deg splice guard");

    // --- rest at both ends. Zero-velocity support states, not duplicated
    //     points, are what make Hermite come to a stop.
    assert(plan.result.trajectory_vel.front().cwiseAbs().maxCoeff() <
           1e-3 && "the trajectory must start from rest");
    assert(plan.result.trajectory_vel.back().cwiseAbs().maxCoeff() <
           0.05 && "the trajectory must end essentially at rest");

    // --- fidelity, measured on the reconstruction and gated on it.
    assert(plan.report.task_fidelity_valid &&
           "a reachable circle must trace within the configured tolerance");
    assert(plan.report.command.max_position_m <=
               config.path_following.maximum_planning_error_m &&
           "e_command is the gate and must satisfy the configured requirement");

    // The circle decomposition must be consistent with the total: neither
    // component can exceed the overall deviation.
    assert(plan.report.command_circle.applicable);
    assert(plan.report.command_circle.max_plane_error_m <=
           plan.report.command.max_position_m + 1e-9);
    assert(plan.report.command_circle.max_radial_error_m <=
           plan.report.command.max_position_m + 1e-9);

    // --- every validity flag, and the gate they compose into.
    assert(plan.report.start_state_valid);
    assert(plan.report.joint_limits_valid);
    assert(plan.report.dynamic_limits_valid);
    assert(plan.report.modelled_collision_valid);
    assert(plan.report.hardware_execution_allowed &&
           "all modelled checks passing must clear the trajectory");

    // The report must SAY what the SDF held: a green collision check that
    // did not disclose an almost-empty world would read as "safe near a
    // person", which it is not.
    assert(plan.report.sdf_contents.find("NOT modelled") != std::string::npos &&
           "the report must disclose what the collision check did not cover");

    // --- continuation must not have flipped IK branches. A large step
    //     between NEIGHBOURING samples means the solver jumped to a
    //     different solution branch despite the seeding, which would
    //     execute as a lurch even though every sample is individually
    //     on-path.
    assert(plan.maximum_joint_step_rad < 30.0 * M_PI / 180.0 &&
           "a large joint step between neighbouring samples means a branch flip");

    // Closure drift is REPORTED, not asserted, and deliberately so.
    //
    // It measures how far the redundant elbow ends from where it started
    // while the tool pose closes exactly. On a 7-DoF arm the null-space
    // motion is unconstrained, so for a single lap the drift is harmless —
    // it only matters if a second lap must repeat the first. Measured
    // 2026-08-07 it varies between 0.04 and 14 deg across runs of the SAME
    // request while the traced circle stays at 2.4 mm every time, so
    // gating on it would fail plans that trace perfectly well.
    //
    // The variance has a cause worth knowing: analytical_ik seeds from
    // std::random_device, so the planner is not reproducible run to run.
    // That is a real limitation for repeatable experiments and is recorded
    // as such rather than papered over here.
    std::printf("closure drift %.2f deg (reported, not gated — see the test's "
                "comment on why)\n", plan.closure_drift_rad * 180.0 / M_PI);

    std::printf("\nall circle-plan integration tests passed\n");
    return 0;
}
