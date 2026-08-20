#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "PathIk.h"
#include "PathAssembly.h"
#include "PathFrames.h"
#include "PlannerConfig.h"
#include "PlannerModel.h"
#include "MountSdf.h"
#include "TrajectoryOptimization.h"
#include "ValidatePath.h"

#include <gtsam/inference/Symbol.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

PathSample SampleFor(const Eigen::Matrix<double, 7, 1>& q,
                     const PathIkArm& arm, double t_s) {
    PathSample sample;
    sample.t_s = t_s;
    sample.pose = Eigen::Isometry3d(
        analytical_ik::AnalyticalIKSolver::computeForwardKinematics(
            q, arm.base_transform, arm.end_effector_frame, arm.left_arm));
    return sample;
}

PathSample UnreachableSample(double t_s) {
    PathSample sample;
    sample.t_s = t_s;
    sample.pose.translation() = Eigen::Vector3d(10.0, 10.0, 10.0);
    return sample;
}

double AngleDifference(double a, double b) {
    return std::remainder(a - b, 2.0 * kPi);
}

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

PathIkJointLimits PathLimits(const JointLimits& limits) {
    PathIkJointLimits result;
    for (int joint = 0; joint < 7; ++joint) {
        result.lower_rad(joint) = limits.lower(joint);
        result.upper_rad(joint) = limits.upper(joint);
    }
    return result;
}

void TestGpmp2RecoversOneLocalIkGap(const char* dh_yaml,
                                    const char* joint_limits_yaml,
                                    const char* planner_yaml) {
    const PlannerModel model = LoadPlannerModel(dh_yaml, /*has_tool=*/false);
    const PlannerConfig config = LoadPlannerConfig(planner_yaml);
    const auto [position_limits, velocity_limits] =
        createJointLimits(joint_limits_yaml);
    const PathIkJointLimits path_limits = PathLimits(position_limits);

    // Proven reachable fixture from the original circle-plan integration
    // test: 13/13 path samples solved and the optimized trace stayed inside
    // the physical acceptance threshold.
    CircleSpec spec;
    spec.frame = config::ReferenceFrame::kLeftBase;
    spec.centre_m = Eigen::Vector3d(0.0239, -0.563535, 0.0235506);
    spec.radius_m = 0.03;
    spec.normal = Eigen::Vector3d::UnitZ();
    spec.duration_s = 12.0;
    spec.orientation = OrientationPolicy::kFixed;
    spec.fixed_rpy_rad =
        Eigen::Vector3d(12.3, -63.0, 107.3) * (kPi / 180.0);
    spec.samples = CircleSamplesForChordError(
        spec.radius_m, config.path_following.max_chord_error_m);
    const CartesianPath requested_path =
        PathToMount(GenerateCircle(spec), std::nullopt);

    Eigen::Matrix<double, 7, 1> start;
    start << -26.83, -113.52, 92.11, 109.07, 81.18, -24.70, -138.06;
    start *= kPi / 180.0;

    PathIkArm arm;
    arm.base_transform = model.base_pose.matrix();
    arm.end_effector_frame = model.end_effector_frame;
    arm.left_arm = model.left_arm;
    analytical_ik::IKTolerance tolerance;
    tolerance.converge_position_m =
        config.path_following.maximum_planning_error_m * 0.1;
    tolerance.converge_orientation_rad =
        config.path_following.maximum_orientation_error_rad * 0.1;
    tolerance.accept_position_m =
        config.path_following.maximum_planning_error_m;
    tolerance.accept_orientation_rad =
        config.path_following.maximum_orientation_error_rad;

    // Withhold one otherwise reachable local IK result. PathIk must create
    // the interpolated initializer, while GPMP2 still receives the ORIGINAL
    // reachable Cartesian pose prior at every sample.
    CartesianPath ik_path = requested_path;
    const std::size_t gap = ik_path.samples.size() / 2;
    ik_path.samples[gap].pose.translation() += Eigen::Vector3d(10.0, 10.0, 10.0);
    const PathIkResult walk =
        SolvePathIk(ik_path, arm, start, path_limits, tolerance,
                    /*closed=*/true);
    Require(walk.success && walk.unresolved_samples == 1 &&
                walk.interpolated_samples == 1,
            "one local IK failure did not produce one interpolated seed");

    std::vector<Eigen::Matrix<double, 7, 1>> task_configurations;
    task_configurations.reserve(walk.samples.size());
    for (const PathIkSample& sample : walk.samples)
        task_configurations.push_back(sample.configuration);

    ApproachPacing pacing;
    pacing.velocity_fraction = config.path_following.approach_velocity_fraction;
    pacing.minimum_duration_s = config.path_following.approach_min_duration_s;
    pacing.waypoints = config.path_following.approach_waypoints;
    JointVelocityLimits joint_velocity_limits;
    for (int joint = 0; joint < 7; ++joint)
        joint_velocity_limits(joint) = velocity_limits.upper(joint);
    const AssembledPath assembled = AssembleCirclePlan(
        requested_path, task_configurations, start, joint_velocity_limits,
        pacing,
        Eigen::Vector3d::Constant(
            config.path_following.rotation_prior_sigma_rad),
        Eigen::Vector3d::Constant(
            config.path_following.position_prior_sigma_m));

    gtsam::Values initial;
    const gtsam::Vector zero_velocity = gtsam::Vector::Zero(7);
    for (std::size_t index = 0; index < assembled.waypoints.size(); ++index) {
        initial.insert(gtsam::Symbol('x', index),
                       gtsam::Vector(assembled.initial_configurations[index]));
        initial.insert(gtsam::Symbol('v', index), zero_velocity);
    }
    const auto sdf = MakeMountSdf(MountGridGeometry(), std::nullopt);
    OptimizeTrajectory optimizer;
    const TrajectoryResult solved = optimizer.optimizeTaskTrajectory(
        *model.arm_model, sdf, initial, assembled.waypoints,
        gtsam::Vector(start), assembled.zero_velocity_indices,
        position_limits, velocity_limits, assembled.total_duration_s,
        config.optimizer);
    Require(!solved.trajectory_pos.empty(),
            "GPMP2 returned no trajectory for the recovered circle seed");

    TimedJointTrajectory dense;
    dense.valid = true;
    dense.duration_s = assembled.total_duration_s;
    const double dense_dt = dense.duration_s /
                            static_cast<double>(solved.trajectory_pos.size() - 1);
    for (std::size_t index = 0; index < solved.trajectory_pos.size(); ++index) {
        TimedJointSample sample;
        sample.t_s = static_cast<double>(index) * dense_dt;
        sample.q_rad = solved.trajectory_pos[index];
        sample.qdot_rad_s = solved.trajectory_vel[index];
        dense.samples.push_back(sample);
    }
    const TimedJointSampler sample_at = [&dense, dense_dt](double t_s) {
        const std::size_t index = std::min(
            dense.samples.size() - 1,
            static_cast<std::size_t>(std::llround(t_s / dense_dt)));
        return dense.samples[index];
    };
    ValidationInputs validation;
    validation.desired_task_path = &requested_path;
    validation.task_start_time_s =
        assembled.waypoints[assembled.task_start_index].time_s;
    validation.validation_dt_s = dense_dt;
    validation.maximum_planning_error_m =
        config.path_following.maximum_planning_error_m;
    validation.maximum_orientation_error_rad =
        config.path_following.maximum_orientation_error_rad;
    validation.measured_start = start;
    for (int joint = 0; joint < 7; ++joint) {
        validation.joint_velocity_limits_rad_s(joint) =
            velocity_limits.upper(joint);
        validation.joint_acceleration_limits_rad_s2(joint) =
            velocity_limits.upper(joint) * 2.0;
    }
    const PathValidationReport report = ValidatePlannedPath(
        model, dense, solved.trajectory_pos, dense.duration_s, sample_at, sdf,
        "test empty mount-frame SDF", validation,
        /*optimiser_converged=*/true);
    Require(report.command.max_position_m <=
                config.path_following.maximum_planning_error_m &&
                report.command.max_orientation_rad <=
                    config.path_following.maximum_orientation_error_rad,
            "GPMP2 did not recover the full dense circle inside the threshold");
}

}  // namespace

int main(int argc, char** argv) {
    Require(argc == 4,
            "usage: test_path_ik dh_flange.yaml joint_limits.yaml planner.yaml");
    PathIkArm arm;
    arm.end_effector_frame = config::kRightEndEffectorFrame;
    const auto [position_limits, ignored_velocity_limits] =
        createJointLimits(argv[2]);
    (void)ignored_velocity_limits;
    const PathIkJointLimits path_limits = PathLimits(position_limits);

    // Joint 1 is continuous. The valid neighbours intentionally straddle
    // its -pi/+pi seam, where arithmetic averaging would choose the long
    // route through zero.
    Eigen::Matrix<double, 7, 1> before;
    before << 3.10, 0.20, -0.20, 0.30, 0.10, -0.30, 0.20;
    Eigen::Matrix<double, 7, 1> after = before;
    after(0) = -3.10;

    CartesianPath isolated_gap;
    isolated_gap.samples = {
        SampleFor(before, arm, 0.0),
        UnreachableSample(1.0),
        SampleFor(after, arm, 2.0),
    };

    analytical_ik::IKTolerance tolerance;
    tolerance.converge_position_m = 1e-5;
    tolerance.converge_orientation_rad = 1e-5;
    tolerance.accept_position_m = 1e-3;
    tolerance.accept_orientation_rad = 1e-3;

    const PathIkResult recovered =
        SolvePathIk(isolated_gap, arm, before, path_limits, tolerance, true);
    Require(recovered.samples.size() == 3, "unexpected sample count");
    Require(recovered.success, "isolated unresolved sample was not seedable");
    Require(recovered.samples[0].solved, "first valid neighbour did not solve");
    Require(!recovered.samples[1].solved, "unresolved sample was marked solved");
    Require(recovered.samples[2].solved, "second valid neighbour did not solve");
    Require(recovered.unresolved_samples == 1 &&
                recovered.interpolated_samples == 1,
            "isolated unresolved sample did not receive one interpolated seed");
    Require(recovered.maximum_joint_step_rad < 0.2,
            "continuous-joint step diagnostic used raw subtraction");
    Require(recovered.closure_drift_rad < 0.2,
            "continuous-joint closure diagnostic used raw subtraction");

    // The unresolved sample is a seed interpolation, not the failed IK
    // near-miss. Bounded joints interpolate linearly; continuous joint 1
    // takes the shortest wrapped arc.
    const Eigen::Matrix<double, 7, 1>& q_before =
        recovered.samples[0].configuration;
    const Eigen::Matrix<double, 7, 1>& q_after =
        recovered.samples[2].configuration;
    const Eigen::Matrix<double, 7, 1>& q_gap =
        recovered.samples[1].configuration;
    const double expected_continuous =
        q_before(0) + 0.5 * std::remainder(q_after(0) - q_before(0), 2.0 * kPi);
    Require(std::abs(AngleDifference(q_gap(0), expected_continuous)) < 1e-6,
            "continuous joint used long-way interpolation");
    for (int joint = 1; joint < 7; ++joint)
        Require(std::abs(q_gap(joint) -
                         0.5 * (q_before(joint) + q_after(joint))) < 1e-6,
                "bounded joint interpolation was not linear");

    // A leading unresolved region has no two-sided continuation seed and is
    // rejected as an initialization failure rather than filled blindly.
    CartesianPath leading_gap;
    leading_gap.samples = {
        UnreachableSample(0.0),
        UnreachableSample(1.0),
        SampleFor(after, arm, 2.0),
    };
    const PathIkResult leading_rejected =
        SolvePathIk(leading_gap, arm, before, path_limits, tolerance, false);
    Require(!leading_rejected.success,
            "leading unresolved region was incorrectly accepted");

    // Several consecutive failures are not a trustworthy initializer even
    // when the path is closed and valid neighbours exist on both sides.
    CartesianPath long_gap;
    long_gap.samples = {
        SampleFor(before, arm, 0.0), UnreachableSample(1.0),
        UnreachableSample(2.0),      UnreachableSample(3.0),
        UnreachableSample(4.0),      SampleFor(after, arm, 5.0),
    };
    const PathIkResult long_rejected =
        SolvePathIk(long_gap, arm, before, path_limits, tolerance, true);
    Require(!long_rejected.success &&
                long_rejected.maximum_unresolved_run == 4,
            "a long unresolved run was incorrectly accepted");

    // The only unseedable path: nothing solved anywhere, so no anchor for a
    // finite continuous seed exists.
    CartesianPath hopeless;
    hopeless.samples = {UnreachableSample(0.0), UnreachableSample(1.0)};
    const PathIkResult none =
        SolvePathIk(hopeless, arm, before, path_limits, tolerance, false);
    Require(!none.success && none.unresolved_samples == 2,
            "a path with no solved sample was not reported as unseedable");

    Eigen::Matrix<double, 7, 1> outside_limits = before;
    outside_limits(1) = path_limits.upper_rad(1) + 0.01;
    bool rejected_seed = false;
    try {
        (void)SolvePathIk(isolated_gap, arm, outside_limits, path_limits,
                          tolerance, false);
    } catch (const std::invalid_argument&) {
        rejected_seed = true;
    }
    Require(rejected_seed,
            "bounded start seed outside planner limits was silently changed");

    // A closed path has no privileged beginning. One unresolved sample at
    // each end is one two-sample gap across the circle seam, bounded by the
    // valid samples on either side.
    CartesianPath closed_seam_gap;
    closed_seam_gap.samples = {
        UnreachableSample(0.0),
        SampleFor(after, arm, 1.0),
        SampleFor(before, arm, 2.0),
        UnreachableSample(3.0),
    };
    const PathIkResult closed_recovered =
        SolvePathIk(closed_seam_gap, arm, before, path_limits, tolerance, true);
    Require(closed_recovered.success,
            "closed-circle seam gap was treated as two open-path gaps");
    Require(closed_recovered.interpolated_samples == 2,
            "closed-circle seam gap did not interpolate both endpoints");

    TestGpmp2RecoversOneLocalIkGap(argv[1], argv[2], argv[3]);

    std::puts("test_path_ik: all assertions passed");
    return 0;
}
