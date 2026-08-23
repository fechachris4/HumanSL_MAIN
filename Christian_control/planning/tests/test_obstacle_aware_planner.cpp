#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "CartesianPath.h"
#include "MountSdf.h"
#include "PathAssembly.h"
#include "PinocchioKinematicsAdapter.h"
#include "PlanDebugDump.h"
#include "PlanSolver.h"
#include "PlannerConfig.h"
#include "PlannerModel.h"
#include "WorldCartesianTrajectoryWire.h"
#include "WorldTrajectoryProjection.h"

namespace {

int failures = 0;
std::optional<std::filesystem::path> artifact_root;

void Check(bool condition, const std::string& message)
{
    if (!condition) {
        std::printf("FAIL: %s\n", message.c_str());
        ++failures;
    }
}

void CheckArtifactWrite(const std::optional<std::string>& error,
                        const std::string& request_id)
{
    Check(!error, request_id + ": " + error.value_or(""));
}

const char* SphereGroupName(CollisionSphereGroup group)
{
    switch (group) {
    case CollisionSphereGroup::kMountInterface: return "mount_interface";
    case CollisionSphereGroup::kProximalArm: return "proximal_arm";
    case CollisionSphereGroup::kUpperArm: return "upper_arm";
    case CollisionSphereGroup::kForearm: return "forearm";
    case CollisionSphereGroup::kTool: return "tool";
    }
    return "unknown";
}

void WriteFixturePlannerYaml(const std::filesystem::path& path,
                             const PlannerConfig& config,
                             const std::string& request_id)
{
    std::ofstream file(path);
    file << std::setprecision(17)
         << "# Display-only snapshot of the exact in-process fixture scene.\n"
         << "# It is not a complete planner configuration.\n"
         << "path_following:\n"
         << "  maximum_planning_error_m: "
         << config.path_following.maximum_planning_error_m << "\n"
         << "  maximum_orientation_error_rad: "
         << config.path_following.maximum_orientation_error_rad << "\n"
         << "obstacles:\n"
         << "  minimum_clearance_m: " << config.minimum_clearance_m << "\n"
         << "  preferred_clearance_m: "
         << config.optimizer.preferred_clearance_m << "\n"
         << "  scene:\n";
    for (const NamedStaticObstacle& obstacle : config.scene) {
        file << "    " << obstacle.id << ":\n"
             << "      enabled: " << (obstacle.enabled ? "true" : "false")
             << "\n"
             << "      shape: " << StaticObstacleShapeName(obstacle.geometry)
             << "\n";
        std::visit(
            [&](const auto& primitive) {
                using Primitive = std::decay_t<decltype(primitive)>;
                const Eigen::Vector3d center = [&]() {
                    if constexpr (std::is_same_v<Primitive, AxisAlignedBox>)
                        return primitive.center;
                    else
                        return primitive.center_mount_m;
                }();
                file << "      center_mount_m: [" << center.x() << ", "
                     << center.y() << ", " << center.z() << "]\n";
                if constexpr (std::is_same_v<Primitive, AxisAlignedBox>) {
                    file << "      half_extent_m: ["
                         << primitive.half_extent.x() << ", "
                         << primitive.half_extent.y() << ", "
                         << primitive.half_extent.z() << "]\n";
                } else {
                    file << "      radius_m: " << primitive.radius_m << "\n"
                         << "      height_m: " << primitive.height_m << "\n";
                }
            },
            obstacle.geometry);
        file << "      permitted_sphere_groups: [";
        for (std::size_t index = 0;
             index < obstacle.permitted_sphere_groups.size(); ++index) {
            if (index > 0)
                file << ", ";
            file << SphereGroupName(obstacle.permitted_sphere_groups[index]);
        }
        file << "]\n";
    }
    Check(static_cast<bool>(file), request_id + ": failed writing planner.yaml");
}

void WriteCommonArtifacts(const std::string& request_id,
                          const std::string& plan_kind,
                          PlanStatus expected_status,
                          bool motion_capable,
                          const PlannerConfig& config,
                          const PlannerModel& model,
                          PlanStatus status,
                          const std::string& failure_reason,
                          const std::optional<TrajectoryResult>& trajectory,
                          const PlanValidationReport& validation,
                          double final_goal_error_m,
                          double total_time_s,
                          const PlanJointLimits& limits,
                          const std::vector<CandidateEvidence>& attempts,
                          const std::optional<std::size_t>& selected_attempt,
                          const CartesianPath* path,
                          const PathIkResult* walk)
{
    if (!artifact_root)
        return;

    const std::filesystem::path directory = *artifact_root / request_id;
    std::error_code directory_error;
    std::filesystem::create_directories(directory, directory_error);
    Check(!directory_error,
          request_id + ": cannot create artifact directory: " +
              directory_error.message());
    if (directory_error)
        return;

    PlanDebugMeta meta;
    meta.arm = "right";
    meta.plan_kind = plan_kind;
    meta.status = status;
    meta.failure_reason = failure_reason;
    meta.final_goal_error_m = final_goal_error_m;
    meta.total_time_s = total_time_s;
    meta.extra = {
        {"request_id", request_id},
        {"evidence_level", "unit test"},
        {"scenario_class", motion_capable ? "motion_capable" :
                                             "deliberate_negative"},
        {"expected_status", PlanStatusName(expected_status)},
        {"minimum_clearance_m", std::to_string(config.minimum_clearance_m)},
        {"preferred_clearance_m",
         std::to_string(config.optimizer.preferred_clearance_m)},
        {"world_T_mount", "identity"},
    };
    if (status != PlanStatus::kFailed) {
        meta.extra.emplace_back(
            "requested_terminal_position_error_m",
            std::to_string(validation.requested_terminal_position_error_m));
        meta.extra.emplace_back(
            "requested_terminal_orientation_error_rad",
            std::to_string(validation.requested_terminal_orientation_error_rad));
    }

    const std::string output = directory.string();
    WriteFixturePlannerYaml(directory / "planner.yaml", config, request_id);
    CheckArtifactWrite(WritePlanMetaCsv(output, meta), request_id);
    CheckArtifactWrite(WriteJointLimitsCsv(output, limits), request_id);
    CheckArtifactWrite(
        WriteCandidateAttemptsCsv(output, attempts, selected_attempt), request_id);
    if (trajectory && !trajectory->trajectory_pos.empty()) {
        CheckArtifactWrite(WriteJointTrajectoryCsv(output, *trajectory), request_id);
        try {
            const WorldCartesianTrajectory projected = ProjectWorldTrajectory(
                model, Eigen::Isometry3d::Identity(),
                trajectory->trajectory_pos, trajectory->trajectory_vel,
                total_time_s, 1, 1);
            std::ofstream cart(directory / "cart_traj.txt");
            cart << FormatWorldCartesianTrajectoryBlock(projected);
            Check(static_cast<bool>(cart),
                  request_id + ": failed writing cart_traj.txt");
        } catch (const std::exception& error) {
            Check(false, request_id + ": Cartesian projection failed: " +
                             error.what());
        }
    }
    if (path != nullptr && walk != nullptr)
        CheckArtifactWrite(WritePathIkCsv(output, *path, *walk, limits),
                           request_id);
}

void WritePointArtifacts(const std::string& request_id,
                         PlanStatus expected_status,
                         bool motion_capable,
                         const PlannerConfig& config,
                         const PlannerModel& model,
                         const PlanOutcome& result)
{
    WriteCommonArtifacts(
        request_id, "point", expected_status, motion_capable, config, model,
        result.status, result.failure_reason, result.trajectory, result.validation,
        result.final_goal_error_m, result.total_time_sec, result.joint_limits,
        result.candidate_attempts, result.selected_candidate_attempt, nullptr,
        nullptr);
}

void WritePathArtifacts(const std::string& request_id,
                        PlanStatus expected_status,
                        const PlannerConfig& config,
                        const PlannerModel& model,
                        const CartesianPath& path,
                        const PathPlanOutcome& result)
{
    WriteCommonArtifacts(
        request_id, "path", expected_status, true, config, model, result.status,
        result.failure_reason, result.trajectory, result.validation,
        result.validation.requested_terminal_position_error_m,
        result.total_time_sec, result.joint_limits, result.candidate_attempts,
        result.selected_candidate_attempt, &path, &result.ik_walk);
}

Eigen::Matrix<double, 7, 1> StartQ()
{
    Eigen::Matrix<double, 7, 1> q;
    q << 1.655554, 1.798854, -0.406276, 0.129150,
         -0.162317, -0.099385, -3.057126;
    return q;
}

Eigen::Isometry3d IndependentToolPoseInMount(
    const PlannerModel& model, const Eigen::Matrix<double, 7, 1>& q)
{
    const auto base_pose =
        pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
            q, model.end_effector_frame, model.left_arm);
    Eigen::Isometry3d base_T_tool = Eigen::Isometry3d::Identity();
    base_T_tool.linear() = base_pose.rotation;
    base_T_tool.translation() = base_pose.position;
    return pinocchio_kinematics_adapter::MountFromBase(model.left_arm) *
           base_T_tool;
}

double IndependentBoxClearance(const AxisAlignedBox& box,
                               const Eigen::Vector3d& centre,
                               double sphere_radius_m)
{
    const Eigen::Vector3d d =
        (centre - box.center).cwiseAbs() - box.half_extent;
    return d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0) -
           sphere_radius_m;
}

double IndependentCylinderClearance(const MountCylinder& cylinder,
                                    const Eigen::Vector3d& centre,
                                    double sphere_radius_m)
{
    const Eigen::Vector3d local = centre - cylinder.center_mount_m;
    const Eigen::Vector2d q(local.head<2>().norm() - cylinder.radius_m,
                            std::abs(local.z()) - 0.5 * cylinder.height_m);
    return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0) -
           sphere_radius_m;
}

double IndependentMinimumClearance(const PlannerModel& model,
                                   const std::vector<NamedStaticObstacle>& scene,
                                   const Eigen::Matrix<double, 7, 1>& q)
{
    const auto centres = model.arm_model->sphereCentersMat(gtsam::Vector(q));
    double clearance = std::numeric_limits<double>::infinity();
    for (const NamedStaticObstacle& obstacle : scene) {
        if (!obstacle.enabled)
            continue;
        for (std::size_t sphere = 0; sphere < model.authored_spheres.size(); ++sphere) {
            if (std::find(obstacle.permitted_sphere_groups.begin(),
                          obstacle.permitted_sphere_groups.end(),
                          model.sphere_groups[sphere]) !=
                obstacle.permitted_sphere_groups.end())
                continue;
            const Eigen::Vector3d centre = centres.col(static_cast<int>(sphere));
            const double radius = model.arm_model->sphere_radius(sphere);
            const double measured = std::visit(
                [&](const auto& primitive) {
                    using Primitive = std::decay_t<decltype(primitive)>;
                    if constexpr (std::is_same_v<Primitive, AxisAlignedBox>) {
                        return IndependentBoxClearance(primitive, centre, radius);
                    } else {
                        return IndependentCylinderClearance(primitive, centre, radius);
                    }
                },
                obstacle.geometry);
            clearance = std::min(clearance, measured);
        }
    }
    return clearance;
}

void CheckExecutableIndependently(const PlannerModel& model,
                                  const std::vector<NamedStaticObstacle>& scene,
                                  double required_clearance_m,
                                  const gtsam::Pose3& requested_terminal,
                                  const TrajectoryResult& trajectory,
                                  PlanStatus expected_status,
                                  const std::string& scenario)
{
    Check(!trajectory.trajectory_pos.empty(), scenario + ": trajectory is nonempty");
    if (trajectory.trajectory_pos.empty())
        return;
    const Eigen::Matrix<double, 7, 1> final_q = trajectory.trajectory_pos.back();
    const Eigen::Isometry3d final_pose = IndependentToolPoseInMount(model, final_q);
    const double position_error =
        (final_pose.translation() - requested_terminal.translation()).norm();
    const double orientation_error = Eigen::AngleAxisd(
        requested_terminal.rotation().matrix().transpose() * final_pose.linear()).angle();
    if (expected_status == PlanStatus::kReached) {
        Check(position_error <= 0.001, scenario + ": independent terminal position is exact");
        Check(orientation_error <= 0.01, scenario + ": independent terminal orientation is exact");
    } else {
        Check(position_error > 0.001 || orientation_error > 0.01,
              scenario + ": independently measured shortfall is nonzero");
    }
    if (!scene.empty()) {
        double minimum = std::numeric_limits<double>::infinity();
        for (const auto& state : trajectory.trajectory_pos)
            minimum = std::min(minimum, IndependentMinimumClearance(model, scene, state));
        Check(minimum + 1e-9 >= required_clearance_m,
              scenario + ": independent primitive clearance passes");
    }
}

PlannerConfig TestConfig()
{
    PlannerConfig config = LoadPlannerConfig("../config/planner.yaml");
    config.scene.clear();
    config.minimum_clearance_m = 0.002;
    config.optimizer.preferred_clearance_m = 0.02;
    config.optimizer.collision_sigma = 0.005;
    config.optimizer.max_iterations = 200;
    config.path_following.validation_dt_s = 0.01;
    return config;
}

std::vector<CollisionSphereGroup> AllGroupsExcept(CollisionSphereGroup included)
{
    std::vector<CollisionSphereGroup> permitted;
    for (CollisionSphereGroup group :
         {CollisionSphereGroup::kMountInterface,
          CollisionSphereGroup::kProximalArm,
          CollisionSphereGroup::kUpperArm,
          CollisionSphereGroup::kForearm,
          CollisionSphereGroup::kTool})
        if (group != included)
            permitted.push_back(group);
    return permitted;
}

std::size_t LastSphereInGroup(const PlannerModel& model, CollisionSphereGroup group)
{
    for (std::size_t index = model.sphere_groups.size(); index-- > 0;)
        if (model.sphere_groups[index] == group)
            return index;
    throw std::runtime_error("fixture sphere group is absent");
}

NamedStaticObstacle BoxAtSphere(const PlannerModel& model,
                                const Eigen::Matrix<double, 7, 1>& q,
                                CollisionSphereGroup group,
                                const Eigen::Vector3d& half_extent,
                                const std::string& id)
{
    const std::size_t sphere = LastSphereInGroup(model, group);
    const auto centres = model.arm_model->sphereCentersMat(gtsam::Vector(q));
    NamedStaticObstacle obstacle;
    obstacle.id = id;
    obstacle.enabled = true;
    obstacle.geometry = AxisAlignedBox{
        centres.col(static_cast<int>(sphere)), half_extent};
    obstacle.permitted_sphere_groups = AllGroupsExcept(group);
    return obstacle;
}

gtsam::Pose3 PoseFromQ(const PlannerModel& model,
                       const Eigen::Matrix<double, 7, 1>& q)
{
    const Eigen::Isometry3d pose = IndependentToolPoseInMount(model, q);
    return gtsam::Pose3(gtsam::Rot3(pose.linear()), gtsam::Point3(pose.translation()));
}

void FixturePointDetour(const PlannerModel& model, const std::string& limits)
{
    PlannerConfig config = TestConfig();
    const auto q_start = StartQ();
    auto q_goal = q_start;
    q_goal(0) += 0.45;
    q_goal(3) -= 0.20;
    const auto q_mid = 0.5 * (q_start + q_goal);
    NamedStaticObstacle permitted_mount = BoxAtSphere(
        model, q_start, CollisionSphereGroup::kMountInterface,
        Eigen::Vector3d::Constant(0.001), "permitted_mount_overlap");
    permitted_mount.permitted_sphere_groups = {
        CollisionSphereGroup::kMountInterface,
        CollisionSphereGroup::kProximalArm,
        CollisionSphereGroup::kUpperArm,
        CollisionSphereGroup::kForearm,
        CollisionSphereGroup::kTool};
    config.scene.push_back(std::move(permitted_mount));
    config.scene.push_back(BoxAtSphere(
        model, q_mid, CollisionSphereGroup::kTool,
        Eigen::Vector3d::Constant(0.006), "point_blocker"));
    Check(IndependentMinimumClearance(
              model, {config.scene.front()}, q_start) >=
              config.minimum_clearance_m,
          "explicitly permitted mount-interface overlap is ignored");
    Check(IndependentMinimumClearance(
              model, {config.scene.back()}, q_mid) <
              config.minimum_clearance_m,
          "direct point seed crosses prohibited tool geometry");
    const gtsam::Pose3 goal = PoseFromQ(model, q_goal);
    PlanRequest request{q_start, std::nullopt, goal.translation(), goal.rotation().matrix()};
    const PlanOutcome result = SolveToPosition(model, request, limits, config);
    WritePointArtifacts("synthetic_point_detour", PlanStatus::kReached, true,
                        config, model, result);
    Check(result.status == PlanStatus::kReached && result.trajectory,
          "point detour reaches exact terminal");
    if (result.trajectory)
        CheckExecutableIndependently(model, config.scene, config.minimum_clearance_m,
                                     goal, *result.trajectory, result.status,
                                     "point detour");

    PlannerConfig prohibited = TestConfig();
    prohibited.scene.push_back(BoxAtSphere(
        model, q_start, CollisionSphereGroup::kProximalArm,
        Eigen::Vector3d::Constant(0.001), "prohibited_proximal_overlap"));
    Check(IndependentMinimumClearance(model, prohibited.scene, q_start) <
              prohibited.minimum_clearance_m,
          "prohibited proximal-arm overlap is present");
    const PlanOutcome rejected = SolveToPosition(model, request, limits, prohibited);
    Check(rejected.status == PlanStatus::kFailed && !rejected.trajectory &&
              rejected.candidate_attempts.empty(),
          "prohibited proximal-arm overlap fails before candidate search");
}

void FixtureTraceDetourAndRejoin(const PlannerModel& model, const std::string& limits)
{
    PlannerConfig config = TestConfig();
    const auto q_start = StartQ();
    const gtsam::Pose3 start = PoseFromQ(model, q_start);
    auto q_mid = q_start;
    q_mid(0) += 0.35;
    q_mid(3) -= 0.15;
    CartesianPath path;
    constexpr int kSegments = 24;
    for (int index = 0; index <= kSegments; ++index) {
        const double u = static_cast<double>(index) / kSegments;
        const Eigen::Matrix<double, 7, 1> q =
            q_start + std::sin(M_PI * u) * (q_mid - q_start);
        PathSample sample;
        sample.t_s = 8.0 * u;
        sample.pose = IndependentToolPoseInMount(model, q);
        path.samples.push_back(std::move(sample));
    }
    const auto start_centres = model.arm_model->sphereCentersMat(gtsam::Vector(q_start));
    const auto mid_centres = model.arm_model->sphereCentersMat(gtsam::Vector(q_mid));
    const std::size_t sphere =
        LastSphereInGroup(model, CollisionSphereGroup::kTool);
    const Eigen::Vector3d direction =
        (mid_centres.col(static_cast<int>(sphere)) -
         start_centres.col(static_cast<int>(sphere)))
            .normalized();
    const double sphere_radius = model.arm_model->sphere_radius(sphere);
    const double half = 0.0001;
    AxisAlignedBox blocker;
    blocker.half_extent = Eigen::Vector3d::Constant(half);
    blocker.center = mid_centres.col(static_cast<int>(sphere)) +
                     direction * (sphere_radius + config.minimum_clearance_m -
                                  0.006 + half);
    config.scene.push_back(NamedStaticObstacle{
        "trace_blocker", true, blocker,
        AllGroupsExcept(CollisionSphereGroup::kTool)});
    Check(IndependentMinimumClearance(model, config.scene, q_mid) <
              config.minimum_clearance_m,
          "requested trace crosses prohibited geometry");
    const PathPlanOutcome result =
        SolveAlongPath(model, path, q_start, std::nullopt, limits, config);
    WritePathArtifacts("synthetic_trace_detour_rejoin", PlanStatus::kReached,
                       config, model, path, result);
    Check(result.status == PlanStatus::kReached && result.trajectory,
          "trace detour rejoins exact terminal");
    if (result.trajectory) {
        CheckExecutableIndependently(model, config.scene, config.minimum_clearance_m,
                                     start, *result.trajectory, result.status,
                                     "trace detour");
        Check(result.validation.trace_max_position_m >
                  config.path_following.maximum_planning_error_m,
              "trace deviation above the reporting threshold remains quality evidence");
    }
}

void FixtureExactRoutesExhaustThenShorten(const PlannerModel& model,
                                          const std::string& limits)
{
    PlannerConfig config = TestConfig();
    const auto q_start = StartQ();
    auto q_goal = q_start;
    q_goal(0) += 0.35;
    const gtsam::Pose3 goal = PoseFromQ(model, q_goal);
    const std::size_t tool_sphere =
        LastSphereInGroup(model, CollisionSphereGroup::kTool);
    const auto start_centres =
        model.arm_model->sphereCentersMat(gtsam::Vector(q_start));
    const auto goal_centres =
        model.arm_model->sphereCentersMat(gtsam::Vector(q_goal));
    const Eigen::Vector3d motion =
        goal_centres.col(static_cast<int>(tool_sphere)) -
        start_centres.col(static_cast<int>(tool_sphere));
    const Eigen::Vector3d outward = motion.normalized();
    const double radius = model.arm_model->sphere_radius(tool_sphere);
    const double half = 0.0001;
    AxisAlignedBox shallow_blocker;
    shallow_blocker.half_extent = Eigen::Vector3d::Constant(half);
    shallow_blocker.center =
        goal_centres.col(static_cast<int>(tool_sphere)) +
        outward * (radius + config.minimum_clearance_m - 0.004 + half);
    config.scene.push_back(NamedStaticObstacle{
        "terminal_blocker", true, shallow_blocker,
        AllGroupsExcept(CollisionSphereGroup::kTool)});
    PlanRequest request{q_start, std::nullopt, goal.translation(), goal.rotation().matrix()};
    const PlanOutcome result = SolveToPosition(model, request, limits, config);
    WritePointArtifacts("synthetic_exact_exhausted_shortened",
                        PlanStatus::kGoalBlocked, true, config, model, result);
    Check(result.status == PlanStatus::kGoalBlocked && result.trajectory,
          "exhausted exact routes enter shortened search");
    Check(result.selected_candidate_attempt.has_value(),
          "shortened winner has selected evidence");
    if (result.selected_candidate_attempt) {
        const std::size_t selected_index = *result.selected_candidate_attempt;
        const CandidateEvidence& selected = result.candidate_attempts[selected_index];
        Check(selected.terminal_kind == PlanStatus::kGoalBlocked &&
                  selected.disposition == "best_validated_bounded_candidate" &&
                  selected.validation.executable,
              "selected evidence names the validated GOAL_BLOCKED winner");
        Check(std::any_of(
                  result.candidate_attempts.begin(),
                  result.candidate_attempts.begin() + selected_index,
                  [](const CandidateEvidence& attempt) {
                      return attempt.terminal_kind == PlanStatus::kReached &&
                             attempt.duration_attempt > 0;
                  }),
              "an exact GPMP2 attempt precedes the shortened winner");
    }
    if (result.trajectory)
        CheckExecutableIndependently(model, config.scene, config.minimum_clearance_m,
                                     goal, *result.trajectory, result.status,
                                     "shortened point");
}

void FixtureNoShortenedRoute(const PlannerModel& model, const std::string& limits)
{
    PlannerConfig config = TestConfig();
    const PlannerJointLimits parsed_limits = createJointLimits(limits);
    auto q_start = StartQ();
    q_start(1) = parsed_limits.position_rad.upper(1);
    Eigen::Matrix<double, 7, 1> qdot_start =
        Eigen::Matrix<double, 7, 1>::Zero();
    qdot_start(1) = 0.05;
    auto q_goal = q_start;
    q_goal(0) += 0.20;
    const gtsam::Pose3 goal = PoseFromQ(model, q_goal);
    PlanRequest request{q_start, qdot_start, goal.translation(), goal.rotation().matrix()};
    const PlanOutcome result = SolveToPosition(model, request, limits, config);
    WritePointArtifacts("synthetic_no_shortened_route", PlanStatus::kFailed,
                        false, config, model, result);
    Check(result.status == PlanStatus::kFailed && !result.trajectory,
          "exhausted shortened search returns FAILED without trajectory");
    Check(!result.terminal_candidate && !result.selected_candidate_attempt,
          "FAILED owns no terminal or selected trajectory evidence");
    Check(std::any_of(result.candidate_attempts.begin(), result.candidate_attempts.end(),
                      [](const CandidateEvidence& attempt) {
                          return attempt.duration_attempt > 0 &&
                                 !attempt.validation.executable;
                      }),
          "FAILED follows at least one invalid GPMP2 candidate");
}

void FixtureMovingStartDurationRepair(const PlannerModel& model,
                                      const std::string& limits)
{
    PlannerConfig config = TestConfig();
    config.motion.min_duration_s = 0.03;
    config.motion.nominal_speed_mps = 10.0;
    const auto q_start = StartQ();
    Eigen::Matrix<double, 7, 1> qdot_start;
    qdot_start << 0.08, -0.06, 0.05, -0.04, 0.03, -0.02, 0.01;
    auto q_goal = q_start;
    q_goal(0) += 0.20;
    q_goal(3) -= 0.10;
    const gtsam::Pose3 goal = PoseFromQ(model, q_goal);
    PlanRequest request{q_start, qdot_start, goal.translation(), goal.rotation().matrix()};
    const PlanOutcome result = SolveToPosition(model, request, limits, config);
    WritePointArtifacts("synthetic_moving_start_duration_repair",
                        PlanStatus::kReached, true, config, model, result);
    Check(result.status == PlanStatus::kReached && result.trajectory,
          "moving-start plan reaches exact terminal");
    if (!result.trajectory)
        return;
    Check((result.trajectory->trajectory_pos.front() - q_start).cwiseAbs().maxCoeff() < 1e-12,
          "moving-start q(0) is measured q");
    Check((result.trajectory->trajectory_vel.front() - qdot_start).cwiseAbs().maxCoeff() < 1e-12,
          "moving-start qdot(0) is measured qdot");
    Check(result.selected_candidate_attempt &&
              result.candidate_attempts[*result.selected_candidate_attempt].duration_attempt > 1,
          "dynamic excess triggers a fresh longer-duration solve");
    Check(result.validation.max_velocity_ratio <= 1.0 + 1e-9 &&
              result.validation.max_acceleration_ratio <= 1.0 + 1e-9,
          "repaired trajectory meets effective dynamic limits");
}

}  // namespace

int main(int argc, char** argv)
{
    Check(argc == 2 || argc == 3,
          "usage: test_obstacle_aware_planner dh_tool.yaml [artifact_root]");
    if (argc != 2 && argc != 3)
        return 1;
    if (argc == 3)
        artifact_root = std::filesystem::path(argv[2]);
    const PlannerModel model = LoadPlannerModel(argv[1], /*has_tool=*/true);
    const std::string limits = "../config/joint_limits.yaml";
    FixturePointDetour(model, limits);
    FixtureTraceDetourAndRejoin(model, limits);
    FixtureExactRoutesExhaustThenShorten(model, limits);
    FixtureNoShortenedRoute(model, limits);
    FixtureMovingStartDurationRepair(model, limits);
    if (failures == 0)
        std::puts("test_obstacle_aware_planner: all assertions passed");
    return failures == 0 ? 0 : 1;
}
