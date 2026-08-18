// Hardware-free tests for trajectory assembly: the free approach and the
// constrained task phase in one problem.
//
// The check that matters most is the UNIFORM TIME GRID. GPMP2's
// densifyTrajectory interpolates with a single delta_t derived from the
// total duration and step count, so non-uniform waypoint times are not
// merely awkward — the emitted timestamps silently stop matching the
// requested ones. On 2026-08-07 that made the validator measure the
// approach as though it were the circle and report 110 mm of "path error"
// that was really the approach doing its job. Assembly now enforces the
// grid, and this test is what keeps it enforced.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "CartesianPath.h"
#include "PathAssembly.h"
#include "PathFrames.h"

namespace {

CartesianPath MakeCircle(int samples, double duration_s) {
    CircleSpec spec;
    spec.centre_m = Eigen::Vector3d(0.4, 0.0, 0.3);
    spec.radius_m = 0.05;
    spec.normal = Eigen::Vector3d::UnitZ();
    spec.samples = samples;
    spec.duration_s = duration_s;
    return GenerateCircle(spec);
}

std::vector<Eigen::Matrix<double, 7, 1>> FakeSolutions(std::size_t n) {
    std::vector<Eigen::Matrix<double, 7, 1>> out;
    for (std::size_t i = 0; i < n; ++i)
        out.push_back(Eigen::Matrix<double, 7, 1>::Constant(0.1 * static_cast<double>(i)));
    return out;
}

}  // namespace

int main() {
    // The planner's single frame boundary: a path declared in Mount is
    // carried through the immutable Vicon snapshot into WORLD, including
    // both translation and orientation. A WORLD path is already resolved
    // and must pass through unchanged.
    const Eigen::Isometry3d world_T_mount =
        Eigen::Translation3d(1.0, 2.0, 3.0) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    CartesianPath mount_path;
    mount_path.frame = config::ReferenceFrame::kMount;
    PathSample mount_sample;
    mount_sample.pose = Eigen::Translation3d(1.0, 0.0, 0.0) *
                        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
    mount_path.samples.push_back(mount_sample);
    const CartesianPath world_path = PathToWorld(mount_path, world_T_mount);
    assert(world_path.frame == config::ReferenceFrame::kWorld);
    assert((world_path.samples[0].pose.translation() -
            Eigen::Vector3d(1.0, 3.0, 3.0)).norm() < 1e-12);
    assert((world_path.samples[0].pose.linear() -
            (world_T_mount.linear() * mount_sample.pose.linear())).norm() < 1e-12);

    CartesianPath already_world = world_path;
    assert((PathToWorld(already_world, world_T_mount).samples[0].pose.matrix() -
            already_world.samples[0].pose.matrix()).norm() < 1e-12);

    const CartesianPath circle = MakeCircle(12, 12.0);
    const auto solutions = FakeSolutions(circle.samples.size());
    const Eigen::Matrix<double, 7, 1> measured =
        Eigen::Matrix<double, 7, 1>::Constant(-0.5);
    const JointVelocityLimits limits = JointVelocityLimits::Constant(1.0);
    ApproachPacing pacing;

    const AssembledPath plan = AssembleCirclePlan(
        circle, solutions, measured, limits, pacing,
        Eigen::Vector3d::Constant(0.01), Eigen::Vector3d::Constant(0.005));

    // --- the uniform grid, checked independently of the assembler's own
    //     internal guard.
    assert(plan.waypoints.size() >= 2);
    const double interval = plan.waypoints[1].time_s - plan.waypoints[0].time_s;
    assert(interval > 0.0);
    for (std::size_t i = 1; i < plan.waypoints.size(); ++i) {
        const double step = plan.waypoints[i].time_s - plan.waypoints[i - 1].time_s;
        assert(std::abs(step - interval) < 1e-9 &&
               "waypoints must lie on ONE uniform time grid — GPMP2 assumes a "
               "constant support interval");
    }
    assert(plan.waypoints.front().time_s == 0.0 && "the grid must start at zero");

    // --- the interval must equal the task path's own spacing, so the
    //     circle's requested timing survives assembly unchanged.
    const double task_interval =
        circle.duration_s() / static_cast<double>(circle.samples.size() - 1);
    assert(std::abs(interval - task_interval) < 1e-9 &&
           "the grid interval must be the task path's own sample spacing");

    // --- which waypoints are constrained, and which are free.
    assert(!plan.waypoints[0].pose_prior.has_value() &&
           "waypoint 0 carries the stiff JOINT prior; a competing pose prior "
           "would pull against the splice guard");
    for (std::size_t i = 1; i < plan.task_start_index; ++i)
        assert(!plan.waypoints[i].pose_prior.has_value() &&
               "the approach is task-space UNCONSTRAINED");
    for (std::size_t i = plan.task_start_index; i < plan.waypoints.size(); ++i)
        assert(plan.waypoints[i].pose_prior.has_value() &&
               "every traced waypoint must carry a pose prior");

    // Exactly one constrained waypoint per task sample, none lost or added.
    assert(plan.waypoints.size() - plan.task_start_index == circle.samples.size());

    // --- the traced targets must be the requested poses, unaltered.
    for (std::size_t i = 0; i < circle.samples.size(); ++i) {
        const auto& prior = plan.waypoints[plan.task_start_index + i].pose_prior;
        assert((prior->target.translation() -
                circle.samples[i].pose.translation()).norm() < 1e-12);
    }

    // --- rest states: start, task-path start, end. Hermite through a
    //     zero-velocity support state comes to rest without duplicate points.
    const auto has = [&](std::size_t index) {
        return std::find(plan.zero_velocity_indices.begin(),
                         plan.zero_velocity_indices.end(),
                         index) != plan.zero_velocity_indices.end();
    };
    assert(has(0) && "the trajectory must start at rest");
    assert(has(plan.task_start_index) &&
           "the arm must STOP before it begins tracing");
    assert(has(plan.waypoints.size() - 1) && "the trajectory must end at rest");

    // --- the initial guess starts where the arm actually is, and arrives at
    //     the first traced solution.
    assert((plan.initial_configurations.front() - measured).norm() < 1e-12);
    assert((plan.initial_configurations[plan.task_start_index] -
            solutions.front()).norm() < 1e-12);

    // --- a large joint displacement must lengthen the approach: pacing
    //     comes from JOINT travel over joint limits, not Cartesian distance.
    auto far = FakeSolutions(circle.samples.size());
    for (auto& q : far) q += Eigen::Matrix<double, 7, 1>::Constant(3.0);
    const AssembledPath longer = AssembleCirclePlan(
        circle, far, measured, limits, pacing,
        Eigen::Vector3d::Constant(0.01), Eigen::Vector3d::Constant(0.005));
    assert(longer.task_start_index > plan.task_start_index &&
           "a farther first solution must buy the approach more time");

    // --- malformed input is refused rather than silently reshaped.
    bool threw = false;
    try {
        AssembleCirclePlan(circle, FakeSolutions(3), measured, limits, pacing,
                           Eigen::Vector3d::Constant(0.01),
                           Eigen::Vector3d::Constant(0.005));
    } catch (const std::invalid_argument&) { threw = true; }
    assert(threw && "a solution count that does not match the path is refused");

    std::printf("all path-assembly tests passed\n");
    return 0;
}
