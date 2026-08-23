#include "PathAssembly.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Eigen::Vector3d DeterministicTangent(const Eigen::Vector3d& normal) {
    if (!normal.allFinite() || normal.norm() == 0.0)
        throw std::invalid_argument("bypass normal must be finite and nonzero");
    const Eigen::Vector3d unit_normal = normal.normalized();
    const Eigen::Vector3d basis[] = {Eigen::Vector3d::UnitX(),
                                     Eigen::Vector3d::UnitY(),
                                     Eigen::Vector3d::UnitZ()};
    int selected = 0;
    double smallest = std::abs(unit_normal.dot(basis[0]));
    for (int i = 1; i < 3; ++i) {
        const double candidate = std::abs(unit_normal.dot(basis[i]));
        if (candidate < smallest) {
            smallest = candidate;
            selected = i;
        }
    }
    const Eigen::Vector3d tangent =
        basis[selected] - unit_normal.dot(basis[selected]) * unit_normal;
    return tangent.normalized();
}

std::vector<JointConfiguration> PointBypassSeed(
    const JointConfiguration& start, const JointConfiguration& terminal,
    std::size_t support_count,
    const std::optional<std::pair<double, JointConfiguration>>& midpoint) {
    if (support_count < 2)
        throw std::invalid_argument("point bypass needs at least two support states");
    if (midpoint && !(midpoint->first > 0.0 && midpoint->first < 1.0))
        throw std::invalid_argument("point bypass midpoint fraction must be in (0,1)");
    std::vector<JointConfiguration> seed(support_count);
    for (std::size_t i = 0; i < support_count; ++i) {
        const double u = static_cast<double>(i) /
                         static_cast<double>(support_count - 1);
        if (!midpoint) {
            seed[i] = (1.0 - u) * start + u * terminal;
        } else if (u <= midpoint->first) {
            const double local = u / midpoint->first;
            seed[i] = (1.0 - local) * start + local * midpoint->second;
        } else {
            const double local = (u - midpoint->first) /
                                 (1.0 - midpoint->first);
            seed[i] = (1.0 - local) * midpoint->second + local * terminal;
        }
    }
    return seed;
}

CartesianPath TraceBypassSeed(const CartesianPath& path, double parameter_u,
                              const Eigen::Vector3d& displacement,
                              std::size_t sample_radius) {
    if (path.samples.empty()) return path;
    if (!std::isfinite(parameter_u) || !displacement.allFinite() ||
        sample_radius == 0)
        throw std::invalid_argument("trace bypass inputs must be finite");
    CartesianPath result = path;
    const double u = std::clamp(parameter_u, 0.0, 1.0);
    const std::size_t nearest = static_cast<std::size_t>(std::llround(
        u * static_cast<double>(path.samples.size() - 1)));
    for (std::size_t i = 0; i < result.samples.size(); ++i) {
        const std::size_t distance = i > nearest ? i - nearest : nearest - i;
        if (distance > sample_radius) continue;
        const double weight =
            1.0 - static_cast<double>(distance) /
                      static_cast<double>(sample_radius);
        result.samples[i].pose.translation() += weight * displacement;
    }
    return result;
}

AssembledPath AssembleCirclePlan(
    const CartesianPath& task_path,
    const std::vector<Eigen::Matrix<double, 7, 1>>& task_configurations,
    const Eigen::Matrix<double, 7, 1>& measured,
    const JointVelocityLimits& joint_velocity_limits,
    const ApproachPacing& pacing,
    const Eigen::Vector3d& rotation_sigma_rad,
    const Eigen::Vector3d& translation_sigma_m) {

    if (task_path.samples.size() < 2)
        throw std::invalid_argument(
            "a task path needs at least two samples, got " +
            std::to_string(task_path.samples.size()));
    if (task_configurations.size() != task_path.samples.size())
        throw std::invalid_argument(
            "task_configurations has " + std::to_string(task_configurations.size()) +
            " entries but the path has " + std::to_string(task_path.samples.size()) +
            " samples — they must be the continuation-IK solutions of THIS path");
    if (pacing.waypoints < 1)
        throw std::invalid_argument("the approach needs at least one interior waypoint");
    if (!(joint_velocity_limits.array() > 0.0).all())
        throw std::invalid_argument("joint velocity limits must all be positive");
    if (!(pacing.velocity_fraction > 0.0 && pacing.velocity_fraction <= 1.0))
        throw std::invalid_argument("approach velocity_fraction must be in (0, 1]");

    // ---- approach duration, from JOINT displacement -------------------
    // The approach has no commanded Cartesian route (that is the point of
    // leaving it unconstrained), so pacing it by Cartesian distance would
    // be timing a path that does not exist. Every joint must have time to
    // cover its own displacement at a fraction of its limit; the slowest
    // joint sets the duration.
    const Eigen::Matrix<double, 7, 1> displacement =
        (task_configurations.front() - measured).cwiseAbs();
    const Eigen::Matrix<double, 7, 1> per_joint_time =
        displacement.cwiseQuotient(joint_velocity_limits * pacing.velocity_fraction);
    const double approach_duration_s =
        std::max(pacing.minimum_duration_s, per_joint_time.maxCoeff());

    // GPMP2 assumes a CONSTANT support interval: densifyTrajectory computes
    // delta_t = total_duration / total_steps and interpolates every segment
    // with it. Non-uniform waypoint times are therefore not merely awkward,
    // they are silently wrong — the emitted timestamps would not be the ones
    // requested, and the validator would measure the traced phase over the
    // wrong window (observed 2026-08-07: an approach measured as if it were
    // the circle, reporting 110 mm of "path error" that was really the
    // approach doing its job).
    //
    // So the two phases are put on ONE uniform grid. The task path's own
    // spacing sets the interval, and the approach is given the whole number
    // of intervals that covers its required duration — which can only make
    // the approach slower than its floor, never faster.
    AssembledPath assembled;
    const double task_interval_s =
        task_path.duration_s() / static_cast<double>(task_path.samples.size() - 1);
    const std::size_t approach_states = std::max<std::size_t>(
        static_cast<std::size_t>(pacing.waypoints),
        static_cast<std::size_t>(std::ceil(approach_duration_s / task_interval_s)));
    const double uniform_approach_duration_s =
        static_cast<double>(approach_states) * task_interval_s;

    assembled.task_start_index = approach_states;  // waypoint 0 + (states-1) interior
    assembled.total_duration_s =
        uniform_approach_duration_s + task_path.duration_s();

    const std::size_t total =
        assembled.task_start_index + task_path.samples.size();
    assembled.waypoints.reserve(total);
    assembled.initial_configurations.reserve(total);

    // ---- waypoint 0: the measured configuration -----------------------
    // No pose prior. The optimiser applies an exact measured-position
    // equality here, and a competing pose prior would pull against the one
    // controller's splice guard actually requires.
    OptimisationWaypoint start;
    start.time_s = 0.0;
    assembled.waypoints.push_back(start);
    assembled.initial_configurations.push_back(measured);

    // ---- approach: unconstrained --------------------------------------
    // Initial guess only. Joint-space interpolation toward the first task
    // solution gives the optimiser somewhere sensible to start; with no
    // pose prior it is free to move these anywhere smoothness and obstacle
    // avoidance prefer, so this is NOT a commanded straight line in any
    // space.
    for (std::size_t i = 1; i < approach_states; ++i) {
        const double fraction =
            static_cast<double>(i) / static_cast<double>(approach_states);
        OptimisationWaypoint waypoint;
        waypoint.time_s = static_cast<double>(i) * task_interval_s;
        assembled.waypoints.push_back(waypoint);  // pose_prior stays empty
        assembled.initial_configurations.push_back(
            measured + fraction * (task_configurations.front() - measured));
    }

    // ---- task: every support point constrained ------------------------
    for (std::size_t i = 0; i < task_path.samples.size(); ++i) {
        OptimisationWaypoint waypoint;
        waypoint.time_s = uniform_approach_duration_s + task_path.samples[i].t_s;

        PosePrior prior;
        prior.target = task_path.samples[i].pose;
        prior.rotation_sigma_rad = rotation_sigma_rad;
        prior.translation_sigma_m = translation_sigma_m;
        waypoint.pose_prior = prior;

        assembled.waypoints.push_back(waypoint);
        assembled.initial_configurations.push_back(task_configurations[i]);
    }

    // Rest at the task-path start: the arm stops before it begins tracing,
    // which is simpler and more repeatable than blending an arbitrary
    // approach direction into the path tangent. Hermite through a
    // zero-velocity support state genuinely comes to rest, so no repeated
    // waypoints are needed. The optimiser adds the first and last itself.
    assembled.zero_velocity_indices.push_back(assembled.task_start_index);
    assembled.zero_velocity_indices.push_back(assembled.waypoints.size() - 1);

    // The grid must be uniform, or densifyTrajectory's constant delta_t
    // silently disagrees with these timestamps. Checked rather than assumed.
    for (std::size_t i = 1; i < assembled.waypoints.size(); ++i) {
        const double interval =
            assembled.waypoints[i].time_s - assembled.waypoints[i - 1].time_s;
        if (std::abs(interval - task_interval_s) > 1e-9 * task_interval_s)
            throw std::logic_error(
                "assembled waypoints are not on a uniform time grid at index " +
                std::to_string(i) + " — GPMP2 interpolates with a constant "
                "delta_t and would emit different timestamps than requested");
    }

    std::size_t bad = 0;
    if (!WaypointTimesAreMonotonic(assembled.waypoints, &bad))
        throw std::invalid_argument(
            "assembled waypoint times are not strictly increasing at index " +
            std::to_string(bad) + " — check the task path's own timing");
    return assembled;
}
