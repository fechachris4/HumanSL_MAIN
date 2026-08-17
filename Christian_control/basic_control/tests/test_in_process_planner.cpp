// Direct planner-worker ownership and shutdown characterization.
#undef NDEBUG

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "InProcessPlanner.h"

namespace {

WorldCartesianTrajectory MakeTrajectory(std::uint64_t id,
                                        std::uint64_t sequence)
{
    WorldCartesianTrajectory trajectory;
    trajectory.trajectory_id = id;
    trajectory.planner_vicon_sequence = sequence;
    WorldCartesianTrajectoryPoint first;
    first.t_from_start_s = 0.0;
    first.position_world_m = Eigen::Vector3d(1.0, 2.0, 3.0);
    first.orientation_world = Eigen::Quaterniond::Identity();
    WorldCartesianTrajectoryPoint last = first;
    last.t_from_start_s = 0.2;
    last.position_world_m = Eigen::Vector3d(1.1, 2.1, 3.1);
    last.arrival_eligible = true;
    trajectory.points = {first, last};
    return trajectory;
}

PlannerSolveResult FakeSolve(const PlanningRequest& request,
                             const PlannerRuntimeConfig&,
                             std::ostream&)
{
    PlannerSolveResult result;
    result.exit_code = 0;
    result.trajectory = std::make_unique<WorldCartesianTrajectory>(
        MakeTrajectory(request.request_id, request.vicon_sequence));
    return result;
}

std::atomic<bool> slow_entered{false};
std::atomic<bool> slow_release{false};

PlannerSolveResult SlowSolve(const PlanningRequest& request,
                             const PlannerRuntimeConfig& config,
                             std::ostream& diagnostics)
{
    slow_entered.store(true, std::memory_order_release);
    while (!slow_release.load(std::memory_order_acquire))
        std::this_thread::yield();
    return FakeSolve(request, config, diagnostics);
}

PlanningRequest Request(std::uint64_t id)
{
    PlanningRequest request;
    request.request_id = id;
    request.vicon_sequence = id + 100;
    request.q_rad.setZero();
    return request;
}

}  // namespace

int main()
{
    PlanningRequestSlot requests;
    CartesianTrajectoryMailbox trajectories;
    PlannerRuntimeConfig config;

    requests.Publish(Request(17));
    std::atomic<bool> stop{false};
    std::thread worker(RunInProcessPlanner, std::ref(requests),
                       std::ref(trajectories), std::cref(config),
                       std::cref(stop), &FakeSolve);

    std::unique_ptr<WorldCartesianTrajectory> received;
    for (int attempt = 0; attempt < 100 && !received; ++attempt) {
        received = trajectories.Take();
        if (!received)
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    assert(received);
    assert(received->trajectory_id == 17);
    assert(received->planner_vicon_sequence == 117);
    assert(received->points.size() == 2);
    assert(received->points[1].t_from_start_s == 0.2);
    assert(received->points[1].position_world_m.isApprox(
        Eigen::Vector3d(1.1, 2.1, 3.1), 0.0));
    received.reset();
    stop.store(true, std::memory_order_release);
    worker.join();

    // A result completed after stop is requested must not be published.
    PlanningRequestSlot slow_requests;
    CartesianTrajectoryMailbox slow_trajectories;
    std::atomic<bool> slow_stop{false};
    slow_entered.store(false, std::memory_order_release);
    slow_release.store(false, std::memory_order_release);
    slow_requests.Publish(Request(23));
    std::thread slow_worker(RunInProcessPlanner, std::ref(slow_requests),
                            std::ref(slow_trajectories), std::cref(config),
                            std::cref(slow_stop), &SlowSolve);
    for (int attempt = 0; attempt < 100 &&
         !slow_entered.load(std::memory_order_acquire); ++attempt)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(slow_entered.load(std::memory_order_acquire));
    slow_stop.store(true, std::memory_order_release);
    slow_release.store(true, std::memory_order_release);
    slow_worker.join();
    assert(!slow_trajectories.Take());

    return 0;
}
