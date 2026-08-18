// CartesianTrajectoryMailbox — one-slot typed handoff for world-frame
// Cartesian planner trajectories.

#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include "WorldCartesianTrajectory.h"

class CartesianTrajectoryMailbox
{
public:
    CartesianTrajectoryMailbox() = default;
    ~CartesianTrajectoryMailbox();
    CartesianTrajectoryMailbox(const CartesianTrajectoryMailbox&) = delete;
    CartesianTrajectoryMailbox& operator=(const CartesianTrajectoryMailbox&) =
        delete;

    // Planner worker. Replacing an unread plan deletes that one displaced
    // bounded trajectory here, outside the 500 Hz thread.
    void Publish(std::unique_ptr<WorldCartesianTrajectory> trajectory);

    // Control thread. One atomic exchange; the caller owns the result. The
    // returned allocation must later be passed to Retire, not destroyed in
    // the cyclic path.
    std::unique_ptr<WorldCartesianTrajectory> Take();

    // Control thread: transfer a consumed/rejected/cancelled allocation to an
    // intrusive lock-free retire stack. No allocation, destruction, or wait.
    void Retire(std::unique_ptr<WorldCartesianTrajectory> trajectory) noexcept;

    // Non-real-time planner thread (or teardown/tests): reclaim every retired
    // trajectory. Returns the number destroyed.
    std::size_t DeleteRetired() noexcept;

private:
    std::atomic<WorldCartesianTrajectory*> slot_{nullptr};
    std::atomic<WorldCartesianTrajectory*> retired_{nullptr};
};
