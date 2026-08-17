#include "CartesianTrajectoryMailbox.h"

CartesianTrajectoryMailbox::~CartesianTrajectoryMailbox()
{
    delete slot_.exchange(nullptr, std::memory_order_acq_rel);
    DeleteRetired();
}

void CartesianTrajectoryMailbox::Publish(
    std::unique_ptr<WorldCartesianTrajectory> trajectory)
{
    delete slot_.exchange(trajectory.release(), std::memory_order_acq_rel);
}

std::unique_ptr<WorldCartesianTrajectory> CartesianTrajectoryMailbox::Take()
{
    return std::unique_ptr<WorldCartesianTrajectory>(
        slot_.exchange(nullptr, std::memory_order_acq_rel));
}

void CartesianTrajectoryMailbox::Retire(
    std::unique_ptr<WorldCartesianTrajectory> trajectory) noexcept
{
    WorldCartesianTrajectory* raw = trajectory.release();
    if (!raw)
        return;
    raw->reclamation_next = retired_.load(std::memory_order_relaxed);
    while (!retired_.compare_exchange_weak(
        raw->reclamation_next, raw, std::memory_order_release,
        std::memory_order_relaxed)) {
    }
}

std::size_t CartesianTrajectoryMailbox::DeleteRetired() noexcept
{
    WorldCartesianTrajectory* trajectory =
        retired_.exchange(nullptr, std::memory_order_acquire);
    std::size_t deleted = 0;
    while (trajectory) {
        WorldCartesianTrajectory* next = trajectory->reclamation_next;
        delete trajectory;
        trajectory = next;
        ++deleted;
    }
    return deleted;
}
