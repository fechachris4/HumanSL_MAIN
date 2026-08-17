// Typed mailbox characterization. The production controller receives the
// planner's WorldCartesianTrajectory by ownership transfer; no pipe or text
// parser is part of this test.
#undef NDEBUG

#include <cassert>
#include <memory>

#include "CartesianTrajectoryMailbox.h"

namespace {

std::unique_ptr<WorldCartesianTrajectory> MakeTrajectory(
    std::uint64_t trajectory_id, std::uint64_t vicon_sequence)
{
    auto trajectory = std::make_unique<WorldCartesianTrajectory>();
    trajectory->trajectory_id = trajectory_id;
    trajectory->planner_vicon_sequence = vicon_sequence;
    WorldCartesianTrajectoryPoint first;
    first.t_from_start_s = 0.0;
    first.position_world_m = Eigen::Vector3d(1.0, 2.0, 3.0);
    first.orientation_world = Eigen::Quaterniond::Identity();
    WorldCartesianTrajectoryPoint last = first;
    last.t_from_start_s = 0.25;
    last.position_world_m = Eigen::Vector3d(1.25, 2.5, 3.75);
    last.arrival_eligible = true;
    trajectory->points = {first, last};
    return trajectory;
}

}  // namespace

int main()
{
    CartesianTrajectoryMailbox mailbox;

    std::unique_ptr<WorldCartesianTrajectory> first = MakeTrajectory(1, 42);
    mailbox.Publish(std::move(first));
    assert(!first);

    std::unique_ptr<WorldCartesianTrajectory> second = MakeTrajectory(2, 99);
    mailbox.Publish(std::move(second));
    assert(!second);

    std::unique_ptr<WorldCartesianTrajectory> received = mailbox.Take();
    assert(received);
    assert(received->trajectory_id == 2);
    assert(received->planner_vicon_sequence == 99);
    assert(received->points.size() == 2);
    assert(received->points[0].t_from_start_s == 0.0);
    assert(received->points[1].t_from_start_s == 0.25);
    assert(received->points[1].position_world_m.isApprox(
        Eigen::Vector3d(1.25, 2.5, 3.75), 0.0));
    assert(received->points[1].arrival_eligible);
    assert(!mailbox.Take());

    mailbox.Retire(std::move(received));
    assert(!received);
    assert(mailbox.DeleteRetired() == 1);
    assert(mailbox.DeleteRetired() == 0);
    return 0;
}
