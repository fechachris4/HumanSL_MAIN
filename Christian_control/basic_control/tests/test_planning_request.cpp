#undef NDEBUG

#include <cassert>
#include <limits>

#include "PlanningRequest.h"
#include "PlanningRequestSlot.h"

PlanningRequest Request(std::uint64_t id)
{
    PlanningRequest request;
    request.request_id = id;
    request.vicon_sequence = 42;
    request.vicon_frame_number = 99;
    request.receive_steady_s = 10.0;
    request.age_s = 0.004;
    request.world_T_mount.translation() = Eigen::Vector3d(1, 2, 3);
    request.world_T_mount.linear() =
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    request.q_rad << 1, 2, 3, 4, 5, 6, 7;
    return request;
}

int main()
{
    const PlanningRequest expected = Request(7);
    assert(!ValidatePlanningRequest(expected).has_value());

    PlanningRequest stale = Request(8);
    stale.age_s = 0.051;
    assert(ValidatePlanningRequest(stale).has_value());
    PlanningRequest no_sequence = Request(8);
    no_sequence.vicon_sequence = 0;
    assert(ValidatePlanningRequest(no_sequence).has_value());
    PlanningRequest nonfinite = Request(8);
    nonfinite.q_rad[3] = std::numeric_limits<double>::quiet_NaN();
    assert(ValidatePlanningRequest(nonfinite).has_value());

    PlanningRequestSlot slot;
    slot.Publish(Request(1));
    slot.Publish(Request(2));
    slot.Publish(Request(3));
    PlanningRequest latest;
    assert(slot.TakeLatest(latest) && latest.request_id == 3);
    assert(!slot.TakeLatest(latest));
    return 0;
}
