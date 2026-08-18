#include "ViconSnapshot.h"

#include <cassert>

int main() {
    MarkerSample marker;
    assert(marker.name.empty());
    assert(marker.position_m == Eigen::Vector3d::Zero());
    assert(!marker.valid);
    assert(marker.invalid_reason.empty());

    SegmentSample segment;
    assert(segment.subject_name.empty());
    assert(segment.segment_name.empty());
    assert(segment.position_m == Eigen::Vector3d::Zero());
    assert(segment.orientation.coeffs() == Eigen::Quaterniond::Identity().coeffs());
    assert(!segment.valid);
    assert(segment.invalid_reason.empty());

    ViconSnapshot snapshot;
    assert(snapshot.frame_number == 0);
    assert(snapshot.host_time_s == 0.0);
    assert(snapshot.frame_rate_hz == 0.0);
    assert(snapshot.latency_total_s == 0.0);
    assert(snapshot.markers.empty());
    assert(snapshot.segments.empty());
    return 0;
}
