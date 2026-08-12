#include "SnapshotBuilder.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace {

MarkerData MakeMarker(const std::string& name, double x, double y, double z,
                       bool occluded) {
    MarkerData m;
    m.name = name;
    m.x = x;
    m.y = y;
    m.z = z;
    m.occluded = occluded;
    return m;
}

SegmentData MakeSegment(const std::string& subject, const std::string& segment,
                         double x, double y, double z, double qx, double qy,
                         double qz, double qw, bool occluded) {
    SegmentData s;
    s.subject_name = subject;
    s.segment_name = segment;
    s.x = x;
    s.y = y;
    s.z = z;
    s.qx = qx;
    s.qy = qy;
    s.qz = qz;
    s.qw = qw;
    s.occluded = occluded;
    return s;
}

}  // namespace

int main() {
    // Frame metadata passes through unchanged.
    {
        const auto snapshot = BuildSnapshot(42, 1.5, 100.0, 0.02, {}, {});
        assert(snapshot.frame_number == 42);
        assert(snapshot.host_time_s == 1.5);
        assert(snapshot.frame_rate_hz == 100.0);
        assert(snapshot.latency_total_s == 0.02);
        assert(snapshot.markers.empty());
        assert(snapshot.segments.empty());
    }

    // Valid marker: millimetres convert to metres exactly.
    {
        const std::vector<MarkerData> markers = {
            MakeMarker("M1", 1000.0, 2000.0, 3000.0, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, markers, {});
        assert(snapshot.markers.size() == 1);
        const auto& m = snapshot.markers[0];
        assert(m.name == "M1");
        assert(m.valid);
        assert(m.invalid_reason.empty());
        assert(std::abs(m.position_m.x() - 1.0) < 1e-12);
        assert(std::abs(m.position_m.y() - 2.0) < 1e-12);
        assert(std::abs(m.position_m.z() - 3.0) < 1e-12);
    }

    // Occluded marker: invalid, reason stated.
    {
        const std::vector<MarkerData> markers = {
            MakeMarker("M2", 0.0, 0.0, 0.0, true)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, markers, {});
        assert(!snapshot.markers[0].valid);
        assert(snapshot.markers[0].invalid_reason == "occluded");
    }

    // Valid segment: unit quaternion, millimetres convert to metres.
    {
        const std::vector<SegmentData> segments = {MakeSegment(
            "Dr Octopus Christian", "Mount", 100.0, 200.0, 300.0, 0.0, 0.0,
            0.0, 1.0, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(snapshot.segments.size() == 1);
        const auto& s = snapshot.segments[0];
        assert(s.subject_name == "Dr Octopus Christian");
        assert(s.segment_name == "Mount");
        assert(s.valid);
        assert(s.invalid_reason.empty());
        assert(std::abs(s.position_m.x() - 0.1) < 1e-12);
        assert(std::abs(s.position_m.y() - 0.2) < 1e-12);
        assert(std::abs(s.position_m.z() - 0.3) < 1e-12);
        assert(std::abs(s.orientation.w() - 1.0) < 1e-12);
    }

    // Occluded segment: invalid, reason "occluded" (checked before the
    // quaternion, since occlusion is the SDK's own definitive signal).
    {
        const std::vector<SegmentData> segments = {
            MakeSegment("S", "Seg", 0, 0, 0, 0, 0, 0, 1, true)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(!snapshot.segments[0].valid);
        assert(snapshot.segments[0].invalid_reason == "occluded");
    }

    // Non-finite quaternion: invalid, reason mentions "finite".
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const std::vector<SegmentData> segments = {
            MakeSegment("S", "Seg", 0, 0, 0, nan, 0, 0, 1, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(!snapshot.segments[0].valid);
        assert(snapshot.segments[0].invalid_reason == "non-finite quaternion");
    }

    // Non-unit-norm quaternion: invalid, reason mentions "unit-norm".
    {
        const std::vector<SegmentData> segments = {
            MakeSegment("S", "Seg", 0, 0, 0, 2.0, 0, 0, 0, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(!snapshot.segments[0].valid);
        assert(snapshot.segments[0].invalid_reason == "quaternion not unit-norm");
    }

    return 0;
}
