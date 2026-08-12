#include "SnapshotBuilder.h"

#include <cmath>

namespace {

constexpr double kMmToM = 0.001;

// How far a quaternion's norm may sit from 1 and still be trusted as
// numerically valid. The SDK does not guarantee unit norm on every frame;
// tighter than this starts rejecting good data, looser risks treating
// garbage as a rotation.
constexpr double kQuaternionNormTolerance = 1e-3;

bool IsFiniteQuaternion(double qx, double qy, double qz, double qw) {
    return std::isfinite(qx) && std::isfinite(qy) && std::isfinite(qz) &&
           std::isfinite(qw);
}

}  // namespace

ViconSnapshot BuildSnapshot(unsigned int frame_number, double host_time_s,
                             double frame_rate_hz, double latency_total_s,
                             const std::vector<MarkerData>& markers,
                             const std::vector<SegmentData>& segments) {
    ViconSnapshot snapshot;
    snapshot.frame_number = frame_number;
    snapshot.host_time_s = host_time_s;
    snapshot.frame_rate_hz = frame_rate_hz;
    snapshot.latency_total_s = latency_total_s;

    snapshot.markers.reserve(markers.size());
    for (const auto& marker : markers) {
        MarkerSample sample;
        sample.name = marker.name;
        sample.position_m = Eigen::Vector3d(marker.x * kMmToM, marker.y * kMmToM,
                                             marker.z * kMmToM);
        if (marker.occluded) {
            sample.valid = false;
            sample.invalid_reason = "occluded";
        } else {
            sample.valid = true;
        }
        snapshot.markers.push_back(sample);
    }

    snapshot.segments.reserve(segments.size());
    for (const auto& segment : segments) {
        SegmentSample sample;
        sample.subject_name = segment.subject_name;
        sample.segment_name = segment.segment_name;
        sample.position_m = Eigen::Vector3d(segment.x * kMmToM, segment.y * kMmToM,
                                             segment.z * kMmToM);
        sample.orientation = Eigen::Quaterniond(segment.qw, segment.qx, segment.qy,
                                                 segment.qz);

        if (segment.occluded) {
            sample.valid = false;
            sample.invalid_reason = "occluded";
        } else if (!IsFiniteQuaternion(segment.qx, segment.qy, segment.qz,
                                        segment.qw)) {
            sample.valid = false;
            sample.invalid_reason = "non-finite quaternion";
        } else if (std::abs(sample.orientation.norm() - 1.0) >
                   kQuaternionNormTolerance) {
            sample.valid = false;
            sample.invalid_reason = "quaternion not unit-norm";
        } else {
            sample.valid = true;
        }
        snapshot.segments.push_back(sample);
    }

    return snapshot;
}
