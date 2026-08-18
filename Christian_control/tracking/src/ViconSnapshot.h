#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <vector>

// One labelled marker in a validated Vicon frame, position in metres,
// Vicon-world frame. valid=false means position_m must not be used;
// invalid_reason says why (e.g. "occluded").
struct MarkerSample {
    std::string name;
    Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
    bool valid = false;
    std::string invalid_reason;
};

// One subject/segment pose in a validated Vicon frame, Vicon-world frame,
// metres and a checked quaternion. valid=false means position_m and
// orientation must not be used.
struct SegmentSample {
    std::string subject_name;
    std::string segment_name;
    Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool valid = false;
    std::string invalid_reason;
};

// One validated Vicon frame: every marker and segment the SDK returned,
// converted to metres, with quaternions checked (finite, unit-norm)
// rather than silently normalised. No SDK types, no I/O — a plain value.
struct ViconSnapshot {
    unsigned int frame_number = 0;
    double host_time_s = 0.0;      // steady_clock at the read, seconds
    double frame_rate_hz = 0.0;    // SDK-reported server rate
    double latency_total_s = 0.0;  // SDK-reported total latency
    std::vector<MarkerSample> markers;
    std::vector<SegmentSample> segments;
};
