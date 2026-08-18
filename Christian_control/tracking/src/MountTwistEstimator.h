#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "ViconSnapshot.h"

// Filtered spatial twist of the tracked Mount segment, expressed in the
// Vicon-world frame. `valid` requires two consecutive advancing valid Vicon
// frames. `updated=false` with valid=true means ClientPull returned the same
// frame again: this is the previously estimated twist under zero-order hold,
// never a second finite difference of reused data.
struct MountTwistEstimate {
    Eigen::Vector3d linear_m_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_rad_s = Eigen::Vector3d::Zero();
    std::uint32_t source_frame_number = 0;
    bool valid = false;
    bool updated = false;
};

// Stateful, I/O-free estimator for one Vicon stream. The finite difference
// uses the server frame count/rate rather than host receive jitter:
//
//   dt = (frame_k - frame_prev) / frame_rate_hz
//   v  = (p_k - p_prev) / dt
//   w  = Log(R_k R_prev^T) / dt                    (world axes)
//
// Both halves are filtered by the exact first-order discretisation
// alpha = 1-exp(-dt/tau). tau=0 selects no filtering (alpha=1), useful for
// deterministic tests. A discontinuity invalidates the estimate rather than
// differentiating across missing, occluded, out-of-order, or prolonged data.
class MountTwistEstimator {
public:
    MountTwistEstimator(double filter_tau_s, double reset_gap_s);

    MountTwistEstimate Update(const ViconSnapshot& snapshot);
    void Reset() noexcept;

private:
    void Seed(const SegmentSample& mount, std::uint32_t frame_number) noexcept;

    double filter_tau_s_ = 0.0;
    double reset_gap_s_ = 0.0;
    bool seeded_ = false;
    std::uint32_t previous_frame_number_ = 0;
    Eigen::Vector3d previous_position_m_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d previous_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d filtered_linear_m_s_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d filtered_angular_rad_s_ = Eigen::Vector3d::Zero();
    MountTwistEstimate last_estimate_;
};
