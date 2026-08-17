#include "MountTwistEstimator.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

namespace {

constexpr double kQuaternionNormTolerance = 1e-3;

const SegmentSample* ValidMount(const ViconSnapshot& snapshot) {
    for (const SegmentSample& segment : snapshot.segments) {
        if (segment.segment_name != "Mount")
            continue;
        if (!segment.valid || !segment.position_m.allFinite() ||
            !segment.orientation.coeffs().allFinite() ||
            std::abs(segment.orientation.norm() - 1.0) >
                kQuaternionNormTolerance)
            return nullptr;
        return &segment;
    }
    return nullptr;
}

Eigen::Vector3d RotationLog(const Eigen::Matrix3d& rotation) {
    const Eigen::AngleAxisd angle_axis(rotation);
    if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite())
        return Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    return angle_axis.angle() * angle_axis.axis();
}

} // namespace

MountTwistEstimator::MountTwistEstimator(double filter_tau_s,
                                         double reset_gap_s)
    : filter_tau_s_(filter_tau_s), reset_gap_s_(reset_gap_s) {
    if (!std::isfinite(filter_tau_s_) || filter_tau_s_ < 0.0)
        throw std::invalid_argument(
            "Mount twist filter time constant must be finite and non-negative");
    if (!std::isfinite(reset_gap_s_) || reset_gap_s_ <= 0.0)
        throw std::invalid_argument(
            "Mount twist reset gap must be finite and positive");
}

void MountTwistEstimator::Reset() noexcept {
    seeded_ = false;
    previous_frame_number_ = 0;
    previous_position_m_.setZero();
    previous_rotation_.setIdentity();
    filtered_linear_m_s_.setZero();
    filtered_angular_rad_s_.setZero();
    last_estimate_ = MountTwistEstimate{};
}

void MountTwistEstimator::Seed(const SegmentSample& mount,
                               std::uint32_t frame_number) noexcept {
    seeded_ = true;
    previous_frame_number_ = frame_number;
    previous_position_m_ = mount.position_m;
    previous_rotation_ = mount.orientation.toRotationMatrix();
}

MountTwistEstimate MountTwistEstimator::Update(
    const ViconSnapshot& snapshot) {
    const SegmentSample* mount = ValidMount(snapshot);
    if (mount == nullptr || !std::isfinite(snapshot.frame_rate_hz) ||
        snapshot.frame_rate_hz <= 0.0) {
        Reset();
        return {};
    }

    if (!seeded_) {
        Seed(*mount, snapshot.frame_number);
        return {};
    }

    if (snapshot.frame_number == previous_frame_number_) {
        MountTwistEstimate held = last_estimate_;
        held.updated = false;
        return held;
    }
    if (snapshot.frame_number < previous_frame_number_) {
        Reset();
        return {};
    }

    const std::uint32_t frame_delta =
        snapshot.frame_number - previous_frame_number_;
    const double dt_s =
        static_cast<double>(frame_delta) / snapshot.frame_rate_hz;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        Reset();
        return {};
    }
    if (dt_s > reset_gap_s_) {
        Reset();
        Seed(*mount, snapshot.frame_number);
        return {};
    }

    const Eigen::Matrix3d rotation = mount->orientation.toRotationMatrix();
    const Eigen::Vector3d linear_raw_m_s =
        (mount->position_m - previous_position_m_) / dt_s;
    const Eigen::Vector3d angular_raw_rad_s =
        RotationLog(rotation * previous_rotation_.transpose()) / dt_s;
    if (!linear_raw_m_s.allFinite() || !angular_raw_rad_s.allFinite()) {
        Reset();
        return {};
    }

    const double alpha = filter_tau_s_ == 0.0
                             ? 1.0
                             : 1.0 - std::exp(-dt_s / filter_tau_s_);
    filtered_linear_m_s_ =
        alpha * linear_raw_m_s + (1.0 - alpha) * filtered_linear_m_s_;
    filtered_angular_rad_s_ =
        alpha * angular_raw_rad_s +
        (1.0 - alpha) * filtered_angular_rad_s_;

    Seed(*mount, snapshot.frame_number);
    last_estimate_.linear_m_s = filtered_linear_m_s_;
    last_estimate_.angular_rad_s = filtered_angular_rad_s_;
    last_estimate_.source_frame_number = snapshot.frame_number;
    last_estimate_.valid = true;
    last_estimate_.updated = true;
    return last_estimate_;
}
