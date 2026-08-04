#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <Eigen/Core>

// Cartesian quantities are expressed in metres and seconds.  This profile is
// deliberately pure: it owns only fixed-size values and performs no I/O.
struct CartesianMotionLimits {
    double max_speed_m_s;
    double max_acceleration_m_s2;
    double max_jerk_m_s3;
};

struct CartesianProfileSample {
    Eigen::Vector3d position_m;
    Eigen::Vector3d velocity_m_s;
    Eigen::Vector3d acceleration_m_s2;
    Eigen::Vector3d jerk_m_s3;
    bool complete;
};

class CartesianSegmentProfile {
public:
    static std::optional<CartesianSegmentProfile> Create(
        const Eigen::Vector3d& start_m,
        const Eigen::Vector3d& finish_m,
        const CartesianMotionLimits& limits)
    {
        if (!start_m.allFinite() || !finish_m.allFinite() ||
            !std::isfinite(limits.max_speed_m_s) ||
            !std::isfinite(limits.max_acceleration_m_s2) ||
            !std::isfinite(limits.max_jerk_m_s3) ||
            limits.max_speed_m_s <= 0.0 ||
            limits.max_acceleration_m_s2 <= 0.0 ||
            limits.max_jerk_m_s3 <= 0.0) {
            return std::nullopt;
        }

        const Eigen::Vector3d delta_m = finish_m - start_m;
        const double length_m = delta_m.norm();
        if (!std::isfinite(length_m))
            return std::nullopt;

        if (length_m == 0.0)
            return CartesianSegmentProfile(start_m, finish_m, 0.0);

        constexpr double kPeakSpeed = 2.1875;
        constexpr double kPeakAcceleration = 7.51318840439933;
        constexpr double kPeakJerk = 52.5;
        const double duration_s = std::max(
            {kPeakSpeed * length_m / limits.max_speed_m_s,
             std::sqrt(kPeakAcceleration * length_m /
                       limits.max_acceleration_m_s2),
             std::cbrt(kPeakJerk * length_m / limits.max_jerk_m_s3)});
        if (!std::isfinite(duration_s) || duration_s <= 0.0)
            return std::nullopt;

        return CartesianSegmentProfile(start_m, finish_m, duration_s);
    }

    double duration_s() const { return duration_s_; }

    std::optional<CartesianProfileSample> Sample(double elapsed_s) const
    {
        if (!std::isfinite(elapsed_s) || elapsed_s < 0.0)
            return std::nullopt;

        if (duration_s_ == 0.0 || elapsed_s >= duration_s_)
            return TerminalSample();

        const double u = elapsed_s / duration_s_;
        const double u2 = u * u;
        const double u3 = u2 * u;
        const double u4 = u3 * u;
        const double u5 = u4 * u;
        const double u6 = u5 * u;

        const double s = 35.0 * u4 - 84.0 * u5 + 70.0 * u6 - 20.0 * u6 * u;
        const double ds_du =
            140.0 * u3 - 420.0 * u4 + 420.0 * u5 - 140.0 * u6;
        const double d2s_du2 =
            420.0 * u2 - 1680.0 * u3 + 2100.0 * u4 - 840.0 * u5;
        const double d3s_du3 =
            840.0 * u - 5040.0 * u2 + 8400.0 * u3 - 4200.0 * u4;

        const Eigen::Vector3d delta_m = finish_m_ - start_m_;
        return CartesianProfileSample{
            start_m_ + s * delta_m,
            (ds_du / duration_s_) * delta_m,
            (d2s_du2 / (duration_s_ * duration_s_)) * delta_m,
            (d3s_du3 / (duration_s_ * duration_s_ * duration_s_)) * delta_m,
            false};
    }

private:
    CartesianSegmentProfile(const Eigen::Vector3d& start_m,
                            const Eigen::Vector3d& finish_m,
                            double duration_s)
        : start_m_(start_m), finish_m_(finish_m), duration_s_(duration_s)
    {
    }

    CartesianProfileSample TerminalSample() const
    {
        return CartesianProfileSample{finish_m_, Eigen::Vector3d::Zero(),
                                      Eigen::Vector3d::Zero(),
                                      Eigen::Vector3d::Zero(), true};
    }

    Eigen::Vector3d start_m_;
    Eigen::Vector3d finish_m_;
    double duration_s_;
};
