#include "MountTwistEstimator.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

constexpr double kTolerance = 1e-10;

bool Near(double actual, double expected, double tolerance = kTolerance) {
    return std::abs(actual - expected) <= tolerance;
}

ViconSnapshot Snapshot(unsigned int frame_number, double frame_rate_hz,
                       const Eigen::Vector3d& position_m,
                       const Eigen::Quaterniond& orientation,
                       bool valid = true,
                       const char* segment_name = "Mount") {
    ViconSnapshot snapshot;
    snapshot.frame_number = frame_number;
    snapshot.frame_rate_hz = frame_rate_hz;
    SegmentSample mount;
    mount.segment_name = segment_name;
    mount.position_m = position_m;
    mount.orientation = orientation;
    mount.valid = valid;
    snapshot.segments.push_back(mount);
    return snapshot;
}

Eigen::Quaterniond Yaw(double radians) {
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()));
}

} // namespace

int main() {
    // First valid frame seeds history; the first advancing pair establishes
    // translation and principal SO(3)-log angular velocity in world axes.
    {
        MountTwistEstimator estimator(/*filter_tau_s=*/0.0,
                                      /*reset_gap_s=*/0.20);
        const MountTwistEstimate first = estimator.Update(
            Snapshot(100, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0)));
        assert(!first.valid);
        assert(!first.updated);

        const MountTwistEstimate moved = estimator.Update(
            Snapshot(101, 100.0, Eigen::Vector3d(0.01, 0.0, 0.0),
                     Yaw(0.01)));
        assert(moved.valid);
        assert(moved.updated);
        assert(moved.source_frame_number == 101);
        assert(Near(moved.linear_m_s.x(), 1.0));
        assert(Near(moved.linear_m_s.y(), 0.0));
        assert(Near(moved.linear_m_s.z(), 0.0));
        assert(Near(moved.angular_rad_s.x(), 0.0));
        assert(Near(moved.angular_rad_s.y(), 0.0));
        assert(Near(moved.angular_rad_s.z(), 1.0));
    }

    // The exact discretisation alpha=1-exp(-dt/tau) is applied causally.
    // tau=dt/log(2) makes alpha exactly 0.5: equal raw 1 m/s samples filter
    // from zero to 0.5, then 0.75 m/s.
    {
        const double tau_s = 0.01 / std::log(2.0);
        MountTwistEstimator estimator(tau_s, 0.20);
        estimator.Update(
            Snapshot(10, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0)));
        const auto half = estimator.Update(
            Snapshot(11, 100.0, Eigen::Vector3d(0.01, 0.0, 0.0), Yaw(0.0)));
        const auto three_quarters = estimator.Update(
            Snapshot(12, 100.0, Eigen::Vector3d(0.02, 0.0, 0.0), Yaw(0.0)));
        assert(half.valid && half.updated);
        assert(three_quarters.valid && three_quarters.updated);
        assert(Near(half.linear_m_s.x(), 0.5));
        assert(Near(three_quarters.linear_m_s.x(), 0.75));

        // ClientPull may return the same frame again. It is a ZOH read, not a
        // new derivative sample, even if its payload differs unexpectedly.
        const auto repeated = estimator.Update(
            Snapshot(12, 100.0, Eigen::Vector3d(99.0, 0.0, 0.0), Yaw(2.0)));
        assert(repeated.valid);
        assert(!repeated.updated);
        assert(Near(repeated.linear_m_s.x(), 0.75));
        assert(repeated.source_frame_number == 12);
    }

    // Quaternion hemisphere has no physical meaning: q and -q produce the
    // same rotation matrix and therefore zero angular velocity.
    {
        MountTwistEstimator estimator(0.0, 0.20);
        const Eigen::Quaterniond q = Yaw(0.4);
        Eigen::Quaterniond minus_q;
        minus_q.coeffs() = -q.coeffs();
        estimator.Update(Snapshot(20, 100.0, Eigen::Vector3d::Zero(), q));
        const auto estimate = estimator.Update(
            Snapshot(21, 100.0, Eigen::Vector3d::Zero(), minus_q));
        assert(estimate.valid);
        assert(estimate.angular_rad_s.norm() <= kTolerance);
    }

    // Every discontinuity resets history. The next valid frame seeds again;
    // no derivative is ever taken across occlusion or bad timing.
    {
        MountTwistEstimator estimator(0.0, 0.20);
        estimator.Update(
            Snapshot(30, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0)));
        assert(!estimator.Update(
                    Snapshot(31, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0),
                             /*valid=*/false))
                    .valid);
        assert(!estimator.Update(
                    Snapshot(32, 100.0, Eigen::Vector3d(5.0, 0.0, 0.0),
                             Yaw(0.0)))
                    .valid);
        assert(estimator.Update(
                   Snapshot(33, 100.0, Eigen::Vector3d(5.01, 0.0, 0.0),
                            Yaw(0.0)))
                   .valid);

        assert(!estimator.Update(
                    Snapshot(34, 0.0, Eigen::Vector3d::Zero(), Yaw(0.0)))
                    .valid);
        assert(!estimator.Update(
                    Snapshot(35, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0)))
                    .valid);

        const double nan = std::numeric_limits<double>::quiet_NaN();
        assert(!estimator.Update(
                    Snapshot(36, 100.0, Eigen::Vector3d(nan, 0.0, 0.0),
                             Yaw(0.0)))
                    .valid);
        assert(!estimator.Update(
                    Snapshot(37, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0),
                             /*valid=*/true, /*segment_name=*/"RightBase"))
                    .valid);
    }

    // Backwards frames and gaps beyond reset_gap_s cannot bridge history.
    {
        MountTwistEstimator estimator(0.0, 0.20);
        estimator.Update(
            Snapshot(50, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0)));
        assert(!estimator.Update(
                    Snapshot(49, 100.0, Eigen::Vector3d::Ones(), Yaw(0.1)))
                    .valid);
        assert(!estimator.Update(
                    Snapshot(51, 100.0, Eigen::Vector3d::Zero(), Yaw(0.0)))
                    .valid);
        assert(!estimator.Update(
                    Snapshot(80, 100.0, Eigen::Vector3d::Ones(), Yaw(0.2)))
                    .valid);
        assert(!estimator.Update(
                    Snapshot(81, std::numeric_limits<double>::infinity(),
                             Eigen::Vector3d::Ones(), Yaw(0.2)))
                    .valid);
    }

    // Constructor configuration is rejected rather than creating unstable
    // filter coefficients or a reset policy with no positive interval.
    {
        bool threw = false;
        try {
            MountTwistEstimator bad(-0.01, 0.20);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        assert(threw);
        threw = false;
        try {
            MountTwistEstimator bad(0.01, 0.0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        assert(threw);
    }

    return 0;
}
