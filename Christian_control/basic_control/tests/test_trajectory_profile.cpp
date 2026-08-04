// Hardware-free behavioral tests for the Cartesian segment time scaling.
// No Kortex, Pinocchio, I/O, allocation, or robot connection.

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "TrajectoryProfile.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    void TestSeventhOrderCartesianSegment()
    {
        const Eigen::Vector3d start(0.0, 0.0, 0.0);
        const Eigen::Vector3d finish(0.1, -0.2, 0.2); // exactly 0.3 m
        const CartesianMotionLimits limits{0.05, 0.10, 0.50};
        const auto profile = CartesianSegmentProfile::Create(start, finish, limits);
        Check(profile.has_value(), "finite segment and positive limits create a profile");
        if (!profile)
            return;

        // For 0.3 m the speed bound dominates:
        // T = max(2.1875 L/v, sqrt(7.5131884044 L/a), cbrt(52.5 L/j)).
        Check(std::abs(profile->duration_s() - 13.125) < 1e-12,
              "duration uses the analytical seventh-order derivative maxima");

        const auto at_start = profile->Sample(0.0);
        Check(at_start.has_value(), "zero elapsed time is a valid profile sample");
        if (at_start) {
            Check((at_start->position_m - start).norm() < 1e-15,
                  "profile starts at the requested Cartesian point");
            Check(at_start->velocity_m_s.norm() == 0.0 &&
                      at_start->acceleration_m_s2.norm() == 0.0 &&
                      at_start->jerk_m_s3.norm() == 0.0,
                  "profile starts at rest with zero acceleration and jerk");
            Check(!at_start->complete, "profile is not terminal at its start");
        }

        const auto halfway = profile->Sample(0.5 * profile->duration_s());
        Check(halfway.has_value(), "half-duration sample is valid");
        if (halfway) {
            Check((halfway->position_m - 0.5 * (start + finish)).norm() < 1e-12,
                  "symmetric seventh-order scaling reaches the geometric midpoint");
            const Eigen::Vector3d expected_velocity =
                (finish - start) * (2.1875 / profile->duration_s());
            Check((halfway->velocity_m_s - expected_velocity).norm() < 1e-12,
                  "midpoint velocity uses the exact peak scaling derivative");
            Check(halfway->acceleration_m_s2.norm() < 1e-12,
                  "symmetric profile acceleration crosses zero at its midpoint");
        }

        double peak_speed = 0.0;
        double peak_acceleration = 0.0;
        double peak_jerk = 0.0;
        for (int i = 0; i <= 10000; ++i) {
            const double t = profile->duration_s() * i / 10000.0;
            const auto sample = profile->Sample(t);
            Check(sample.has_value(), "every finite in-range time produces a sample");
            if (!sample)
                continue;
            peak_speed = std::max(peak_speed, sample->velocity_m_s.norm());
            peak_acceleration =
                std::max(peak_acceleration, sample->acceleration_m_s2.norm());
            peak_jerk = std::max(peak_jerk, sample->jerk_m_s3.norm());
        }
        Check(peak_speed <= limits.max_speed_m_s + 1e-12,
              "sampled Cartesian speed stays within its configured bound");
        Check(peak_acceleration <= limits.max_acceleration_m_s2 + 1e-12,
              "sampled Cartesian acceleration stays within its configured bound");
        Check(peak_jerk <= limits.max_jerk_m_s3 + 1e-12,
              "sampled Cartesian jerk stays within its configured bound");

        const auto terminal = profile->Sample(profile->duration_s());
        Check(terminal.has_value(), "terminal time produces a sample");
        if (terminal) {
            Check((terminal->position_m - finish).norm() < 1e-15,
                  "terminal profile sample is the exact requested endpoint");
            Check(terminal->velocity_m_s.norm() == 0.0 &&
                      terminal->acceleration_m_s2.norm() == 0.0 &&
                      terminal->jerk_m_s3.norm() == 0.0,
                  "terminal sample is exactly at rest with zero acceleration and jerk");
            Check(terminal->complete, "terminal sample enables arrival evaluation");
        }
        const auto after = profile->Sample(profile->duration_s() + 1.0);
        Check(after && after->complete && (after->position_m - finish).norm() == 0.0,
              "elapsed time beyond the duration holds the exact endpoint");
    }

    void TestZeroLengthAndInvalidInputs()
    {
        const CartesianMotionLimits limits{0.05, 0.10, 0.50};
        const Eigen::Vector3d point(0.1, -0.2, 0.3);
        const auto stationary =
            CartesianSegmentProfile::Create(point, point, limits);
        Check(stationary && stationary->duration_s() == 0.0,
              "zero-length motion is a valid zero-duration hold");
        const auto held = stationary ? stationary->Sample(0.0) : std::nullopt;
        Check(held && held->complete && (held->position_m - point).norm() == 0.0,
              "zero-length motion is immediately terminal at the requested point");

        Check(!CartesianSegmentProfile::Create(
                   point, point, CartesianMotionLimits{0.0, 0.10, 0.50}),
              "zero speed limit is rejected");
        Check(!CartesianSegmentProfile::Create(
                   point, point, CartesianMotionLimits{0.05, -0.10, 0.50}),
              "negative acceleration limit is rejected");
        Check(!CartesianSegmentProfile::Create(
                   point, point,
                   CartesianMotionLimits{0.05, 0.10,
                       std::numeric_limits<double>::quiet_NaN()}),
              "non-finite jerk limit is rejected");
        Eigen::Vector3d nonfinite = point;
        nonfinite.x() = std::numeric_limits<double>::infinity();
        Check(!CartesianSegmentProfile::Create(point, nonfinite, limits),
              "non-finite Cartesian endpoint is rejected");
        Check(stationary &&
                  !stationary->Sample(std::numeric_limits<double>::quiet_NaN()),
              "non-finite elapsed time is rejected rather than poisoning a reference");
        Check(stationary && !stationary->Sample(-0.001),
              "negative elapsed time is rejected rather than reversing a profile");
    }
} // namespace

int main()
{
    TestSeventhOrderCartesianSegment();
    TestZeroLengthAndInvalidInputs();
    if (failures == 0) {
        std::cout << "all trajectory-profile tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
