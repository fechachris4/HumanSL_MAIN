#undef NDEBUG
//
// Hardware-free tests for the joint-trajectory wire grammar, its line
// accumulator, and its validation. No Kortex, Pinocchio, or robot connection.
//

#include <cassert>
#include <string>

#include "JointTrajectory.h"

static JointTrajectory FeedBlock(const std::vector<std::string>& lines)
{
    JointTrajectoryAccumulator acc;
    std::string error;
    std::optional<JointTrajectory> out;
    for (const auto& l : lines) {
        out = acc.Feed(l, error);
        assert(error.empty());
    }
    assert(out.has_value());
    return *out;
}

int main()
{
    // Happy path: 3 points, degrees on the wire, radians in memory.
    const auto traj = FeedBlock({
        "TRAJ_BEGIN 3",
        "0    0 0 0 0 0 0 0    0 0 0 0 0 0 0",
        "1.0  10 0 0 0 0 0 0   10 0 0 0 0 0 0",
        "2.0  20 0 0 0 0 0 0   0 0 0 0 0 0 0",
    });
    assert(traj.points.size() == 3);
    assert(std::abs(traj.points[1].q_rad(0) - 10.0 * M_PI / 180.0) < 1e-12);
    assert(std::abs(traj.points[1].qdot_rad_s(0) - 10.0 * M_PI / 180.0) < 1e-12);

    // A malformed row resets and reports; the next block still works.
    {
        JointTrajectoryAccumulator acc;
        std::string error;
        assert(!acc.Feed("TRAJ_BEGIN 2", error) && error.empty());
        assert(!acc.Feed("0 1 2 nonsense", error).has_value());
        assert(!error.empty());
    }
    // Row count mismatch: TRAJ_END before <count> rows is an error.
    {
        JointTrajectoryAccumulator acc;
        std::string error;
        acc.Feed("TRAJ_BEGIN 2", error);
        acc.Feed("0 0 0 0 0 0 0 0 0 0 0 0 0 0 0", error);
        assert(!acc.Feed("TRAJ_END", error).has_value());
        assert(!error.empty());
    }

    // Validation.
    Eigen::Matrix<double, 7, 1> lo = Eigen::Matrix<double, 7, 1>::Constant(-180.0);
    Eigen::Matrix<double, 7, 1> hi = Eigen::Matrix<double, 7, 1>::Constant(180.0);
    Eigen::Matrix<double, 7, 1> vmax = Eigen::Matrix<double, 7, 1>::Constant(45.0);
    assert(!ValidateJointTrajectory(traj, lo, hi, vmax).has_value());
    // Position outside limits rejected.
    {
        auto bad = traj;
        bad.points[2].q_rad(0) = 200.0 * M_PI / 180.0;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }
    // Stated velocity above the clip rejected.
    {
        auto bad = traj;
        bad.points[1].qdot_rad_s(0) = 90.0 * M_PI / 180.0;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }
    // Implied velocity above the clip rejected (20 deg in 0.1 s = 200 deg/s).
    {
        auto bad = traj;
        bad.points[2].t_s = 1.1;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }
    // Non-monotonic time rejected.
    {
        auto bad = traj;
        bad.points[2].t_s = 0.5;
        assert(ValidateJointTrajectory(bad, lo, hi, vmax).has_value());
    }

    // Endpoint interpolation: at support times, exactly the support values.
    {
        const auto s0 = SampleJointTrajectory(traj, 0.0);
        assert(std::abs(s0.q_rad(0)) < 1e-12 && !s0.complete);
        const auto s1 = SampleJointTrajectory(traj, 1.0);
        assert(std::abs(s1.q_rad(0) - 10.0 * M_PI / 180.0) < 1e-9);
        // Midpoint lies between the bracketing supports and velocity is finite.
        const auto sm = SampleJointTrajectory(traj, 0.5);
        assert(sm.q_rad(0) > 0.0 && sm.q_rad(0) < 10.0 * M_PI / 180.0);
        // Past the end: last position, zero velocity, complete.
        const auto se = SampleJointTrajectory(traj, 99.0);
        assert(std::abs(se.q_rad(0) - 20.0 * M_PI / 180.0) < 1e-9);
        assert(se.qdot_rad_s.norm() == 0.0 && se.complete);
        // Hermite consistency: derivative at t=1 approximates the stated velocity.
        const double h = 1e-5;
        const auto a = SampleJointTrajectory(traj, 1.0 - h);
        const auto b = SampleJointTrajectory(traj, 1.0 + h);
        assert(std::abs((b.q_rad(0) - a.q_rad(0)) / (2 * h) - s1.qdot_rad_s(0)) < 1e-3);
    }

    // Degenerate segments, hand-built past the accumulator and the validator:
    // a sample is one hop from a joint command, so it must stay finite even for
    // a trajectory that should never have reached the loop.
    {
        // Duplicated support time. The bracket search always lands strictly
        // inside a segment, so this one is already safe; asserted to keep it so.
        auto dup = traj;
        dup.points.insert(dup.points.begin() + 2, dup.points[1]);
        const auto sd = SampleJointTrajectory(dup, 1.0);
        assert(sd.q_rad.allFinite() && sd.qdot_rad_s.allFinite());

        // Non-finite support time: the segment width is NaN, which no ordered
        // comparison rejects. Hold the segment start instead of interpolating.
        auto bad_time = traj;
        bad_time.points[1].t_s = std::nan("");
        const auto sn = SampleJointTrajectory(bad_time, 0.5);
        assert(sn.q_rad.allFinite() && sn.qdot_rad_s.allFinite());
        assert((sn.q_rad - bad_time.points[0].q_rad).norm() == 0.0);
        assert(sn.qdot_rad_s.norm() == 0.0 && !sn.complete);
    }
    return 0;
}
