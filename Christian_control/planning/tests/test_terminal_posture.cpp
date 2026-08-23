#include <cmath>
#include <cstdio>
#include <string>

#include "PathIk.h"

// Terminal candidates are ranked by posture quality, not by joint-space
// distance alone. The scenario is the 2026-08-23 goal-8 failure shape:
// candidate A achieves the pose with joint 2 at 126.5 deg (0.4 deg from the
// planner limit) and is close to the current configuration; candidate B
// achieves the same pose with joint 2 at 91 deg (35.9 deg margin) but is
// farther away. On a physical robot B is clearly better, and the score must
// say so.

namespace
{

    int failures = 0;

    void Check(bool condition, const std::string& what)
    {
        if (!condition) {
            std::printf("FAIL: %s\n", what.c_str());
            ++failures;
        }
    }

    constexpr double kDeg = M_PI / 180.0;

    PathIkJointLimits PlannerLimits()
    {
        PathIkJointLimits limits;
        const double sentinel = 1e20;
        limits.lower_rad << -sentinel, -126.9 * kDeg, -sentinel, -145.8 * kDeg,
            -sentinel, -118.3 * kDeg, -sentinel;
        limits.upper_rad << sentinel, 126.9 * kDeg, sentinel, 145.8 * kDeg,
            sentinel, 118.3 * kDeg, sentinel;
        return limits;
    }

} // namespace

int main()
{
    const PathIkJointLimits limits = PlannerLimits();

    Eigen::Matrix<double, 7, 1> measured;
    measured << 92.1 * kDeg, 57.3 * kDeg, 12.2 * kDeg, -80.8 * kDeg,
        -94.0 * kDeg, 77.2 * kDeg, 177.9 * kDeg;

    // Candidate A: cramped — joint 2 essentially at its stop, other bounded
    // joints comfortable, close to the measured configuration.
    Eigen::Matrix<double, 7, 1> cramped = measured;
    cramped(1) = 126.5 * kDeg;

    // Candidate B: roomy — joint 2 mid-range, but a larger overall move.
    Eigen::Matrix<double, 7, 1> roomy = measured;
    roomy(1) = 91.0 * kDeg;
    roomy(0) = 40.0 * kDeg;
    roomy(2) = 60.0 * kDeg;

    const double cramped_score = TerminalPostureScore(cramped, measured, limits);
    const double roomy_score = TerminalPostureScore(roomy, measured, limits);
    Check(std::isfinite(cramped_score) && std::isfinite(roomy_score),
          "scores are finite");
    Check(roomy_score < cramped_score,
          "roomy candidate outranks the cramped one (roomy " +
              std::to_string(roomy_score) + " vs cramped " +
              std::to_string(cramped_score) + ")");

    // A candidate identical to the measured configuration mid-range scores
    // better than the same posture displaced by a full revolution of a
    // continuous joint: continuous joints contribute distance, never margin.
    Eigen::Matrix<double, 7, 1> mid = measured;
    mid(1) = 0.0;
    Eigen::Matrix<double, 7, 1> wound = mid;
    wound(0) += 2.0 * M_PI;
    const double mid_score = TerminalPostureScore(mid, measured, limits);
    const double wound_score = TerminalPostureScore(wound, measured, limits);
    Check(std::abs(mid_score - wound_score) < 1e-9,
          "a full revolution of a continuous joint is the same posture "
          "(wrapped displacement)");

    // Candidates whose worst margins sit in the same band are
    // posture-equivalent, and the legacy closest-candidate preference
    // decides — deliberately, so the ranking is behaviour-preserving except
    // where headroom differs materially. Both of these keep joint 2 more
    // than 30 deg from its stops; the nearer one must win.
    Eigen::Matrix<double, 7, 1> near_same_band = measured;
    near_same_band(1) = 70.0 * kDeg;   // margin 56.9 deg, small move
    Eigen::Matrix<double, 7, 1> far_same_band = measured;
    far_same_band(1) = 0.0;            // margin 126.9 deg, larger move
    Check(TerminalPostureScore(near_same_band, measured, limits) <
              TerminalPostureScore(far_same_band, measured, limits),
          "within a margin band the closer candidate keeps winning");

    if (failures == 0)
        std::puts("test_terminal_posture: all assertions passed");
    return failures == 0 ? 0 : 1;
}
