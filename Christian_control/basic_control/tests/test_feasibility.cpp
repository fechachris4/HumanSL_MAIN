//
// Hardware-free tests for the graded feasibility supervision
// (src/Feasibility.h — pure Eigen). No Kortex, no Pinocchio.
//

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "Feasibility.h"

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

    const double kDeg = M_PI / 180.0;
    const double kNaN = std::numeric_limits<double>::quiet_NaN();

    void TestJointLimitMargin()
    {
        Eigen::Matrix<double, 7, 1> limit;
        limit << 0, 126.9 * kDeg, 0, 145.0 * kDeg, 0, 118.0 * kDeg, 0;

        // All bounded joints at zero: the tightest limit is the margin.
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
        Check(std::abs(JointLimitMarginRad(q, limit) - 118.0 * kDeg) < 1e-12,
              "margin at zero is the tightest limit");

        // j4 10 deg from its limit is the worst joint.
        q[3] = 135.0 * kDeg;
        Check(std::abs(JointLimitMarginRad(q, limit) - 10.0 * kDeg) < 1e-12,
              "margin tracks the worst bounded joint");

        // Kortex wrap: 350 deg = signed -10, so j2's margin is
        // 126.9 - 10 = 116.9 (not negative, not a full turn off).
        q.setZero();
        q[1] = 350.0 * kDeg;
        Check(std::abs(JointLimitMarginRad(q, limit) - 116.9 * kDeg) < 1e-9,
              "margin wraps positions to the nearest turn");

        // Past the limit the margin goes negative — it grades, not clips.
        q.setZero();
        q[5] = 125.0 * kDeg;
        Check(std::abs(JointLimitMarginRad(q, limit) - (-7.0 * kDeg)) < 1e-12,
              "margin keeps grading past the limit");

        // No bounded joints: +infinity, never advises.
        Check(std::isinf(JointLimitMarginRad(
                  q, Eigen::Matrix<double, 7, 1>::Zero())),
              "no bounded joints -> infinite margin");
    }

    void TestDegradationVotes()
    {
        FeasibilityThresholds t;
        t.sigma_min = 0.02;
        t.joint_margin_rad = 5.0 * kDeg;
        t.posture_error_rad = 15.0 * kDeg;
        t.position_error_m = 0.05;

        Check(!FeasibilityDegraded(0.1, 20.0 * kDeg, 5.0 * kDeg, 0.01, t),
              "healthy measures do not vote degraded");
        Check(FeasibilityDegraded(0.01, 20.0 * kDeg, 5.0 * kDeg, 0.01, t),
              "sigma below threshold votes degraded");
        Check(FeasibilityDegraded(0.1, 3.0 * kDeg, 5.0 * kDeg, 0.01, t),
              "limit margin below threshold votes degraded");
        Check(FeasibilityDegraded(0.1, 20.0 * kDeg, 20.0 * kDeg, 0.01, t),
              "posture deviation above threshold votes degraded");
        Check(FeasibilityDegraded(0.1, 20.0 * kDeg, 5.0 * kDeg, 0.10, t),
              "tracking error above threshold votes degraded");

        // NaN abstains — a cycle that computed nothing advises nothing.
        Check(!FeasibilityDegraded(kNaN, kNaN, kNaN, kNaN, t),
              "all-NaN measures abstain");
        // But finite bad measures still vote next to NaN ones.
        Check(FeasibilityDegraded(kNaN, 3.0 * kDeg, kNaN, kNaN, t),
              "a finite bad measure votes even among NaN abstentions");
    }

    void TestAdvisoryDebounce()
    {
        ReplanAdvisor advisor(3);
        Check(!advisor.Update(true), "one degraded cycle does not advise");
        Check(!advisor.Update(true), "two degraded cycles do not advise");
        Check(advisor.Update(true), "the third consecutive cycle advises");
        Check(advisor.Update(true), "the advisory holds while degradation holds");
        Check(!advisor.Update(false), "one healthy cycle clears the advisory");
        Check(!advisor.Update(true), "the count restarts after clearing");

        // A zero/negative cycle count disables the advisory entirely.
        ReplanAdvisor disabled(0);
        Check(!disabled.Update(true) && !disabled.Update(true),
              "a non-positive debounce disables the advisory");
    }

} // namespace

int main()
{
    TestJointLimitMargin();
    TestDegradationVotes();
    TestAdvisoryDebounce();
    if (failures == 0) {
        std::cout << "all feasibility tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
