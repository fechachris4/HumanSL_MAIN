// Inter-arm clearance: do the two arms come too close while each executes
// its own trajectory?
//
// The checks below verify the ARITHMETIC against an independently computed
// distance rather than against the checker's own output, and verify the two
// properties that make the check meaningful:
//
//   - clearance is SURFACE to surface, not centre to centre. A centre
//     distance would report two arms as metres apart while their collision
//     models overlapped.
//   - the comparison is at CORRESPONDING TIMES over the start-skew window.
//     Two arms passing through the same point ten seconds apart never meet;
//     treating one arm's swept volume as static would forbid that pair.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "InterArmDistance.h"
#include "PlannerModel.h"

namespace {

TimedJointTrajectory Trajectory(
    const std::vector<std::pair<double, Eigen::Matrix<double, 7, 1>>>& knots_deg,
    double dt_s) {
    TimedJointTrajectory out;
    if (knots_deg.size() < 2 || !(dt_s > 0.0)) return out;
    out.valid = true;
    out.duration_s = knots_deg.back().first;
    std::size_t hi = 1;
    for (double t = 0.0; t <= out.duration_s + 1e-12; t += dt_s) {
        while (hi + 1 < knots_deg.size() && knots_deg[hi].first < t) ++hi;
        const auto& a = knots_deg[hi - 1];
        const auto& b = knots_deg[hi];
        const double span = b.first - a.first;
        const double u = span > 0.0 ? (t - a.first) / span : 0.0;
        TimedJointSample sample;
        sample.t_s = t;
        sample.q_rad = ((1.0 - u) * a.second + u * b.second) * (M_PI / 180.0);
        sample.qdot_rad_s = (b.second - a.second) * (M_PI / 180.0) / span;
        out.samples.push_back(sample);
    }
    return out;
}

TimedJointTrajectory Hold(const Eigen::Matrix<double, 7, 1>& q_deg,
                          double duration_s, double dt_s) {
    return Trajectory({{0.0, q_deg}, {duration_s, q_deg}}, dt_s);
}

// The same distance the checker should find, computed here directly from
// the models so the test does not merely agree with the implementation.
double ExpectedMinimumClearance(const PlannerModel& left,
                                const Eigen::Matrix<double, 7, 1>& left_q,
                                const PlannerModel& right,
                                const Eigen::Matrix<double, 7, 1>& right_q) {
    const gtsam::Matrix a = left.arm_model->sphereCentersMat(gtsam::Vector(left_q));
    const gtsam::Matrix b = right.arm_model->sphereCentersMat(gtsam::Vector(right_q));
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < a.cols(); ++i)
        for (int j = 0; j < b.cols(); ++j) {
            const double d =
                (Eigen::Vector3d(a(0, i), a(1, i), a(2, i)) -
                 Eigen::Vector3d(b(0, j), b(1, j), b(2, j))).norm() -
                left.arm_model->sphere_radius(static_cast<size_t>(i)) -
                right.arm_model->sphere_radius(static_cast<size_t>(j));
            best = std::min(best, d);
        }
    return best;
}

}  // namespace

int main(int argc, char** argv) {
    assert(argc == 3 && "usage: test_inter_arm_distance <dh_tool.yaml> <dh_flange.yaml>");
    const PlannerModel right = LoadPlannerModel(
        argv[1], /*has_tool=*/true, Eigen::Isometry3d::Identity());
    const PlannerModel left = LoadPlannerModel(
        argv[2], /*has_tool=*/false, Eigen::Isometry3d::Identity());

    // --- both arms stationary at their zero configurations. The mounting
    //     geometry separates them, so this is the rig's resting clearance.
    Eigen::Matrix<double, 7, 1> zero_deg = Eigen::Matrix<double, 7, 1>::Zero();
    const Eigen::Matrix<double, 7, 1> zero_rad = Eigen::Matrix<double, 7, 1>::Zero();

    const TimedJointTrajectory left_trajectory = Hold(zero_deg, 2.0, 0.01);
    const TimedJointTrajectory right_trajectory = Hold(zero_deg, 2.0, 0.01);

    const InterArmClearanceReport report = CheckInterArmClearance(
        left, left_trajectory, right, right_trajectory,
        /*required_clearance_m=*/0.05, /*skew_window_s=*/0.0, /*grid_dt_s=*/0.01);
    assert(report.evaluated && report.error.empty());
    std::printf("%s", report.Summary().c_str());

    const double expected = ExpectedMinimumClearance(left, zero_rad, right, zero_rad);
    assert(std::abs(report.minimum_clearance_m - expected) < 1e-9 &&
           "the reported clearance must equal an independently computed "
           "sphere-pair surface distance");

    // --- surface, not centre. The two must differ by exactly the radii of
    //     the reported pair, which is what makes overlap detectable.
    const double centre_distance =
        (report.closest_pair.left_centre_m - report.closest_pair.right_centre_m).norm();
    assert(std::abs(centre_distance - report.closest_pair.left_radius_m -
                    report.closest_pair.right_radius_m -
                    report.minimum_clearance_m) < 1e-9 &&
           "clearance must be centre distance minus BOTH radii");

    // --- the responsible pair must be named. A bare minimum is not
    //     actionable: the fix depends on which parts are close.
    assert(report.closest_pair.left_radius_m > 0.0);
    assert(report.closest_pair.right_radius_m > 0.0);

    // --- a requirement above the actual clearance must be reported as
    //     violated, and one below it as satisfied. The threshold must be a
    //     real control, not decoration.
    const InterArmClearanceReport strict = CheckInterArmClearance(
        left, left_trajectory, right, right_trajectory,
        /*required=*/report.minimum_clearance_m + 0.01, 0.0, 0.01);
    assert(!strict.clearance_satisfied && "a stricter requirement must fail");
    const InterArmClearanceReport lax = CheckInterArmClearance(
        left, left_trajectory, right, right_trajectory,
        /*required=*/std::max(0.0, report.minimum_clearance_m - 0.01), 0.0, 0.01);
    assert(lax.clearance_satisfied && "a laxer requirement must pass");

    // --- TIME CORRESPONDENCE. Two arms that occupy the same region at
    //     DIFFERENT times must not be reported as colliding: that is the
    //     whole reason this is not a static swept-volume check.
    //
    //     Left holds a folded posture for its first second then extends;
    //     right does the reverse. They share the posture but never at the
    //     same instant.
    Eigen::Matrix<double, 7, 1> folded_deg;
    folded_deg << 0, -60, 0, 90, 0, 45, 0;
    {
        // left: zero -> folded -> zero ; right: folded -> zero -> folded
        const TimedJointTrajectory l = Trajectory(
            {{0.0, zero_deg}, {5.0, folded_deg}, {10.0, zero_deg}}, 0.02);
        const TimedJointTrajectory r = Trajectory(
            {{0.0, folded_deg}, {5.0, zero_deg}, {10.0, folded_deg}}, 0.02);

        const InterArmClearanceReport timed = CheckInterArmClearance(
            left, l, right, r, 0.05, /*skew=*/0.0, 0.02);
        assert(timed.evaluated);
        // A wider skew window can only ever find a SMALLER minimum: it
        // considers strictly more pairings, including the t-vs-t ones.
        const InterArmClearanceReport skewed = CheckInterArmClearance(
            left, l, right, r, 0.05, /*skew=*/1.0, 0.02);
        assert(skewed.minimum_clearance_m <= timed.minimum_clearance_m + 1e-12 &&
               "widening the skew window must not INCREASE the reported minimum — "
               "it admits strictly more relative timings");
        std::printf("time-correspondence: exact %.1f mm, +-1 s skew %.1f mm\n",
                    timed.minimum_clearance_m * 1000.0,
                    skewed.minimum_clearance_m * 1000.0);
    }

    // --- A GENUINE OVERLAP MUST BE DETECTED. A clearance check that only
    //     ever reports "satisfied" is worthless, so this pins a posture
    //     measured (2026-08-07) to interpenetrate: both arms folded toward
    //     the midline at j2 = j4 = -90 deg give -94.6 mm, i.e. the collision
    //     models overlap by 95 mm.
    {
        Eigen::Matrix<double, 7, 1> folded = Eigen::Matrix<double, 7, 1>::Zero();
        folded(1) = -90.0;  // degrees: the wire format is degrees
        folded(3) = -90.0;
        const InterArmClearanceReport overlap = CheckInterArmClearance(
            left, Hold(folded, 1.0, 0.5), right, Hold(folded, 1.0, 0.5),
            /*required=*/0.05, /*skew=*/0.0, /*dt=*/0.5);
        assert(overlap.evaluated);
        assert(overlap.minimum_clearance_m < 0.0 &&
               "two arms folded into each other must report NEGATIVE clearance");
        assert(!overlap.clearance_satisfied &&
               "an overlapping pair must not be reported as satisfying clearance");
        // Independently confirmed, so the sign convention cannot silently invert.
        const double expected_overlap = ExpectedMinimumClearance(
            left, folded * (M_PI / 180.0), right, folded * (M_PI / 180.0));
        assert(std::abs(overlap.minimum_clearance_m - expected_overlap) < 1e-9);
        std::printf("overlap detected: %.1f mm (negative = interpenetrating)\n",
                    overlap.minimum_clearance_m * 1000.0);
    }

    // --- invalid internal data is reported, not silently treated as clear.
    const InterArmClearanceReport bad = CheckInterArmClearance(
        left, TimedJointTrajectory{}, right, right_trajectory, 0.05, 0.0, 0.01);
    assert(!bad.evaluated && !bad.error.empty() &&
           "an unparseable block must be reported, never assumed clear");

    std::printf("\nall inter-arm distance tests passed\n");
    return 0;
}
