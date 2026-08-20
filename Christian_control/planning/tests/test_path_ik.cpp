#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "PathIk.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

PathSample SampleFor(const Eigen::Matrix<double, 7, 1>& q,
                     const PathIkArm& arm, double t_s) {
    PathSample sample;
    sample.t_s = t_s;
    sample.pose = Eigen::Isometry3d(
        analytical_ik::AnalyticalIKSolver::computeForwardKinematics(
            q, arm.base_transform, arm.end_effector_frame, arm.left_arm));
    return sample;
}

PathSample UnreachableSample(double t_s) {
    PathSample sample;
    sample.t_s = t_s;
    sample.pose.translation() = Eigen::Vector3d(10.0, 10.0, 10.0);
    return sample;
}

double AngleDifference(double a, double b) {
    return std::remainder(a - b, 2.0 * kPi);
}

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
    PathIkArm arm;
    arm.end_effector_frame = config::kRightEndEffectorFrame;

    // Joint 1 is continuous. The valid neighbours intentionally straddle
    // its -pi/+pi seam, where arithmetic averaging would choose the long
    // route through zero.
    Eigen::Matrix<double, 7, 1> before;
    before << 3.10, 0.20, -0.20, 0.30, 0.10, -0.30, 0.20;
    Eigen::Matrix<double, 7, 1> after = before;
    after(0) = -3.10;

    CartesianPath isolated_gap;
    isolated_gap.samples = {
        SampleFor(before, arm, 0.0),
        UnreachableSample(1.0),
        SampleFor(after, arm, 2.0),
    };

    analytical_ik::IKTolerance tolerance;
    tolerance.converge_position_m = 1e-5;
    tolerance.converge_orientation_rad = 1e-5;
    tolerance.accept_position_m = 1e-3;
    tolerance.accept_orientation_rad = 1e-3;

    const PathIkResult recovered =
        SolvePathIk(isolated_gap, arm, before, tolerance, false);
    Require(recovered.samples.size() == 3, "unexpected sample count");
    Require(recovered.success, "isolated unresolved sample was not seedable");
    Require(recovered.samples[0].solved, "first valid neighbour did not solve");
    Require(!recovered.samples[1].solved, "unresolved sample was marked solved");
    Require(recovered.samples[2].solved, "second valid neighbour did not solve");
    Require(recovered.samples[1].alternative_seed_attempts ==
                kPathIkAlternativeSeedCount,
            "alternative seed count was not exactly four");
    Require(recovered.samples[1].seed_interpolated,
            "unresolved sample did not receive an interpolated seed");

    // The unresolved sample is a seed interpolation, not the failed IK
    // near-miss. Bounded joints interpolate linearly; continuous joint 1
    // takes the shortest wrapped arc.
    const Eigen::Matrix<double, 7, 1>& q_before =
        recovered.samples[0].configuration;
    const Eigen::Matrix<double, 7, 1>& q_after =
        recovered.samples[2].configuration;
    const Eigen::Matrix<double, 7, 1>& q_gap =
        recovered.samples[1].configuration;
    const double expected_continuous =
        q_before(0) + 0.5 * std::remainder(q_after(0) - q_before(0), 2.0 * kPi);
    Require(std::abs(AngleDifference(q_gap(0), expected_continuous)) < 1e-6,
            "continuous joint used long-way interpolation");
    for (int joint = 1; joint < 7; ++joint)
        Require(std::abs(q_gap(joint) -
                         0.5 * (q_before(joint) + q_after(joint))) < 1e-6,
                "bounded joint interpolation was not linear");

    // A leading unresolved region has no two-sided continuation seed and is
    // rejected as an initialization failure rather than filled blindly.
    CartesianPath leading_gap;
    leading_gap.samples = {
        UnreachableSample(0.0),
        UnreachableSample(1.0),
        SampleFor(after, arm, 2.0),
    };
    const PathIkResult rejected =
        SolvePathIk(leading_gap, arm, before, tolerance, false);
    Require(!rejected.success,
            "leading unresolved region was incorrectly accepted");

    std::puts("test_path_ik: all assertions passed");
    return 0;
}
