//
// The posture-preference term of the reactive law (slice 2 of
// planner/controller redundancy agreement). Header-only law, no robot:
// every case builds a Jacobian whose null space is known by construction
// (J = [I6 | 0] has exactly the joint-7 axis), so the projected result can
// be checked against hand arithmetic.
//
// The term is a PREFERENCE with two hard subordinations:
//   - the compile-time switch and the per-cycle has_posture flag both
//     default off, and either alone silences it exactly;
//   - a joint whose limit-avoidance objective is active contributes no
//     posture objective — safety outranks taste on that joint.
//

#include <cmath>
#include <cstdio>
#include <string>

#include "ReactiveLaw.h"

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++failures;
    }
}

// J = [I6 | 0]: joints 1-6 are the task space, joint 7 is exactly the null
// space. With small lambda the damped projector is near-exact on axis 7.
Eigen::Matrix<double, 6, 7> TaskJacobian() {
    Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Zero();
    jacobian.block<6, 6>(0, 0).setIdentity();
    return jacobian;
}

ReactivePoseGains BaseGains() {
    ReactivePoseGains gains;
    gains.kp_position_s_inv = 1.0;
    gains.kp_rotation_s_inv = 1.0;
    gains.limit_avoid_gain_s_inv = 2.0;
    gains.dls_lambda = 1e-6;
    gains.null_space_enabled = true;
    return gains;
}

constexpr double kDeg = M_PI / 180.0;

}  // namespace

int main() {
    const Eigen::Matrix<double, 6, 7> jacobian = TaskJacobian();
    const Eigen::Vector3d zero3 = Eigen::Vector3d::Zero();
    const Eigen::Matrix<double, 7, 1> no_limits =
        Eigen::Matrix<double, 7, 1>::Zero();
    const double zone_rad = 20.0 * kDeg;

    Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> posture = Eigen::Matrix<double, 7, 1>::Zero();

    // 1. Switch off (default gains): passing a posture changes nothing,
    //    bit for bit.
    {
        ReactivePoseGains gains = BaseGains();
        posture.setConstant(0.5);
        const ReactiveSolution without = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, no_limits, zone_rad,
            gains);
        const ReactiveSolution with = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, no_limits, zone_rad,
            gains, /*has_posture=*/true, posture);
        Check((without.qdot_null_rad_s - with.qdot_null_rad_s).norm() == 0.0,
              "posture term disabled by default: identical null velocity");
    }

    // 2. Enabled but no posture supplied this cycle: still silent.
    {
        ReactivePoseGains gains = BaseGains();
        gains.posture_enabled = true;
        gains.posture_gain_s_inv = 0.5;
        const ReactiveSolution solution = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, no_limits, zone_rad,
            gains, /*has_posture=*/false, posture);
        Check(solution.qdot_null_rad_s.norm() == 0.0,
              "no posture this cycle: null velocity stays zero");
    }

    // 3. Pure posture on the null axis: joint 7 moves toward the preference
    //    at gain x error; task joints stay untouched.
    {
        ReactivePoseGains gains = BaseGains();
        gains.posture_enabled = true;
        gains.posture_gain_s_inv = 0.5;
        q.setZero();
        posture.setZero();
        posture(6) = 0.2;  // rad
        const ReactiveSolution solution = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, no_limits, zone_rad,
            gains, true, posture);
        Check(std::abs(solution.qdot_null_rad_s(6) - 0.5 * 0.2) < 1e-9,
              "null-axis posture error commands gain x error on joint 7");
        Check(solution.qdot_null_rad_s.head<6>().cwiseAbs().maxCoeff() < 1e-9,
              "task-space joints receive no posture velocity");
    }

    // 4. Wrapping: measured 350 deg, preferred 10 deg is a +20 deg error,
    //    never a -340 deg sweep. (Kortex reports [0, 360).)
    {
        ReactivePoseGains gains = BaseGains();
        gains.posture_enabled = true;
        gains.posture_gain_s_inv = 0.5;
        q.setZero();
        q(6) = 350.0 * kDeg;
        posture.setZero();
        posture(6) = 10.0 * kDeg;
        const ReactiveSolution solution = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, no_limits, zone_rad,
            gains, true, posture);
        Check(std::abs(solution.qdot_null_rad_s(6) - 0.5 * 20.0 * kDeg) < 1e-9,
              "posture error wraps to the short way around");
    }

    // 5. Priority: a joint inside its limit-avoidance zone takes only the
    //    avoidance objective; its posture pull is suppressed, and the
    //    posture term keeps acting on the other joints.
    {
        ReactivePoseGains gains = BaseGains();
        gains.posture_enabled = true;
        gains.posture_gain_s_inv = 0.5;
        Eigen::Matrix<double, 7, 1> limits = no_limits;
        limits(6) = 120.0 * kDeg;         // joint 7 bounded for this case
        q.setZero();
        q(6) = 110.0 * kDeg;              // 10 deg inside the 20 deg zone
        posture.setZero();
        posture(6) = 119.0 * kDeg;        // preference pulls OUTWARD
        const ReactiveSolution with_posture = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, limits, zone_rad, gains,
            true, posture);
        ReactivePoseGains avoidance_only = gains;
        avoidance_only.posture_enabled = false;
        const ReactiveSolution avoidance = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, limits, zone_rad,
            avoidance_only, false, posture);
        Check((with_posture.qdot_null_rad_s - avoidance.qdot_null_rad_s)
                      .norm() == 0.0,
              "inside the avoidance zone the posture pull is suppressed");
        Check(avoidance.qdot_null_rad_s(6) < 0.0,
              "sanity: avoidance pushes joint 7 inward");
    }

    // 6. The leak decomposition stays consistent: leak_twist is exactly
    //    J times the null velocity, posture term included.
    {
        ReactivePoseGains gains = BaseGains();
        gains.posture_enabled = true;
        gains.posture_gain_s_inv = 0.5;
        gains.dls_lambda = 0.1;  // damped: projector leaks by design
        q.setZero();
        posture.setConstant(0.3);
        const ReactiveSolution solution = SolveReactiveVelocityDetailed(
            jacobian, zero3, zero3, zero3, zero3, q, no_limits, zone_rad,
            gains, true, posture);
        Check((solution.leak_twist - jacobian * solution.qdot_null_rad_s)
                      .norm() < 1e-12,
              "leak twist equals J times the null velocity");
    }

    if (failures == 0)
        std::printf("test_posture_tracking: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
