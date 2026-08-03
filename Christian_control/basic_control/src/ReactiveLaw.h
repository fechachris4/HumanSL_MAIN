//
// ReactiveLaw — the complete mathematical policy for the reactive pose
// controller, ported from msc_project controller/reactive_controller.py.
// Read the functions below in order:
//
//   1. pose error         e_pos, e_rot = log3(R_des · Rᵀ)
//   2. twist error        e_v, e_w (zero reference twist)
//   3. task twist         ẋ = Kp·e_pose + Kd·e_twist
//   4. damped least sq.   q̇_task = Jᵀ(JJᵀ + λ²I₆)⁻¹ ẋ
//   5. joint centering    q̇_null = −k_null · wrap(q − q_mid)
//   6. null-space proj.   q̇_raw = q̇_task + N q̇_null
//
// Header-only, Eigen-only, fixed-size, no allocation — no robot, no
// Pinocchio, so tests/test_reactive_law.cpp cross-validates it against the
// Python law on identical inputs. Changing the equations belongs here;
// changing gains belongs in Config.h.
//
// Deliberate deviations from the simulation law, for hardware:
// - the null-space projector N = I₇ − Jᵀ(JJᵀ + λ²I₆)⁻¹J is DAMPED (the sim
//   uses an undamped pseudoinverse, ill-conditioned exactly where the DLS
//   is protecting the task solution);
// - the centering error wraps to (−π, π] because Kortex reports positions
//   in [0, 360); MuJoCo's q is continuous, so the sim never needed this.
//

#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Gains and term switches. Disabled terms contribute exactly zero, so the
// staged bring-up (P-only, then Kd, then centering) is configuration.
struct ReactivePoseGains {
    double kp_position_s_inv = 0.0; // 1/s on the position error
    double kp_rotation_s_inv = 0.0; // 1/s on the rotation-log error
    double kd_position = 0.0;       // unitless on the linear-velocity error
    double kd_rotation = 0.0;       // unitless on the angular-velocity error
    double null_gain_s_inv = 0.0;   // 1/s on the joint-centering error
    double dls_lambda = 0.0;        // DLS damping λ (also damps the projector)
    bool position_enabled = true;
    bool orientation_enabled = true;
    bool velocity_enabled = false;
    bool null_space_enabled = false;
};

// Startup multiplier for a secondary objective: the task-space law is
// available immediately, only centering ramps in, so takeover cannot begin
// with a full projected joint transient.
inline double UnitRamp(double elapsed_s, double duration_s)
{
    if (duration_s <= 0.0)
        return 1.0;
    return std::clamp(elapsed_s / duration_s, 0.0, 1.0);
}

// Equation 1 (rotation part): the SO(3) logarithm as angle · axis —
// Pinocchio's log3, but pure Eigen. AngleAxis returns angle ∈ [0, π], so
// the result never wraps the long way.
inline Eigen::Vector3d RotationLog(const Eigen::Matrix3d& rotation)
{
    const Eigen::AngleAxisd angle_axis(rotation);
    return angle_axis.angle() * angle_axis.axis();
}

// Equation 3: desired task twist [v; ω]. Row order matches the Jacobian:
// linear first, angular second.
inline Eigen::Matrix<double, 6, 1>
TaskTwist(const Eigen::Vector3d& e_pos, const Eigen::Vector3d& e_rot,
          const Eigen::Vector3d& e_v, const Eigen::Vector3d& e_w,
          const ReactivePoseGains& gains)
{
    Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
    if (gains.position_enabled)
        twist.head<3>() = gains.kp_position_s_inv * e_pos;
    if (gains.orientation_enabled)
        twist.tail<3>() = gains.kp_rotation_s_inv * e_rot;
    if (gains.velocity_enabled) {
        twist.head<3>() += gains.kd_position * e_v;
        twist.tail<3>() += gains.kd_rotation * e_w;
    }
    return twist;
}

// Equation 4: damped least squares for the 6-DoF task. Build the small 6×6
// system, add λ² on the diagonal, solve with LDLT — no explicit inverse, no
// allocation. λ trades tracking accuracy near singularities for bounded
// joint velocities: at a singular pose the solution stays finite.
inline Eigen::Matrix<double, 7, 1>
DampedLeastSquares6(const Eigen::Matrix<double, 6, 7>& jacobian,
                    const Eigen::Matrix<double, 6, 1>& twist, double lambda)
{
    Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
    jjt.diagonal().array() += lambda * lambda;
    return jacobian.transpose() * jjt.ldlt().solve(twist);
}

// The translational (3×7) counterpart, same construction. Used by the
// direction-probing tool; the control path uses the 6-DoF version above.
inline Eigen::Matrix<double, 7, 1>
DampedLeastSquares(const Eigen::Matrix<double, 3, 7>& jacobian_position,
                   const Eigen::Vector3d& v_desired, double lambda)
{
    Eigen::Matrix3d jjt = jacobian_position * jacobian_position.transpose();
    jjt.diagonal().array() += lambda * lambda;
    return jacobian_position.transpose() * jjt.ldlt().solve(v_desired);
}

// Equations 5-6: joint centering projected into the Jacobian null space.
// `centering_mask` is 1 for joints that center, 0 for joints that must not
// (the Gen3's continuous joints 1/3/5/7 have no meaningful midpoint).
inline Eigen::Matrix<double, 7, 1>
NullSpaceVelocity(const Eigen::Matrix<double, 6, 7>& jacobian,
                  const Eigen::Matrix<double, 7, 1>& q_rad,
                  const Eigen::Matrix<double, 7, 1>& midpoint_rad,
                  const Eigen::Matrix<double, 7, 1>& centering_mask,
                  const ReactivePoseGains& gains)
{
    // Equation 5: the centering objective (wrapped — see the file header).
    Eigen::Matrix<double, 7, 1> objective;
    for (int i = 0; i < 7; ++i)
        objective[i] = -gains.null_gain_s_inv * centering_mask[i] *
                       std::remainder(q_rad[i] - midpoint_rad[i], 2.0 * M_PI);

    // Equation 6: N = I₇ − Jᵀ(JJᵀ + λ²I₆)⁻¹J, the damped projector.
    Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
    jjt.diagonal().array() += gains.dls_lambda * gains.dls_lambda;
    const Eigen::Matrix<double, 7, 7> projector =
        Eigen::Matrix<double, 7, 7>::Identity() -
        jacobian.transpose() * jjt.ldlt().solve(jacobian);
    return projector * objective;
}

// Equations 3-6 composed: the requested joint velocity BEFORE any clamping
// (the Runner clamps, the actuation integrates — never this law).
inline Eigen::Matrix<double, 7, 1>
SolveReactiveVelocity(const Eigen::Matrix<double, 6, 7>& jacobian,
                      const Eigen::Vector3d& e_pos, const Eigen::Vector3d& e_rot,
                      const Eigen::Vector3d& e_v, const Eigen::Vector3d& e_w,
                      const Eigen::Matrix<double, 7, 1>& q_rad,
                      const Eigen::Matrix<double, 7, 1>& midpoint_rad,
                      const Eigen::Matrix<double, 7, 1>& centering_mask,
                      const ReactivePoseGains& gains)
{
    const Eigen::Matrix<double, 6, 1> twist =
        TaskTwist(e_pos, e_rot, e_v, e_w, gains);
    Eigen::Matrix<double, 7, 1> qdot =
        DampedLeastSquares6(jacobian, twist, gains.dls_lambda);
    if (gains.null_space_enabled)
        qdot += NullSpaceVelocity(jacobian, q_rad, midpoint_rad,
                                  centering_mask, gains);
    return qdot;
}
