//
// ReactiveLaw — the complete mathematical policy for the reactive pose
// controller, ported from msc_project controller/reactive_controller.py.
// Read the functions below in order:
//
//   1. pose error         e_pos, e_rot = log3(R_des · Rᵀ)
//   2. twist error        e_v, e_w = reference twist − J·q̇
//   3. task twist         ẋ = Kp·e_pose + Kd·e_twist
//   4. damped least sq.   q̇_task = Jᵀ(JJᵀ + λ²I₆)⁻¹ ẋ
//   5. null-space objectives: joint-limit avoidance (deadband) plus,
//      when a nominal joint state is supplied, posture guidance
//      q̇_nom + kq·wrap(q_nom − q)
//   6. null-space proj.   q̇_raw = q̇_task + N (q̇_limit + q̇_posture)
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
// - the deadband objective wraps to (−π, π] because Kortex reports
//   positions in [0, 360); MuJoCo's q is continuous, so the sim never
//   needed this.
//

#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "State.h" // Twist (equation 2), JointReference + wrap (equation 5b)

// Gains and term switches. Disabled terms contribute exactly zero, so the
// staged bring-up (P-only, then Kd, then limit avoidance) is configuration.
struct ReactivePoseGains {
    double kp_position_s_inv = 0.0; // 1/s on the position error
    double kp_rotation_s_inv = 0.0; // 1/s on the rotation-log error
    double kd_position = 0.0;       // unitless on the linear-velocity error
    double kd_rotation = 0.0;       // unitless on the angular-velocity error
    double limit_avoid_gain_s_inv = 0.0; // 1/s on the zone excess
    double posture_gain_s_inv = 0.0; // 1/s on the wrapped nominal-posture error
    double dls_lambda = 0.0;        // DLS damping λ (also damps the projector)
    bool position_enabled = true;
    bool orientation_enabled = true;
    bool velocity_enabled = false;
    bool null_space_enabled = false;
    bool posture_enabled = false; // nominal-posture guidance (needs null space)
};

// Startup multiplier for a secondary objective: the task-space law is
// available immediately, only the limit-avoidance gain ramps in, so
// takeover cannot begin with a full projected joint transient.
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

// Equation 2: twist error, reference minus actual — the Kd term's input.
// `measured` is the end-effector twist J·q̇, linear in rows 0-2 and angular
// in rows 3-5, the row order the Jacobian already uses. Mirrors
// reactive_controller.py twist_error.
//
// A zero reference (the default Twist) reduces this to −measured, i.e. pure
// damping toward a standstill; a moving target supplies its own velocity
// here so the law feeds it forward instead of resisting it.
inline Twist TwistError(const Twist& reference,
                        const Eigen::Matrix<double, 6, 1>& measured)
{
    Twist error;
    error.linear_m_s = reference.linear_m_s - measured.head<3>();
    error.angular_rad_s = reference.angular_rad_s - measured.tail<3>();
    return error;
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

// Equation 5a: the deadband joint-limit-avoidance objective. `limit_rad`
// holds each joint's software-limit magnitude (applied symmetrically,
// ±limit); 0 marks an unbounded joint. The objective is EXACTLY zero until
// a bounded joint's wrapped position enters the activation zone
// [limit − zone, limit], then pushes inward with gain · excess. Wrapped to
// (−π, π] because Kortex reports positions in [0, 360). Replaced
// wrap-to-midpoint centering on 2026-08-05: centering fought the task from
// everywhere and the damped projector leaked it into task space (218 mm
// stall equilibrium) — see
// docs/superpowers/specs/2026-08-05-null-space-limit-avoidance-design.md.
inline Eigen::Matrix<double, 7, 1>
LimitAvoidanceObjective(const Eigen::Matrix<double, 7, 1>& q_rad,
                        const Eigen::Matrix<double, 7, 1>& limit_rad,
                        double zone_rad,
                        const ReactivePoseGains& gains)
{
    Eigen::Matrix<double, 7, 1> objective;
    for (int i = 0; i < 7; ++i) {
        objective[i] = 0.0;
        if (limit_rad[i] <= 0.0)
            continue; // unbounded joint
        const double signed_rad = std::remainder(q_rad[i], 2.0 * M_PI);
        const double excess =
            std::abs(signed_rad) - (limit_rad[i] - zone_rad);
        if (excess > 0.0)
            objective[i] = -gains.limit_avoid_gain_s_inv * excess *
                           (signed_rad < 0.0 ? -1.0 : 1.0);
    }
    return objective;
}

// Equation 5b: nominal-posture guidance — the planner's nominal joint state
// entering the null space as a SECONDARY objective,
//
//     q̇_nom + kq · wrap(q_nom − q).
//
// The wrap is the same short-way-round rule as WrappedJointError (State.h):
// Kortex reports positions on [0, 360) while a planner's nominal is signed,
// so an unwrapped error near zero would command a full-turn correction.
// Non-finite guidance degrades to NO guidance (exactly zero) rather than
// letting a NaN reach the command — a bad hint must never be worse than no
// hint. With ~1 redundant DoF against a 6-DoF task this is guidance, never
// tracking; the gain stays far below the task Kp because the DAMPED
// projector leaks (the 2026-08-05 stall equilibrium).
inline Eigen::Matrix<double, 7, 1>
PostureObjective(const Eigen::Matrix<double, 7, 1>& q_rad,
                 const JointReference& nominal,
                 const ReactivePoseGains& gains)
{
    if (!gains.posture_enabled)
        return Eigen::Matrix<double, 7, 1>::Zero();
    if (!nominal.q_rad.allFinite() || !nominal.qdot_rad_s.allFinite())
        return Eigen::Matrix<double, 7, 1>::Zero();
    return nominal.qdot_rad_s +
           gains.posture_gain_s_inv * WrappedJointError(nominal.q_rad, q_rad);
}

// Equation 6: N = I₇ − Jᵀ(JJᵀ + λ²I₆)⁻¹J, the damped projector, applied to
// one summed objective. Projection is linear, so summing before projecting
// equals projecting each term — but the solve projects ONCE so the leak
// telemetry accounts for the whole null command, not one term of it.
inline Eigen::Matrix<double, 7, 1>
ProjectToNullSpace(const Eigen::Matrix<double, 6, 7>& jacobian,
                   const Eigen::Matrix<double, 7, 1>& objective,
                   double lambda)
{
    Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
    jjt.diagonal().array() += lambda * lambda;
    const Eigen::Matrix<double, 7, 7> projector =
        Eigen::Matrix<double, 7, 7>::Identity() -
        jacobian.transpose() * jjt.ldlt().solve(jacobian);
    return projector * objective;
}

// Equations 5a+6 composed for limit avoidance alone — the pre-posture shape
// of the law, kept because tests and tools exercise it directly.
inline Eigen::Matrix<double, 7, 1>
LimitAvoidanceVelocity(const Eigen::Matrix<double, 6, 7>& jacobian,
                       const Eigen::Matrix<double, 7, 1>& q_rad,
                       const Eigen::Matrix<double, 7, 1>& limit_rad,
                       double zone_rad,
                       const ReactivePoseGains& gains)
{
    return ProjectToNullSpace(
        jacobian, LimitAvoidanceObjective(q_rad, limit_rad, zone_rad, gains),
        gains.dls_lambda);
}

// The solved velocity split into its two objectives, plus the leak the
// DAMPED projector lets back into task space. The leak twist J·q̇_null is
// the end-effector velocity the limit-avoidance term causes despite
// projection — zero only for an undamped projector. The 2026-08-05 stall
// parked the arm 218 mm short of its target exactly where Kp·e_pos balanced
// this leak, so the decomposition is first-class telemetry, not a debug
// extra.
struct ReactiveSolution {
    Eigen::Matrix<double, 7, 1> qdot_task_rad_s; // equations 3-4
    Eigen::Matrix<double, 7, 1> qdot_null_rad_s; // equations 5-6 (zero when off)
    Eigen::Matrix<double, 6, 1> leak_twist;      // J · q̇_null [v; ω]
};

// Equations 3-6 composed, decomposition preserved. The total command is
// qdot_task_rad_s + qdot_null_rad_s (SolveReactiveVelocity below).
// `nominal` is the OPTIONAL posture guidance (equation 5b): pass the
// planner's nominal joint state to bias the redundant motion toward it, or
// nullptr for the plain law — every pre-slice caller compiles and computes
// unchanged.
inline ReactiveSolution
SolveReactiveVelocityDetailed(const Eigen::Matrix<double, 6, 7>& jacobian,
                              const Eigen::Vector3d& e_pos, const Eigen::Vector3d& e_rot,
                              const Eigen::Vector3d& e_v, const Eigen::Vector3d& e_w,
                              const Eigen::Matrix<double, 7, 1>& q_rad,
                              const Eigen::Matrix<double, 7, 1>& limit_rad,
                              double zone_rad,
                              const ReactivePoseGains& gains,
                              const JointReference* nominal = nullptr)
{
    const Eigen::Matrix<double, 6, 1> twist =
        TaskTwist(e_pos, e_rot, e_v, e_w, gains);
    ReactiveSolution solution;
    solution.qdot_task_rad_s =
        DampedLeastSquares6(jacobian, twist, gains.dls_lambda);
    if (gains.null_space_enabled) {
        Eigen::Matrix<double, 7, 1> objective =
            LimitAvoidanceObjective(q_rad, limit_rad, zone_rad, gains);
        if (nominal != nullptr)
            objective += PostureObjective(q_rad, *nominal, gains);
        solution.qdot_null_rad_s =
            ProjectToNullSpace(jacobian, objective, gains.dls_lambda);
        solution.leak_twist = jacobian * solution.qdot_null_rad_s;
    } else {
        solution.qdot_null_rad_s.setZero();
        solution.leak_twist.setZero();
    }
    return solution;
}

// Equations 3-6 composed: the requested joint velocity BEFORE any clamping
// (the Runner clamps, the actuation integrates — never this law).
inline Eigen::Matrix<double, 7, 1>
SolveReactiveVelocity(const Eigen::Matrix<double, 6, 7>& jacobian,
                      const Eigen::Vector3d& e_pos, const Eigen::Vector3d& e_rot,
                      const Eigen::Vector3d& e_v, const Eigen::Vector3d& e_w,
                      const Eigen::Matrix<double, 7, 1>& q_rad,
                      const Eigen::Matrix<double, 7, 1>& limit_rad,
                      double zone_rad,
                      const ReactivePoseGains& gains,
                      const JointReference* nominal = nullptr)
{
    const ReactiveSolution solution = SolveReactiveVelocityDetailed(
        jacobian, e_pos, e_rot, e_v, e_w, q_rad, limit_rad, zone_rad, gains,
        nominal);
    return solution.qdot_task_rad_s + solution.qdot_null_rad_s;
}
