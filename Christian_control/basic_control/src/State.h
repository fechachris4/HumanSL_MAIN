//
// State — the fixed records that cross module boundaries, and the contract
// that produces references. Eigen only: no Kortex, no Pinocchio.
//
// Architecture (mirrors msc_project's state.py / backend.py split):
//
//   reference sources          THE controller
//   ┌──────────────────┐      ┌────────────────────────┐
//   │ Targets.h        │──Reference──▶ Controller.h    │──q̇──▶ clamp → Actuation.h
//   │ (future: Vicon)  │    {pose}    │ tracks the pose  │
//   └──────────────────┘              └────────────────┘
//
// A source says WHERE to be; the controller says HOW to move there. New
// research inputs are new sources, never new controllers.
//

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Measured robot state. A field belongs here only if the Runner can fill it
// validly EVERY cycle from arm feedback alone; external sensing (Vicon,
// anything future) arrives as a Reference instead.
struct RobotState {
    Eigen::Matrix<double, 7, 1> q_rad;      // measured joint positions
    Eigen::Matrix<double, 7, 1> qdot_rad_s; // measured joint velocities
    double t_s = 0.0;                       // time since takeover
};

// Per-cycle telemetry the source and controller surface. Data only — the
// Runner decides what to print or log. NaN means "this cycle did not
// compute it" (e.g. no Jacobian on a joint reference).
struct ControllerStatus {
    Eigen::Vector3d p_desired = Eigen::Vector3d::Zero(); // current target
    Eigen::Vector3d p_current = Eigen::Vector3d::Zero(); // FK this cycle
    bool arrived_edge = false;    // first crossing under the tolerance
    double arrival_error_m = 0.0; // error norm at that crossing
    bool not_reached_edge = false; // set the cycle the non-arrival timeout fires
    double sigma_min = std::numeric_limits<double>::quiet_NaN();     // σ_min(J)
    double rot_error_rad = std::numeric_limits<double>::quiet_NaN(); // |log3|

    // The law's two terms BEFORE summation (rad/s), and the linear speed of
    // the leak twist J·q̇_null the damped projector lets into task space.
    // NaN default = "this cycle ran no law" (takeover hold). Added after the
    // 2026-08-05 stall, where these two terms silently cancelled at 218 mm
    // from the target and no printed or logged value could show it.
    Eigen::Matrix<double, 7, 1> qdot_task_rad_s =
        Eigen::Matrix<double, 7, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 7, 1> qdot_null_rad_s =
        Eigen::Matrix<double, 7, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    double null_leak_m_s = std::numeric_limits<double>::quiet_NaN();

    // Joint-trajectory following (JointTrajectorySource). The three edges
    // fire on the single cycle that activates, rejects, or completes a
    // trajectory; joint_traj_start_error_deg carries the worst-joint distance
    // that failed the activation splice guard.
    bool joint_traj_activated = false;
    bool joint_traj_rejected = false;
    bool joint_traj_complete_edge = false;
    double joint_traj_start_error_deg = 0.0;

    // The controller's joint-space following-error stop request: the wrapped
    // measured-vs-reference error exceeded config::kTrajFollowingErrorStopDeg
    // on some joint. NOT YET ENFORCED: nothing reads this flag today, so it
    // is telemetry only. The gated task that wires the joint path into
    // Main.cpp must also feed it to the same ResolveStopPriority
    // following-error input the hardware rule uses, keeping the stop reason
    // LoopStop::kFollowingError rather than adding a stop path.
    bool joint_following_error_stop = false;
    double joint_following_error_deg = 0.0;

    // MEASURED tool orientation, flange frame in the right-arm base frame.
    // Hamilton convention, hemisphere-fixed to w >= 0 so logs never jump
    // sign. Telemetry only.
    Eigen::Quaterniond tool_quat{
        std::numeric_limits<double>::quiet_NaN(),  // w
        std::numeric_limits<double>::quiet_NaN(),  // x
        std::numeric_limits<double>::quiet_NaN(),  // y
        std::numeric_limits<double>::quiet_NaN()}; // z
};

// A Cartesian velocity — the reference twist a source commands, or a
// measured one. Mirrors controller/state.py Twist. Both halves default to
// zero, which is what Python spells Twist.zero(): a source that sets
// nothing commands a stationary target.
struct Twist {
    Eigen::Vector3d linear_m_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_rad_s = Eigen::Vector3d::Zero();
};

// A desired end-effector pose and the velocity it is moving at. Empty
// `rotation` means "no orientation requested" — the controller keeps its
// takeover hold orientation. `sequence` identifies distinct targets so
// arrival fires once per target.
//
// `twist` is the FEED-FORWARD reference for the Kd term (ReactiveLaw.h
// equation 2), the counterpart of Python's WorldTarget.twist_world. Leaving
// it zero makes the Kd term pure damping, which is what every source does
// today; a source that moves its target should fill it, or the law will
// fight the motion it asked for.
struct PoseReference {
    Eigen::Vector3d p_desired;               // meters, right-arm base frame
    std::optional<Eigen::Matrix3d> rotation; // base frame; nullopt = hold
    Twist twist;                             // reference velocity, base frame
    std::uint64_t sequence = 0;
    // A profile may pass through its endpoint before it is permitted to
    // advance the target state machine.  Only a stationary terminal sample
    // is eligible to generate the controller's arrival edge.
    bool arrival_eligible = true;
};

// Per-joint (reference - measured), taken on the SHORT way round.
//
// Kortex reports joint positions on [0, 360) while trajectories, limits and
// firmware thresholds are all signed, so the same physical angle arrives a
// full turn apart: a joint truly at -20 deg reads 340 deg. A raw subtraction
// then reports 360 deg of error, which would reject every trajectory at the
// splice guard and, past it, drive a full-speed correction the wrong way
// round. std::remainder maps each difference into [-pi, pi], the same fix
// Runner.cpp and Actuation.cpp already apply at their own boundaries. A
// non-finite input stays non-finite: callers must test for that themselves.
inline Eigen::Matrix<double, 7, 1>
WrappedJointError(const Eigen::Matrix<double, 7, 1>& q_reference_rad,
                  const Eigen::Matrix<double, 7, 1>& q_measured_rad)
{
    Eigen::Matrix<double, 7, 1> error;
    for (int i = 0; i < 7; ++i)
        error[i] = std::remainder(q_reference_rad[i] - q_measured_rad[i],
                                  2.0 * M_PI);
    return error;
}

// A desired joint position and the velocity it is moving at — what a
// joint-space source (a sampled trajectory) commands. Both are seven-wide,
// radians and radians per second, in Kortex actuator order.
struct JointReference {
    Eigen::Matrix<double, 7, 1> q_rad;
    Eigen::Matrix<double, 7, 1> qdot_rad_s;
};

// What a source hands the controller each cycle: one channel, never both.
// The pose channel goes to the reactive law, the joint channel to the joint
// tracking law. Neither set means "no reference": the controller holds the
// takeover pose.
struct Reference {
    std::optional<PoseReference> pose;
    std::optional<JointReference> joint;
};
