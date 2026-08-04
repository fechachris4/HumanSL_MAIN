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
    double sigma_min = std::numeric_limits<double>::quiet_NaN();     // σ_min(J)
    double rot_error_rad = std::numeric_limits<double>::quiet_NaN(); // |log3|

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

// What a source hands the controller each cycle: an optional pose target.
// Unset means "no reference": the controller holds the takeover pose.
struct Reference {
    std::optional<PoseReference> pose;
};
