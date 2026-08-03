//
// State — the fixed records that cross module boundaries, and the contract
// that produces references. Eigen only: no Kortex, no Pinocchio.
//
// Architecture (mirrors msc_project's state.py / backend.py split):
//
//   reference sources          THE controller
//   ┌──────────────────┐      ┌────────────────────────┐
//   │ Targets.h        │──Reference──▶ Controller.h    │──q̇──▶ clamp → Actuation.h
//   │ Trajectory.h     │ {pose|joints}│ tracks whichever │
//   │ (future: Vicon)  │              │ channel is set   │
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

    // Trajectory telemetry: the reference this cycle's command should land
    // on, logged beside commanded and measured so planned/commanded/measured
    // stay comparable. Other sources leave the defaults.
    Eigen::Matrix<double, 7, 1> q_ref_deg =
        Eigen::Matrix<double, 7, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    double playback_t_s = std::numeric_limits<double>::quiet_NaN();
    int playback_state = 0;   // 0 none, 1 playing, 2 done-holding, 3 refused
    bool playback_done_edge = false;    // first cycle past the last sample
    bool playback_refused_edge = false; // first cycle after a Reset refusal
};

// A desired end-effector pose. Empty `rotation` means "no orientation
// requested" — the controller keeps its takeover hold orientation.
// `sequence` identifies distinct targets so arrival fires once per target.
struct PoseReference {
    Eigen::Vector3d p_desired;               // meters, right-arm base frame
    std::optional<Eigen::Matrix3d> rotation; // base frame; nullopt = hold
    std::uint64_t sequence = 0;
};

// Where the joints should be now and at t + dt. The pair encodes the
// feed-forward velocity exactly, so the integrator telescopes with zero
// discretization drift.
struct JointReference {
    Eigen::Matrix<double, 7, 1> q_ref_deg;
    Eigen::Matrix<double, 7, 1> q_ref_next_deg;
};

// What a source hands the controller each cycle; at most one channel set.
// Joint references win when present — they carry the planner's exact joint
// path, and re-solving them through the pose law would let damped least
// squares pick different joint motions and undo its collision avoidance.
// NEITHER set means "no reference": the controller holds the takeover pose.
struct Reference {
    std::optional<PoseReference> pose;
    std::optional<JointReference> joints;
};

// One Reset at takeover, then one Get per cycle. Pure computation: no I/O,
// no allocation, no blocking beyond a bounded store lock.
class ReferenceSource
{
public:
    virtual ~ReferenceSource() = default;

    // T5 of takeover: after PositionIntegration::Prepare, before the first Get.
    // Captures the source's baseline (store sequence, trajectory start gate).
    virtual void Reset(const RobotState& state) = 0;

    // This cycle's reference. May fill the telemetry fields of `status` that
    // describe the reference itself.
    virtual Reference Get(const RobotState& state, double dt_s,
                          ControllerStatus& status) = 0;
};
