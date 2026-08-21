//
// ExecutionCore — the explicit arm execution contract (Plan 01 Task 3):
// one hardware-independent composition of the existing per-cycle command
// pipeline, in the exact frozen Runner order:
//
//   dt clamp + overrun count
//   -> measured joints from the PREVIOUS exchange (deg -> rad here, the
//      same boundary the Runner used)
//   -> world-freshness classification + stale-twist decay (relocated
//      verbatim from the block inlined at Runner.cpp:417-491; the raw
//      WorldSample comes in, the classified RobotState comes out)
//   -> controller.Measure -> reference.Get -> controller.DesiredVelocity
//      (the request_replan edge is produced here as core evidence; the
//      typed PlanningRequest publish remains adapter-to-worker I/O in Runner)
//   -> non-finite hold + consecutive counter
//   -> ClampJointVelocity -> PositionIntegration::Apply
//   == the adapter transmits the returned command frame ==
//   -> ResolveStop: the following-error fact from the reply, the adapter
//      health facts and the core's own joint-boundary fact through the
//      stop priority ladder, then the decision-12 counters.
//
// Step computes through integration and RETURNS the integrated command;
// the stop verdict is a SEPARATE second call because the facts it ranks
// (the reply to this cycle's send, the fault/servoing/acknowledgement
// state decoded from it) do not exist until the adapter has transmitted
// the command. This preserves the hardware runner's send-then-resolve
// order as a frozen invariant: a stop cycle still sends its (held) frame
// first, exactly as Runner.cpp:557 precedes Runner.cpp:590.
//
// What must never enter this type: Kortex frame IDs, acknowledgements,
// firmware fault-bank decoding, servoing-mode reads, Vicon SDK calls,
// logging, sleeps, planner solve/I/O, terminal output. Takeover T1-T6 and
// teardown remain the Runner's. Step and ResolveStop are allocation-free
// after Seed (tests/test_execution_core.cpp proves this with a counting
// operator new/delete hook) and perform fixed-size computation only.
//
// Units and frames: joint measurements enter in actuator-reported degrees
// (bounded joints wrap at [0, 360)); all control mathematics runs in
// radians/metres/seconds; commands leave in continuous degrees (the
// Kortex boundary unit). Every Cartesian quantity is expressed in Vicon
// world W (State.h).
//

#pragma once

#include <array>

#include <Eigen/Dense>

#include "Actuation.h"
#include "CartesianReference.h"
#include "Config.h"
#include "Controller.h"
#include "ExecutionConfig.h"
#include "State.h"
#include "StopPriority.h"

class DualArmKinematics;

// Generic health facts for ONE completed feedback sample, already decoded
// by the adapter (Safety.cpp fault banks, the servoing-mode field and the
// acknowledgement freshness monitor on hardware). The core never sees
// Kortex types; it only ranks these facts. The user/operator stop is
// deliberately NOT here: it is the adapter's loop condition and never
// entered the stop priority ladder (Runner.cpp:384).
struct AdapterHealth {
    bool live_fault = false;           // any live fault bit this sample
    bool low_level_state_lost = false; // arm left LOW_LEVEL_SERVOING
    bool stale_feedback = false;       // acknowledgement freshness expired
};

// One cycle's inputs, assembled by the adapter from the previous exchange
// reply and the wait-free world slot read.
struct ArmExecutionInput {
    // Raw measured cycle interval (t_now - t_prev, s). The core clamps it
    // for integration (never more than 2x nominal) and counts overruns
    // from the raw value (> kOverrunFactor x nominal).
    double dt_s = 0.0;
    // Control time since normal control began (s), stamped into
    // RobotState.t_s (the null-space ramp input).
    double t_s = 0.0;
    // THIS cycle's joint measurement = the PREVIOUS exchange reply, in
    // actuator-reported degrees. Degrees -> radians happens inside the
    // core so the conversion stays bitwise-identical to the Runner's.
    JointVector measured_position_deg{};
    JointVector measured_velocity_deg_s{};
    // The RAW world sample (pose + validity + sequence + age + twist),
    // BEFORE freshness classification — the core owns classifying it.
    WorldSample world;
};

// Everything the pipeline computed this cycle. The adapter transmits
// commanded_deg / commanded_velocity_deg_s and logs the rest; nothing
// here is a hidden side effect.
struct ArmExecutionResult {
    // The classified measured snapshot the law consumed: q/qdot in rad,
    // plus the world_* fields written by the relocated freshness block.
    RobotState state;
    MeasuredCartesianState measured;
    PoseReference reference;
    ControllerStatus controller_status;
    // The controller's raw output BEFORE the non-finite hold, rad/s —
    // the evidence for a kNonFiniteCommand stop is this value.
    JointVector qdot_raw_rad_s{};
    bool nonfinite = false;   // this cycle's output was held (not integrated)
    int nonfinite_count = 0;  // consecutive held cycles (decision 12)
    bool overrun = false;     // raw dt exceeded kOverrunFactor x nominal
    int overrun_count = 0;    // consecutive overruns (decision 12)
    // After the per-joint clamp — the program's single speed limit.
    JointVector qdot_limited_deg_s{};
    std::array<bool, 7> saturated{};
    // Integration evidence: unconstrained proposal, lead-bound flags and
    // the bounded-joint whole-frame hold verdict.
    PositionIntegration::ApplyStatus actuation_status;
    // The integrated position command frame the adapter must transmit —
    // ALSO on a stop cycle (send-then-resolve; a joint-boundary hold
    // transmits the held frame before the runner stops on it).
    JointVector commanded_deg{};
    JointVector commanded_velocity_deg_s{};
};

// The stop verdict for the completed cycle: the priority-ladder decision,
// the following-error fact the core derived from the reply, and the
// decision-12 counter stops, which are evaluated ONLY when the ladder
// found nothing — non-finite first, then overrun (Runner.cpp:632-645).
struct ExecutionStopDecision {
    StopPriorityDecision priority;
    bool following_error = false;
    bool nonfinite_stop = false;
    bool overrun_stop = false;
};

class ArmExecutionCore {
public:
    // Deviations from the Plan 01 interface sketch, with reasons:
    //  - takes the kinematic model (TrackingController cannot exist
    //    without one) and the nominal cycle period (the runner owns
    //    timing; the value must match the pacing period it enforces);
    //  - does NOT take a PlanningArm: the PlanningRequest publish is
    //    adapter I/O and never enters the core (analysis risk R3), so
    //    the core has no use for the arm identity.
    // Copies the configuration snapshot; validates it and the period at
    // construction (never in the cycle). Throws std::invalid_argument on
    // a physically meaningless value.
    ArmExecutionCore(DualArmKinematics& model,
                     const ExecutionConfig& config,
                     CartesianTrajectoryMailbox& mailbox,
                     double nominal_dt_s);

    // Runner T5 equivalent, required exactly once before the first Step:
    // the last takeover feedback becomes both the first measurement and
    // the integrator seed (q_command = q_measured — the ONLY time command
    // state is seeded from measurement). Degrees so the seed is
    // bitwise-identical to the raw feedback (Runner.cpp:368-374). Throws
    // std::logic_error on a second Seed.
    void Seed(const JointVector& measured_position_deg,
              const JointVector& measured_velocity_deg_s);

    // One control cycle through position integration. Fixed-size
    // computation; no allocation, locks, or I/O. Throws std::logic_error
    // before Seed.
    ArmExecutionResult Step(const ArmExecutionInput& input);

    // The completed cycle's stop verdict, after the adapter transmitted
    // the returned command frame. `reply_position_deg` is the reply to
    // THIS cycle's send, in actuator-reported degrees; the core derives
    // the following-error fact from it against the frame it just
    // integrated, through the ONE shared predicate in StopPriority.h
    // (the same definition the takeover phase consumes via Safety.cpp).
    // One-shot per completed Step; throws std::logic_error otherwise, so
    // a verdict can never rank facts against a stale joint-boundary
    // state.
    ExecutionStopDecision ResolveStop(const JointVector& reply_position_deg,
                                      const AdapterHealth& health);

    // Telemetry-only canonical FK for the integrated command last returned by
    // Step. Runner calls this only after a successful exchange and a no-stop
    // ResolveStop verdict, so display work cannot delay send or stop handling.
    // Throws before Seed or while a Step still awaits ResolveStop.
    CartesianPose CommandedTcpMount();

private:
    // Immutable after construction.
    DualArmKinematics& model_;
    ExecutionConfig config_;
    double nominal_dt_s_;
    double overrun_factor_; // construction-time config::kOverrunFactor

    TrackingController controller_;
    CartesianReferenceSource reference_;
    PositionIntegration actuation_;

    bool seeded_ = false;
    bool stop_pending_ = false; // a Step completed, its stop not yet resolved
    long cycle_ = 0;
    RobotState state_{};
    // Persistent command state in the Kortex boundary unit (continuous
    // degrees), exactly the Runner's commanded_deg frame.
    JointVector commanded_deg_{};
    JointVector commanded_velocity_deg_s_{};
    bool joint_limit_warning_ = false; // fact from the last Step's Apply

    // The relocated world-staleness memory (was Runner loop-local state,
    // Runner.cpp:238-243): the last trustworthy Mount twist, whether one
    // exists, and the control-clock stale time. Reset semantics are
    // frozen: every fresh cycle zeroes the stale clock; a fresh sample
    // WITHOUT a valid twist clears the memory entirely (a pre-gap
    // velocity must never be carried into a new pose).
    Eigen::Vector3d last_fresh_mount_linear_world_m_s_ =
        Eigen::Vector3d::Zero();
    Eigen::Vector3d last_fresh_mount_angular_world_rad_s_ =
        Eigen::Vector3d::Zero();
    bool have_last_fresh_mount_twist_ = false;
    // THE stale clock (one clock, one owner — 2026-08-17 consolidation):
    // zeroed on every fresh cycle, advanced by the clamped cycle dt while
    // stale. Besides the stale-twist decay above, its post-update value is
    // handed to CartesianReferenceSource::Get each cycle as the
    // prolonged-stale policy input (the reference keeps no clock of its
    // own). ClampedCycleDt makes dt finite and >= 0, so "add every stale
    // cycle" here equals the reference's former "add only a valid dt".
    double world_stale_elapsed_s_ = 0.0;

    // Decision-12 consecutive-cycle counters.
    int nonfinite_count_ = 0;
    int overrun_count_ = 0;
};
