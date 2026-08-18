//
// ExecutionCore — implementation of the contract declared in
// ExecutionCore.h. Composes the existing units; duplicates no controller
// equation. The world-freshness classification and stale-twist decay are
// the one genuine relocation (verbatim from the block inlined at
// Runner.cpp:417-491); the frozen Task 1 characterization fixture is the
// evidence the move changed nothing.
//

#include "ExecutionCore.h"

#include <cmath>
#include <stdexcept>

#include "Frames.h"

namespace
{
    constexpr int kNumJoints = 7;
    // The same conversion expressions the Runner and Controller compile,
    // so every converted value is bitwise-identical to the pre-extraction
    // pipeline.
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    const ExecutionConfig& Validated(const ExecutionConfig& config)
    {
        ValidateExecutionConfig(config);
        return config;
    }
} // namespace

ArmExecutionCore::ArmExecutionCore(DualArmKinematics& model,
                                   const ExecutionConfig& execution_config,
                                   CartesianTrajectoryMailbox& mailbox,
                                   double nominal_dt_s)
    : config_(Validated(execution_config)),
      nominal_dt_s_(nominal_dt_s),
      // Construction-time snapshot: the overrun definition is compiled
      // policy (Config.h) not yet represented in ExecutionConfig; it is
      // read once here, never in the cycle.
      overrun_factor_(config::kOverrunFactor),
      controller_(model, config_),
      reference_(mailbox, config_),
      actuation_(config_)
{
    if (!std::isfinite(nominal_dt_s_) || nominal_dt_s_ <= 0.0)
        throw std::invalid_argument(
            "ArmExecutionCore: nominal_dt_s must be finite and > 0");
}

void ArmExecutionCore::Seed(const JointVector& measured_position_deg,
                            const JointVector& measured_velocity_deg_s)
{
    if (seeded_)
        throw std::logic_error(
            "ArmExecutionCore::Seed called twice — the command state is "
            "seeded from measurement exactly once, at takeover");
    for (int i = 0; i < kNumJoints; ++i) {
        state_.q_rad[i] = measured_position_deg[i] * kDegToRad;
        state_.qdot_rad_s[i] = measured_velocity_deg_s[i] * kDegToRad;
        commanded_deg_[i] = measured_position_deg[i];
        commanded_velocity_deg_s_[i] = 0.0;
    }
    actuation_.Prepare(state_);
    seeded_ = true;
}

ArmExecutionResult ArmExecutionCore::Step(const ArmExecutionInput& in)
{
    if (!seeded_)
        throw std::logic_error("ArmExecutionCore::Step before Seed");
    ArmExecutionResult out;

    // --- dt sampling and overrun counting (Runner.cpp:391-406). The raw
    // measured dt drives the overrun counter; the CLAMPED dt is what
    // control integrates, so a stall cannot integrate one large position
    // jump. The first cycle uses the nominal period.
    const double dt_s = cycle_ == 0
        ? nominal_dt_s_
        : ClampedCycleDt(in.dt_s, nominal_dt_s_);
    if (cycle_ > 0 && in.dt_s > overrun_factor_ * nominal_dt_s_) {
        ++overrun_count_;
        out.overrun = true;
    } else {
        overrun_count_ = 0;
    }
    out.overrun_count = overrun_count_;

    // --- measured state from the PREVIOUS exchange, deg -> rad at this
    // boundary (Runner.cpp:410-415).
    for (int i = 0; i < kNumJoints; ++i) {
        state_.q_rad[i] = in.measured_position_deg[i] * kDegToRad;
        state_.qdot_rad_s[i] = in.measured_velocity_deg_s[i] * kDegToRad;
    }
    state_.t_s = in.t_s;

    // --- world attachment for this cycle: the relocated freshness block
    // (Runner.cpp:421-491). A valid Mount pose updates the ZOH state; an
    // occluded or absent one keeps the LAST pose (the freeze happens by
    // construction) and only the trust flag drops. A stale sample decays
    // the last trustworthy Mount twist continuously instead of stepping
    // it to zero; a fresh estimator-reset frame (twist invalid) clears
    // the pre-gap twist memory so old motion is never carried into a new
    // pose.
    {
        if (in.world.mount_valid) {
            state_.world_p_mountseg = in.world.mount_position_m;
            state_.world_R_mountseg =
                Eigen::Quaterniond(in.world.mount_quat_xyzw[3],
                                   in.world.mount_quat_xyzw[0],
                                   in.world.mount_quat_xyzw[1],
                                   in.world.mount_quat_xyzw[2])
                    .toRotationMatrix();
        }
        state_.world_fresh = in.world.mount_valid &&
                             in.world.sequence > 0 &&
                             std::isfinite(in.world.age_s) &&
                             in.world.age_s <= config_.world_fresh_max_age_s;
        state_.world_sequence = in.world.sequence;
        if (state_.world_fresh) {
            world_stale_elapsed_s_ = 0.0;
            if (in.world.mount_twist_valid) {
                state_.world_v_mountseg_m_s =
                    in.world.mount_linear_world_m_s;
                state_.world_w_mountseg_rad_s =
                    in.world.mount_angular_world_rad_s;
                last_fresh_mount_linear_world_m_s_ =
                    state_.world_v_mountseg_m_s;
                last_fresh_mount_angular_world_rad_s_ =
                    state_.world_w_mountseg_rad_s;
                have_last_fresh_mount_twist_ = true;
                state_.world_mount_twist_valid = true;
            } else {
                state_.world_v_mountseg_m_s.setZero();
                state_.world_w_mountseg_rad_s.setZero();
                last_fresh_mount_linear_world_m_s_.setZero();
                last_fresh_mount_angular_world_rad_s_.setZero();
                have_last_fresh_mount_twist_ = false;
                state_.world_mount_twist_valid = false;
            }
        } else {
            world_stale_elapsed_s_ += dt_s;
        }
        if (!state_.world_fresh && have_last_fresh_mount_twist_) {
            const double stale_duration_s =
                world_frames::EffectiveStaleDurationS(
                    in.world.age_s, world_stale_elapsed_s_,
                    config_.world_fresh_max_age_s);
            const double decay =
                world_frames::StaleTwistDecayMultiplier(
                    config_.world_fresh_max_age_s + stale_duration_s,
                    config_.world_fresh_max_age_s,
                    config_.mount_twist_filter_tau_s);
            state_.world_v_mountseg_m_s =
                decay * last_fresh_mount_linear_world_m_s_;
            state_.world_w_mountseg_rad_s =
                decay * last_fresh_mount_angular_world_rad_s_;
            state_.world_mount_twist_valid = true;
        }
    }
    out.state = state_;

    // --- one coherent measurement, then one world reference, then the
    // sole Cartesian law (Runner.cpp:497-503). The request_replan edge is
    // produced by reference.Get here; the PlanningRequest publish itself
    // is adapter I/O and happens in the Runner after Step returns.
    ControllerStatus status{};
    const MeasuredCartesianState measured = controller_.Measure(state_);
    const PoseReference cycle_reference =
        reference_.Get(state_, measured, dt_s, world_stale_elapsed_s_,
                       status);
    Eigen::Matrix<double, 7, 1> qdot_raw_rad_s =
        controller_.DesiredVelocity(state_, measured, cycle_reference,
                                    dt_s, status);

    // --- the raw output is recorded BEFORE the non-finite hold
    // (Runner.cpp:527-539): a non-finite value is the recorded evidence
    // for the kNonFiniteCommand stop. Non-finite output never reaches the
    // integrator: hold this cycle and count it (decision 12).
    for (int i = 0; i < kNumJoints; ++i)
        out.qdot_raw_rad_s[i] = qdot_raw_rad_s[i];
    if (!qdot_raw_rad_s.allFinite()) {
        qdot_raw_rad_s.setZero();
        ++nonfinite_count_;
        out.nonfinite = true;
    } else {
        nonfinite_count_ = 0;
    }
    out.nonfinite_count = nonfinite_count_;

    // --- per-joint clamp — the program's single speed limit — then the
    // actuation integrates and produces this cycle's setpoints
    // (Runner.cpp:545-551). A pinned clamp is allowed indefinitely: far
    // targets transit at clip speed by design; there is NO saturation
    // stop.
    const JointVelocityClampResult clamp_result =
        ClampJointVelocity(qdot_raw_rad_s, config_.velocity_limit_deg_s);
    for (int i = 0; i < kNumJoints; ++i) {
        out.qdot_limited_deg_s[i] = clamp_result.qdot_rad_s[i] * kRadToDeg;
        out.saturated[i] = clamp_result.saturated[i];
    }
    const PositionIntegration::ApplyStatus actuation_status =
        actuation_.Apply(clamp_result.qdot_rad_s, state_, dt_s,
                         commanded_deg_, commanded_velocity_deg_s_);

    out.measured = measured;
    out.reference = cycle_reference;
    out.controller_status = status;
    out.actuation_status = actuation_status;
    out.commanded_deg = commanded_deg_;
    out.commanded_velocity_deg_s = commanded_velocity_deg_s_;

    joint_limit_warning_ =
        actuation_status.joint_limit_warning_joint.has_value();
    ++cycle_;
    stop_pending_ = true;
    return out;
}

ExecutionStopDecision ArmExecutionCore::ResolveStop(
    const JointVector& reply_position_deg, const AdapterHealth& health)
{
    if (!stop_pending_)
        throw std::logic_error(
            "ArmExecutionCore::ResolveStop without a completed Step — the "
            "verdict would rank facts against a stale joint-boundary "
            "state");
    stop_pending_ = false;

    ExecutionStopDecision out;
    // The following-error fact: the ONE shared predicate (StopPriority.h,
    // also consumed by the takeover phase through Safety.cpp), gated by
    // this core's config snapshot of the disable flag.
    if (!config_.disable_following_error_stop)
        out.following_error = FollowingErrorExceeded(
            commanded_deg_, reply_position_deg,
            config_.following_error_limit_deg);

    // Resolve every fact independently through the frozen precedence
    // (StopPriority.h): an ignored fault must still taint the verdict and
    // must never mask following error or loss of low-level servoing.
    out.priority = ResolveStopPriority(
        out.following_error, health.live_fault, health.low_level_state_lost,
        joint_limit_warning_, config_.stop_on_fault, health.stale_feedback);

    // Decision-12 counters run only when the priority ladder found
    // nothing (Runner.cpp:632-645): non-finite first, then overrun; a
    // threshold <= 0 disables that stop.
    if (out.priority.reason == StopPriorityReason::kNone) {
        if (config_.nonfinite_stop_cycles > 0 &&
            nonfinite_count_ >= config_.nonfinite_stop_cycles)
            out.nonfinite_stop = true;
        else if (config_.overrun_stop_cycles > 0 &&
                 overrun_count_ >= config_.overrun_stop_cycles)
            out.overrun_stop = true;
    }
    return out;
}
