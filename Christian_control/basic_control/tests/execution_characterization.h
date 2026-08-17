//
// execution_characterization.h — hardware-free harness that freezes the
// CURRENT per-cycle command-generation behaviour of the controller
// (Plan 01, Task 1: characterize before extraction).
//
// The harness drives the REAL production units — TrackingController,
// CartesianReferenceSource, ClampJointVelocity, PositionIntegration and
// ResolveStopPriority — in exactly the order Runner.cpp:384-645 runs them:
//
//   dt sample + overrun count
//   -> joint measurement from the PREVIOUS exchange (deg -> rad)
//   -> world sample classification: ZOH pose, freshness verdict,
//      stale-twist decay (the block inlined at Runner.cpp:417-491)
//   -> controller.Measure
//   -> reference.Get
//   -> controller.DesiredVelocity
//   -> planning-request EDGE (the FIFO publish itself is adapter I/O and
//      stays outside; only the edge is core evidence)
//   -> non-finite hold + consecutive counter
//   -> ClampJointVelocity
//   -> PositionIntegration::Apply
//   -> (the Kortex Send happens HERE in hardware; the harness has no
//      exchange, so the cycle's own feedback is a fixture input)
//   -> ResolveStopPriority, then the non-finite/overrun counter stops.
//
// What is deliberately NOT here, because it is Kortex/Vicon adapter
// ownership and must never enter the extracted core: takeover T1-T6, the
// cyclic Send/Feedback exchange, acknowledgement freshness counting (the
// stale-feedback FACT is a fixture input), fault-bank decoding, logging,
// sleeps and teardown.
//
// Fixture-input conventions (binding corrections from the adversarial
// analysis review):
//   - the world input is the RAW slot sample (Mount pose + valid flag,
//     sequence, age, raw Mount twist + validity), NOT the classified
//     RobotState — the classification is exactly the logic Task 3
//     relocates, so it must be an OUTPUT here, never an input;
//   - the expected record carries the ControllerStatus edges and the
//     stop decision, so single-cycle edges and stop precedence are
//     replayable byte-for-byte;
//   - feedback_deg is the reply of THIS cycle's (absent) exchange: the
//     harness measures it NEXT cycle, encoding the pipeline's inherent
//     one-cycle measurement delay.
//
// Serialization uses 17 significant digits, which round-trips IEEE754
// doubles exactly, so fixture verification is exact text equality.
//
// Test-only code: allocation and file I/O are fine here. The 500 Hz
// constraints apply to the production units this harness CALLS, not to
// the harness itself.
//

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Actuation.h"
#include "CartesianReference.h"
#include "CartesianTrajectoryMailbox.h"
#include "Config.h"
#include "Controller.h"
#include "Frames.h"
#include "Kinematics.h"
#include "State.h"
#include "StopPriority.h"

namespace characterization {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr int kNumJoints = 7;
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ---------------------------------------------------------------------
// Input records (one fixture row = one control cycle)
// ---------------------------------------------------------------------

// The RAW world sample as the Runner's wait-free slot read + age
// computation delivers it, BEFORE any freshness classification. Mirrors
// the BasePoseSample fields the world block actually consumes. Pose in
// Vicon-world metres, quaternion x,y,z,w; twist in world axes.
struct WorldSampleInput {
    bool mount_valid = false;
    Eigen::Vector3d mount_position_m{kNaN, kNaN, kNaN};
    double mount_quat_xyzw[4] = {kNaN, kNaN, kNaN, kNaN};
    std::uint64_t sequence = 0; // 0 = no sample has ever arrived
    double age_s = kNaN;        // sample age at cycle start (NaN = never)
    bool mount_twist_valid = false;
    Eigen::Vector3d mount_linear_world_m_s{kNaN, kNaN, kNaN};
    Eigen::Vector3d mount_angular_world_rad_s{kNaN, kNaN, kNaN};
};

// Generic adapter health facts. On hardware these come from the Kortex
// feedback sample (Safety.cpp) and the acknowledgement monitor; here they
// are fixture inputs so stop precedence is pinned without hardware.
struct AdapterFacts {
    bool live_fault = false;
    bool left_low_level = false;
    bool stale_feedback = false;
};

// An optional two-point WORLD trajectory published into the mailbox
// before this cycle's reference Get (models "a plan arrived earlier").
// Two points exercise activation, interpolation, completion and
// rejection; the wire contract allows more but the pipeline logic does
// not branch on the count beyond front/back/interior.
struct TrajectoryInput {
    bool publish = false;
    std::uint64_t trajectory_id = 0;
    std::uint64_t planner_vicon_sequence = 0;
    WorldCartesianTrajectoryPoint points[2];
};

struct CharacterizationCycle {
    double dt_s = config::kControlDtS; // raw measured cycle dt (t_now - t_prev)
    double t_s = 0.0;                  // control time stamped into RobotState
    JointVector feedback_deg{};        // THIS cycle's exchange reply, raw deg
    JointVector feedback_velocity_deg_s{};
    WorldSampleInput world;
    AdapterFacts adapter;
    TrajectoryInput trajectory;
};

// ---------------------------------------------------------------------
// Expected record (everything the pipeline computed this cycle)
// ---------------------------------------------------------------------

// Stop verdict for one cycle. 0-5 mirror StopPriorityReason's enumerator
// order exactly; 6 and 7 are the Runner's consecutive-counter stops that
// are checked only after the priority ladder (Runner.cpp:632-645).
enum class StopKind : int {
    kNone = 0,
    kRobotFault = 1,
    kFollowingError = 2,
    kLeftLowLevel = 3,
    kJointLimitWarning = 4,
    kStaleFeedback = 5,
    kNonFiniteCommand = 6,
    kOverrun = 7,
};

struct CharacterizationExpected {
    // Post-classification world state — the OUTPUT of the block Task 3
    // relocates (ZOH pose freeze, freshness verdict, stale-twist decay).
    bool world_fresh = false;
    bool world_mount_twist_valid = false;
    Eigen::Vector3d world_p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d world_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d world_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d world_w = Eigen::Vector3d::Zero();

    // Measurement (MeasuredCartesianState, world frame W).
    Eigen::Vector3d meas_p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d meas_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d meas_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d meas_w = Eigen::Vector3d::Zero();

    // Controller telemetry that must survive extraction.
    double sigma_min = kNaN;
    double rot_error_rad = kNaN;
    double null_leak_m_s = kNaN;
    Eigen::Matrix<double, 7, 1> qdot_task_rad_s =
        Eigen::Matrix<double, 7, 1>::Constant(kNaN);
    Eigen::Matrix<double, 7, 1> qdot_null_rad_s =
        Eigen::Matrix<double, 7, 1>::Constant(kNaN);

    // Reference — an expected OUTPUT of the state machine under test.
    Eigen::Vector3d ref_p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ref_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ref_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d ref_w = Eigen::Vector3d::Zero();
    std::uint64_t ref_trajectory_id = 0;
    std::uint64_t ref_planner_vicon_sequence = 0;
    double ref_time_s = 0.0;
    bool ref_arrival_eligible = false;

    // Single-cycle edges (ControllerStatus).
    bool arrived_edge = false;
    bool not_reached_edge = false;
    double arrival_error_m = 0.0;
    bool traj_activated = false;
    bool traj_rejected = false;
    bool traj_complete_edge = false;
    bool traj_cancelled_edge = false;
    bool request_replan_edge = false;
    double start_position_error_m = 0.0;
    double start_orientation_error_rad = 0.0;

    // Command chain.
    Eigen::Matrix<double, 7, 1> qdot_raw_rad_s = // BEFORE the non-finite hold
        Eigen::Matrix<double, 7, 1>::Zero();
    bool nonfinite = false;
    int nonfinite_count = 0;
    bool overrun = false;
    int overrun_count = 0;
    JointVector qdot_limited_deg_s{}; // after the per-joint clamp
    std::array<bool, 7> saturated{};
    JointVector requested_position_deg{}; // integrator proposal, pre-bounds
    std::array<bool, 7> lead_limited{};
    int joint_limit_warning_joint = -1; // -1 = none
    JointVector commanded_deg{};          // setpoints the Runner would SEND
    JointVector commanded_velocity_deg_s{};

    // Stop decision — resolved AFTER the send point (send-then-resolve).
    StopKind stop = StopKind::kNone;
    bool live_fault_observed = false;
};

// ---------------------------------------------------------------------
// The harness
// ---------------------------------------------------------------------

class CharacterizationHarness {
public:
    CharacterizationHarness()
        : dynamics_(GEN3_DUAL_URDF_PATH),
          model_(dynamics_, Arm::kRight, config::kLeftNominalRad,
                 config::kRightBaseFrame, config::kRightEndEffectorFrame),
          controller_(model_),
          reference_(mailbox_),
          actuation_(config::kCommandLeadLimitDeg)
    {
    }

    // T5 equivalent: capture the last takeover feedback as both the first
    // measurement and the integrator seed (q_command = q_measured — the
    // only time command state is seeded from measurement).
    void Seed(const JointVector& feedback_deg,
              const JointVector& feedback_velocity_deg_s)
    {
        prev_feedback_deg_ = feedback_deg;
        prev_feedback_velocity_deg_s_ = feedback_velocity_deg_s;
        for (int i = 0; i < kNumJoints; ++i) {
            state_.q_rad[i] = feedback_deg[i] * kDegToRad;
            state_.qdot_rad_s[i] = feedback_velocity_deg_s[i] * kDegToRad;
            commanded_deg_[i] = feedback_deg[i];
            commanded_velocity_deg_s_[i] = 0.0;
        }
        actuation_.Prepare(state_);
        seeded_ = true;
    }

    // One control cycle in the exact Runner.cpp order (see file comment).
    CharacterizationExpected Step(const CharacterizationCycle& in)
    {
        if (!seeded_)
            throw std::logic_error("CharacterizationHarness::Step before Seed");
        CharacterizationExpected out;

        // --- dt sampling and overrun counting (Runner.cpp:391-406). The
        // raw measured dt drives the overrun counter; the CLAMPED dt is
        // what control integrates, so a stall cannot integrate one large
        // position jump. The first cycle uses the nominal period.
        const double nominal_dt_s = config::kControlDtS;
        const double dt_s = cycle_ == 0
            ? nominal_dt_s
            : ClampedCycleDt(in.dt_s, nominal_dt_s);
        if (cycle_ > 0 && in.dt_s > config::kOverrunFactor * nominal_dt_s) {
            ++overrun_count_;
            out.overrun = true;
        } else {
            overrun_count_ = 0;
        }
        out.overrun_count = overrun_count_;

        // --- measured state from the PREVIOUS exchange, deg -> rad at
        // this boundary (Runner.cpp:410-415).
        for (int i = 0; i < kNumJoints; ++i) {
            state_.q_rad[i] = prev_feedback_deg_[i] * kDegToRad;
            state_.qdot_rad_s[i] =
                prev_feedback_velocity_deg_s_[i] * kDegToRad;
        }
        state_.t_s = in.t_s;

        // --- world attachment for this cycle (Runner.cpp:421-491),
        // verbatim relocation of the inlined block: a valid Mount pose
        // updates the ZOH state, an occluded or absent one keeps the LAST
        // pose and only the trust flag drops; a stale sample decays the
        // last trustworthy Mount twist instead of stepping it to zero; a
        // fresh estimator-reset frame (twist invalid) clears the pre-gap
        // twist memory so old motion is never carried into a new pose.
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
                                 in.world.age_s <= config::kWorldFreshMaxAgeS;
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
                        config::kWorldFreshMaxAgeS);
                const double decay =
                    world_frames::StaleTwistDecayMultiplier(
                        config::kWorldFreshMaxAgeS + stale_duration_s,
                        config::kWorldFreshMaxAgeS,
                        config::kViconMountTwistFilterTauS);
                state_.world_v_mountseg_m_s =
                    decay * last_fresh_mount_linear_world_m_s_;
                state_.world_w_mountseg_rad_s =
                    decay * last_fresh_mount_angular_world_rad_s_;
                state_.world_mount_twist_valid = true;
            }
        }
        out.world_fresh = state_.world_fresh;
        out.world_mount_twist_valid = state_.world_mount_twist_valid;
        out.world_p = state_.world_p_mountseg;
        out.world_R = state_.world_R_mountseg;
        out.world_v = state_.world_v_mountseg_m_s;
        out.world_w = state_.world_w_mountseg_rad_s;

        // --- a plan arriving before this cycle (adapter FIFO thread on
        // hardware; the reference source Takes at most one per cycle).
        if (in.trajectory.publish) {
            auto trajectory = std::make_unique<WorldCartesianTrajectory>();
            trajectory->trajectory_id = in.trajectory.trajectory_id;
            trajectory->planner_vicon_sequence =
                in.trajectory.planner_vicon_sequence;
            trajectory->points = {in.trajectory.points[0],
                                  in.trajectory.points[1]};
            mailbox_.Publish(std::move(trajectory));
        }

        // --- one measurement, one reference, the sole Cartesian law
        // (Runner.cpp:497-503).
        ControllerStatus status{};
        const MeasuredCartesianState measured = controller_.Measure(state_);
        const PoseReference cycle_reference =
            reference_.Get(state_, measured, dt_s, world_stale_elapsed_s_,
                           status);
        Eigen::Matrix<double, 7, 1> qdot_raw_rad_s =
            controller_.DesiredVelocity(state_, measured, cycle_reference,
                                        dt_s, status);

        out.meas_p = measured.ee_pose_world.position_m;
        out.meas_R = measured.ee_pose_world.rotation;
        out.meas_v = measured.ee_twist_world.linear_m_s;
        out.meas_w = measured.ee_twist_world.angular_rad_s;
        out.sigma_min = status.sigma_min;
        out.rot_error_rad = status.rot_error_rad;
        out.null_leak_m_s = status.null_leak_m_s;
        out.qdot_task_rad_s = status.qdot_task_rad_s;
        out.qdot_null_rad_s = status.qdot_null_rad_s;
        out.ref_p = cycle_reference.ee_pose_world.position_m;
        out.ref_R = cycle_reference.ee_pose_world.rotation;
        out.ref_v = cycle_reference.ee_twist_world.linear_m_s;
        out.ref_w = cycle_reference.ee_twist_world.angular_rad_s;
        out.ref_trajectory_id = cycle_reference.trajectory_id;
        out.ref_planner_vicon_sequence =
            cycle_reference.planner_vicon_sequence;
        out.ref_time_s = cycle_reference.t_from_start_s;
        out.ref_arrival_eligible = cycle_reference.arrival_eligible;
        out.arrived_edge = status.arrived_edge;
        out.not_reached_edge = status.not_reached_edge;
        out.arrival_error_m = status.arrival_error_m;
        out.traj_activated = status.cartesian_traj_activated;
        out.traj_rejected = status.cartesian_traj_rejected;
        out.traj_complete_edge = status.cartesian_traj_complete_edge;
        out.traj_cancelled_edge = status.cartesian_traj_cancelled_edge;
        // The planning-request PUBLISH (Runner.cpp:507-521) is adapter
        // I/O; the EDGE below is the core evidence and sits, as in the
        // Runner, before the non-finite hold.
        out.request_replan_edge = status.request_replan_edge;
        out.start_position_error_m =
            status.cartesian_traj_start_position_error_m;
        out.start_orientation_error_rad =
            status.cartesian_traj_start_orientation_error_rad;

        // --- requested velocity captured BEFORE the non-finite hold
        // (Runner.cpp:527-539): a non-finite value is recorded as such —
        // that is the evidence for the kNonFiniteCommand stop.
        out.qdot_raw_rad_s = qdot_raw_rad_s;
        if (!qdot_raw_rad_s.allFinite()) {
            qdot_raw_rad_s.setZero();
            ++nonfinite_count_;
            out.nonfinite = true;
        } else {
            nonfinite_count_ = 0;
        }
        out.nonfinite_count = nonfinite_count_;

        // --- per-joint clamp, then position integration
        // (Runner.cpp:545-551). A pinned clamp is allowed indefinitely.
        const JointVelocityClampResult clamp_result =
            ClampJointVelocity(qdot_raw_rad_s, config::kQdotLimitDegS);
        for (int i = 0; i < kNumJoints; ++i) {
            out.qdot_limited_deg_s[i] =
                clamp_result.qdot_rad_s[i] * kRadToDeg;
            out.saturated[i] = clamp_result.saturated[i];
        }
        const PositionIntegration::ApplyStatus actuation_status =
            actuation_.Apply(clamp_result.qdot_rad_s, state_, dt_s,
                             commanded_deg_, commanded_velocity_deg_s_);
        out.requested_position_deg = actuation_status.requested_deg;
        out.lead_limited = actuation_status.lead_limited;
        out.joint_limit_warning_joint =
            actuation_status.joint_limit_warning_joint.value_or(-1);
        out.commanded_deg = commanded_deg_;
        out.commanded_velocity_deg_s = commanded_velocity_deg_s_;

        // --- the hardware Send happens here (send-then-resolve): the
        // commanded frame above is what the Runner would transmit even on
        // a stop cycle. This cycle's exchange reply is the fixture input.
        ++cycle_;

        // --- stop-fact assembly from the completed feedback sample. The
        // following-error predicate replicates Safety.cpp exactly: the
        // reply is shifted within +-180 deg of the command (FillSample)
        // before the same-unit |command - measured| comparison. The live
        // fault / low-level / stale-acknowledgement FACTS are adapter
        // inputs here (Kortex owns their evidence on hardware).
        bool following_error = false;
        if (!config::kDisableFollowingErrorStop) {
            for (int i = 0; i < kNumJoints; ++i) {
                const double measured_shifted =
                    commanded_deg_[i] +
                    std::remainder(in.feedback_deg[i] - commanded_deg_[i],
                                   360.0);
                if (std::abs(measured_shifted - commanded_deg_[i]) >
                    config::kFollowingErrorLimitDeg)
                    following_error = true;
            }
        }
        const StopPriorityDecision priority = ResolveStopPriority(
            following_error, in.adapter.live_fault,
            in.adapter.left_low_level,
            actuation_status.joint_limit_warning_joint.has_value(),
            config::kStopOnFault, in.adapter.stale_feedback);
        out.live_fault_observed = priority.live_fault_observed;
        out.stop = static_cast<StopKind>(static_cast<int>(priority.reason));
        // Decision-12 counters run only when the priority ladder found
        // nothing (Runner.cpp:632-645): non-finite first, then overrun.
        if (priority.reason == StopPriorityReason::kNone) {
            if (config::kNonFiniteStopCycles > 0 &&
                nonfinite_count_ >= config::kNonFiniteStopCycles)
                out.stop = StopKind::kNonFiniteCommand;
            else if (config::kOverrunStopCycles > 0 &&
                     overrun_count_ >= config::kOverrunStopCycles)
                out.stop = StopKind::kOverrun;
        }

        // --- this cycle's exchange reply becomes the NEXT cycle's
        // measurement (the pipeline's one-cycle measurement delay).
        prev_feedback_deg_ = in.feedback_deg;
        prev_feedback_velocity_deg_s_ = in.feedback_velocity_deg_s;

        // Reclaim retired trajectory allocations (the non-real-time input
        // thread's job on hardware; harmless between cycles in a test).
        mailbox_.DeleteRetired();
        return out;
    }

private:
    Dynamics dynamics_;
    DualArmKinematics model_;
    CartesianTrajectoryMailbox mailbox_;
    TrackingController controller_;
    CartesianReferenceSource reference_;
    PositionIntegration actuation_;

    bool seeded_ = false;
    long cycle_ = 0;
    RobotState state_{};
    JointVector prev_feedback_deg_{};
    JointVector prev_feedback_velocity_deg_s_{};
    JointVector commanded_deg_{};
    JointVector commanded_velocity_deg_s_{};
    // Runner loop-local world staleness state (Runner.cpp:238-243).
    Eigen::Vector3d last_fresh_mount_linear_world_m_s_ =
        Eigen::Vector3d::Zero();
    Eigen::Vector3d last_fresh_mount_angular_world_rad_s_ =
        Eigen::Vector3d::Zero();
    bool have_last_fresh_mount_twist_ = false;
    double world_stale_elapsed_s_ = 0.0;
    // Decision-12 consecutive-cycle counters (Safety.h CycleCounters).
    int nonfinite_count_ = 0;
    int overrun_count_ = 0;
};

// ---------------------------------------------------------------------
// One characterization case: seed + input rows.
// ---------------------------------------------------------------------

struct CharacterizationCase {
    std::string name; // one token, no commas/spaces
    JointVector seed_deg{};
    JointVector seed_velocity_deg_s{};
    std::vector<CharacterizationCycle> rows;
};

// ---------------------------------------------------------------------
// Fixture serialization. One flat CSV: '#' provenance lines, a header
// row, then one row per cycle. 17 significant digits round-trips a
// double exactly, so verification compares text for equality.
// ---------------------------------------------------------------------

inline std::string FormatDouble(double value)
{
    std::ostringstream out;
    out.precision(17);
    out << value;
    return out.str();
}

inline double ParseDouble(const std::string& text)
{
    if (text.empty())
        throw std::runtime_error("empty numeric fixture field");
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == nullptr || *end != '\0')
        throw std::runtime_error("bad numeric fixture field: " + text);
    return value;
}

// Single traversal used for BOTH writing and reading a row, so the
// column list, writer and parser cannot drift apart.
class RowIO {
public:
    enum class Mode { kWrite, kRead, kNames };

    explicit RowIO(Mode mode) : mode_(mode) {}
    void AttachInput(const std::vector<std::string>* fields)
    {
        input_ = fields;
        index_ = 0;
    }

    std::vector<std::string>& fields() { return fields_; }
    const std::vector<std::string>& names() const { return names_; }

    void Text(const char* name, std::string& value)
    {
        if (mode_ == Mode::kNames) { names_.push_back(name); return; }
        if (mode_ == Mode::kWrite) { fields_.push_back(value); return; }
        value = Next(name);
    }
    void D(const char* name, double& value)
    {
        if (mode_ == Mode::kNames) { names_.push_back(name); return; }
        if (mode_ == Mode::kWrite) { fields_.push_back(FormatDouble(value)); return; }
        value = ParseDouble(Next(name));
    }
    void B(const char* name, bool& value)
    {
        if (mode_ == Mode::kNames) { names_.push_back(name); return; }
        if (mode_ == Mode::kWrite) { fields_.push_back(value ? "1" : "0"); return; }
        const std::string text = Next(name);
        if (text != "0" && text != "1")
            throw std::runtime_error(std::string("bad bool field ") + name +
                                     ": " + text);
        value = text == "1";
    }
    void I(const char* name, int& value)
    {
        if (mode_ == Mode::kNames) { names_.push_back(name); return; }
        if (mode_ == Mode::kWrite) { fields_.push_back(std::to_string(value)); return; }
        value = std::stoi(Next(name));
    }
    void U64(const char* name, std::uint64_t& value)
    {
        if (mode_ == Mode::kNames) { names_.push_back(name); return; }
        if (mode_ == Mode::kWrite) { fields_.push_back(std::to_string(value)); return; }
        value = std::stoull(Next(name));
    }

    void Joint(const char* prefix, JointVector& values)
    {
        for (int i = 0; i < kNumJoints; ++i)
            D(JointName(prefix, i).c_str(), values[i]);
    }
    void Joint(const char* prefix, Eigen::Matrix<double, 7, 1>& values)
    {
        for (int i = 0; i < kNumJoints; ++i)
            D(JointName(prefix, i).c_str(), values[i]);
    }
    void JointBool(const char* prefix, std::array<bool, 7>& values)
    {
        for (int i = 0; i < kNumJoints; ++i) {
            bool value = values[i];
            B(JointName(prefix, i).c_str(), value);
            values[i] = value;
        }
    }
    void Vec3(const char* prefix, Eigen::Vector3d& value)
    {
        static const char* axes[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; ++i) {
            const std::string name = std::string(prefix) + "_" + axes[i];
            D(name.c_str(), value[i]);
        }
    }
    void Mat3(const char* prefix, Eigen::Matrix3d& value)
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                const std::string name = std::string(prefix) + "_r" +
                                         std::to_string(r) +
                                         std::to_string(c);
                D(name.c_str(), value(r, c));
            }
    }

private:
    static std::string JointName(const char* prefix, int index)
    {
        return std::string(prefix) + "_j" + std::to_string(index + 1);
    }
    std::string Next(const char* name)
    {
        if (input_ == nullptr || index_ >= input_->size())
            throw std::runtime_error(std::string("fixture row too short at ") +
                                     name);
        return (*input_)[index_++];
    }

    Mode mode_;
    std::vector<std::string> fields_; // write mode
    std::vector<std::string> names_;  // names mode
    const std::vector<std::string>* input_ = nullptr; // read mode
    std::size_t index_ = 0;
};

inline void TraversePoint(RowIO& io, const std::string& prefix,
                          WorldCartesianTrajectoryPoint& point)
{
    io.D((prefix + "_t").c_str(), point.t_from_start_s);
    io.Vec3((prefix + "_p").c_str(), point.position_world_m);
    double qx = point.orientation_world.x();
    double qy = point.orientation_world.y();
    double qz = point.orientation_world.z();
    double qw = point.orientation_world.w();
    io.D((prefix + "_qx").c_str(), qx);
    io.D((prefix + "_qy").c_str(), qy);
    io.D((prefix + "_qz").c_str(), qz);
    io.D((prefix + "_qw").c_str(), qw);
    point.orientation_world = Eigen::Quaterniond(qw, qx, qy, qz);
    io.Vec3((prefix + "_v").c_str(), point.linear_velocity_world_m_s);
    io.Vec3((prefix + "_w").c_str(), point.angular_velocity_world_rad_s);
    io.B((prefix + "_elig").c_str(), point.arrival_eligible);
}

// The complete row schema: case identity + seed + inputs + expected.
inline void TraverseRow(RowIO& io, std::string& case_name, int& row_index,
                        JointVector& seed_deg,
                        JointVector& seed_velocity_deg_s,
                        CharacterizationCycle& in,
                        CharacterizationExpected& exp)
{
    io.Text("case", case_name);
    io.I("row", row_index);
    io.Joint("seed", seed_deg);
    io.Joint("seedvel", seed_velocity_deg_s);

    io.D("in_dt_s", in.dt_s);
    io.D("in_t_s", in.t_s);
    io.Joint("in_fb", in.feedback_deg);
    io.Joint("in_fbvel", in.feedback_velocity_deg_s);
    io.B("in_world_valid", in.world.mount_valid);
    io.U64("in_world_seq", in.world.sequence);
    io.D("in_world_age_s", in.world.age_s);
    io.Vec3("in_world_p", in.world.mount_position_m);
    io.D("in_world_qx", in.world.mount_quat_xyzw[0]);
    io.D("in_world_qy", in.world.mount_quat_xyzw[1]);
    io.D("in_world_qz", in.world.mount_quat_xyzw[2]);
    io.D("in_world_qw", in.world.mount_quat_xyzw[3]);
    io.B("in_world_twist_valid", in.world.mount_twist_valid);
    io.Vec3("in_world_v", in.world.mount_linear_world_m_s);
    io.Vec3("in_world_w", in.world.mount_angular_world_rad_s);
    io.B("in_live_fault", in.adapter.live_fault);
    io.B("in_left_low_level", in.adapter.left_low_level);
    io.B("in_stale_feedback", in.adapter.stale_feedback);
    io.B("in_pub_traj", in.trajectory.publish);
    io.U64("in_traj_id", in.trajectory.trajectory_id);
    io.U64("in_traj_seq", in.trajectory.planner_vicon_sequence);
    TraversePoint(io, "in_traj_p0", in.trajectory.points[0]);
    TraversePoint(io, "in_traj_p1", in.trajectory.points[1]);

    io.B("exp_world_fresh", exp.world_fresh);
    io.B("exp_world_twist_valid", exp.world_mount_twist_valid);
    io.Vec3("exp_world_p", exp.world_p);
    io.Mat3("exp_world_R", exp.world_R);
    io.Vec3("exp_world_v", exp.world_v);
    io.Vec3("exp_world_w", exp.world_w);
    io.Vec3("exp_meas_p", exp.meas_p);
    io.Mat3("exp_meas_R", exp.meas_R);
    io.Vec3("exp_meas_v", exp.meas_v);
    io.Vec3("exp_meas_w", exp.meas_w);
    io.D("exp_sigma_min", exp.sigma_min);
    io.D("exp_rot_error_rad", exp.rot_error_rad);
    io.D("exp_null_leak_m_s", exp.null_leak_m_s);
    io.Joint("exp_qdot_task", exp.qdot_task_rad_s);
    io.Joint("exp_qdot_null", exp.qdot_null_rad_s);
    io.Vec3("exp_ref_p", exp.ref_p);
    io.Mat3("exp_ref_R", exp.ref_R);
    io.Vec3("exp_ref_v", exp.ref_v);
    io.Vec3("exp_ref_w", exp.ref_w);
    io.U64("exp_ref_traj_id", exp.ref_trajectory_id);
    io.U64("exp_ref_planner_seq", exp.ref_planner_vicon_sequence);
    io.D("exp_ref_time_s", exp.ref_time_s);
    io.B("exp_ref_arrival_eligible", exp.ref_arrival_eligible);
    io.B("exp_arrived_edge", exp.arrived_edge);
    io.B("exp_not_reached_edge", exp.not_reached_edge);
    io.D("exp_arrival_error_m", exp.arrival_error_m);
    io.B("exp_traj_activated", exp.traj_activated);
    io.B("exp_traj_rejected", exp.traj_rejected);
    io.B("exp_traj_complete", exp.traj_complete_edge);
    io.B("exp_traj_cancelled", exp.traj_cancelled_edge);
    io.B("exp_replan_edge", exp.request_replan_edge);
    io.D("exp_start_pos_err_m", exp.start_position_error_m);
    io.D("exp_start_rot_err_rad", exp.start_orientation_error_rad);
    io.Joint("exp_qdot_raw", exp.qdot_raw_rad_s);
    io.B("exp_nonfinite", exp.nonfinite);
    io.I("exp_nonfinite_count", exp.nonfinite_count);
    io.B("exp_overrun", exp.overrun);
    io.I("exp_overrun_count", exp.overrun_count);
    io.Joint("exp_qdot_lim", exp.qdot_limited_deg_s);
    io.JointBool("exp_sat", exp.saturated);
    io.Joint("exp_req", exp.requested_position_deg);
    io.JointBool("exp_lead", exp.lead_limited);
    io.I("exp_jlw_joint", exp.joint_limit_warning_joint);
    io.Joint("exp_cmd", exp.commanded_deg);
    io.Joint("exp_cmdvel", exp.commanded_velocity_deg_s);
    int stop = static_cast<int>(exp.stop);
    io.I("exp_stop_reason", stop);
    exp.stop = static_cast<StopKind>(stop);
    io.B("exp_live_fault_observed", exp.live_fault_observed);
}

inline std::vector<std::string> ColumnNames()
{
    RowIO io(RowIO::Mode::kNames);
    std::string case_name;
    int row_index = 0;
    JointVector seed{}, seedvel{};
    CharacterizationCycle in;
    CharacterizationExpected exp;
    TraverseRow(io, case_name, row_index, seed, seedvel, in, exp);
    return io.names();
}

inline std::string JoinComma(const std::vector<std::string>& fields)
{
    std::string line;
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i > 0)
            line += ',';
        line += fields[i];
    }
    return line;
}

inline std::vector<std::string> SplitComma(const std::string& line)
{
    std::vector<std::string> fields;
    std::string current;
    for (const char c : line) {
        if (c == ',') {
            fields.push_back(current);
            current.clear();
        } else {
            current += c;
        }
    }
    fields.push_back(current);
    return fields;
}

// Runs every case through a FRESH harness (one per case, like one run of
// the controller) and returns the per-case expected records.
inline std::vector<std::vector<CharacterizationExpected>>
RunCases(const std::vector<CharacterizationCase>& cases)
{
    std::vector<std::vector<CharacterizationExpected>> results;
    results.reserve(cases.size());
    for (const CharacterizationCase& one_case : cases) {
        CharacterizationHarness harness;
        harness.Seed(one_case.seed_deg, one_case.seed_velocity_deg_s);
        std::vector<CharacterizationExpected> expected;
        expected.reserve(one_case.rows.size());
        for (const CharacterizationCycle& row : one_case.rows)
            expected.push_back(harness.Step(row));
        results.push_back(std::move(expected));
    }
    return results;
}

// Header + rows (no provenance lines): the part of the fixture that must
// be byte-identical between generation runs and on verification.
inline std::string SerializeDataSection(
    const std::vector<CharacterizationCase>& cases,
    const std::vector<std::vector<CharacterizationExpected>>& results)
{
    std::string text = JoinComma(ColumnNames()) + "\n";
    for (std::size_t c = 0; c < cases.size(); ++c) {
        // Local mutable copies: TraverseRow takes non-const references so
        // one function can also parse.
        CharacterizationCase one_case = cases[c];
        for (std::size_t r = 0; r < one_case.rows.size(); ++r) {
            RowIO io(RowIO::Mode::kWrite);
            int row_index = static_cast<int>(r);
            CharacterizationExpected exp = results[c][r];
            TraverseRow(io, one_case.name, row_index, one_case.seed_deg,
                        one_case.seed_velocity_deg_s, one_case.rows[r], exp);
            text += JoinComma(io.fields()) + "\n";
        }
    }
    return text;
}

struct ParsedFixture {
    std::vector<CharacterizationCase> cases;
    std::vector<std::vector<CharacterizationExpected>> expected;
    std::string data_section; // header + rows exactly as stored
};

// Parses a fixture file body (provenance '#' lines skipped). Throws on
// any structural mismatch — a fixture that cannot be read exactly must
// never be silently "repaired".
inline ParsedFixture ParseFixture(std::istream& input)
{
    ParsedFixture fixture;
    std::string line;
    // Skip provenance.
    while (std::getline(input, line)) {
        if (!line.empty() && line[0] == '#')
            continue;
        break;
    }
    const std::vector<std::string> expected_header = ColumnNames();
    if (SplitComma(line) != expected_header)
        throw std::runtime_error(
            "fixture header does not match the compiled schema");
    fixture.data_section = line + "\n";
    while (std::getline(input, line)) {
        if (line.empty())
            continue;
        fixture.data_section += line + "\n";
        const std::vector<std::string> fields = SplitComma(line);
        if (fields.size() != expected_header.size())
            throw std::runtime_error("fixture row has wrong field count");
        RowIO io(RowIO::Mode::kRead);
        io.AttachInput(&fields);
        std::string case_name;
        int row_index = 0;
        JointVector seed{}, seedvel{};
        CharacterizationCycle in;
        CharacterizationExpected exp;
        TraverseRow(io, case_name, row_index, seed, seedvel, in, exp);
        if (fixture.cases.empty() || fixture.cases.back().name != case_name) {
            CharacterizationCase next;
            next.name = case_name;
            next.seed_deg = seed;
            next.seed_velocity_deg_s = seedvel;
            fixture.cases.push_back(next);
            fixture.expected.emplace_back();
        }
        if (row_index !=
            static_cast<int>(fixture.cases.back().rows.size()))
            throw std::runtime_error("fixture rows out of order in case " +
                                     case_name);
        fixture.cases.back().rows.push_back(in);
        fixture.expected.back().push_back(exp);
    }
    return fixture;
}

} // namespace characterization
