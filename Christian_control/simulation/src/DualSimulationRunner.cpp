#include "DualSimulationRunner.h"

#include <stdexcept>

#include <Eigen/Geometry>

namespace
{
    const SimulationConfig& Validated(const SimulationConfig& config)
    {
        ValidateSimulationConfig(config);
        return config;
    }

    // The prescribed Mount motion at one instant: the pose the plant is
    // given and the twist the ideal sensor reports, from ONE call so they
    // cannot disagree (MountMotion.h). With the default kStatic motion
    // the pose is the anchor, bit for bit, and the twist is its exact
    // derivative: zero.
    MountTruth PrescribedMount(const SimulationConfig& config, double time_s)
    {
        return SampleMountMotion(config.mount_motion, time_s,
                                 config.world_T_mount_at_zero);
    }

    // The raw ideal WorldSample for one tick: pose/twist copied exactly
    // from the prescription (quaternion in the x,y,z,w wire order the
    // core parses — State.h:57), sequence strictly advancing from 1,
    // age 0. Raw record only; the core classifies it.
    WorldSample IdealWorldSample(const CartesianPose& world_T_mount,
                                 const Twist& world_V_mount,
                                 std::uint64_t sequence)
    {
        WorldSample sample;
        sample.mount_valid = true;
        sample.mount_position_m = world_T_mount.position_m;
        const Eigen::Quaterniond rotation(world_T_mount.rotation);
        sample.mount_quat_xyzw[0] = rotation.x();
        sample.mount_quat_xyzw[1] = rotation.y();
        sample.mount_quat_xyzw[2] = rotation.z();
        sample.mount_quat_xyzw[3] = rotation.w();
        sample.sequence = sequence;
        sample.age_s = 0.0;
        sample.mount_twist_valid = true;
        sample.mount_linear_world_m_s = world_V_mount.linear_m_s;
        sample.mount_angular_world_rad_s = world_V_mount.angular_rad_s;
        return sample;
    }

    bool StopRequested(const ExecutionStopDecision& decision)
    {
        return decision.priority.reason != StopPriorityReason::kNone ||
               decision.nonfinite_stop || decision.overrun_stop;
    }
} // namespace

DualSimulationRunner::DualSimulationRunner(const SimulationConfig& config)
    : config_(Validated(config)),
      dynamics_(config_.urdf_path),
      // Exactly the parity test's construction of the production
      // kinematics: the controlled arm selects the branch; the OTHER
      // arm's slots hold its nominal pose (config::ArmConfig).
      kinematics_right_(dynamics_, Arm::kRight, config::kLeftNominalRad,
                        config::kRightBaseFrame,
                        config::kRightEndEffectorFrame),
      kinematics_left_(dynamics_, Arm::kLeft, config::kRightNominalRad,
                       config::kRightBaseFrame,
                       config::kRightEndEffectorFrame),
      backend_(config_.model_xml_path, config_.control_dt_s,
               config_.physics_substeps)
{
    core_right_.emplace(kinematics_right_, config_.right_execution,
                        mailbox_right_, config_.control_dt_s);
    core_left_.emplace(kinematics_left_, config_.left_execution,
                       mailbox_left_, config_.control_dt_s);
}

SimArmFeedback DualSimulationRunner::Start()
{
    if (started_)
        throw std::logic_error(
            "DualSimulationRunner::Start twice without Reset — the cores' "
            "one-shot Seed would be repeated");
    // Seeded from the motion AT t = 0, not from the anchor: with a
    // nonzero phase the run starts displaced by A sin(phase), and
    // seeding the plant at the anchor would step the Mount by that much
    // between the seed and the first tick.
    feedback_ = backend_.Seed(PrescribedMount(config_, 0.0).world_T_mount);
    // CONTINUOUS degrees, never the wrapped wire: the backend's deg->rad
    // command conversion has no unwrap (by design), so the integrator
    // seed must be the same continuous value MuJoCo holds (home joint 4
    // is -130 deg; wrapped would read 230 deg and command through the
    // range limit on the first tick).
    core_right_->Seed(feedback_.right.continuous_position_deg,
                      feedback_.right.measured_velocity_deg_s);
    core_left_->Seed(feedback_.left.continuous_position_deg,
                     feedback_.left.measured_velocity_deg_s);
    tick_ = 0;
    started_ = true;
    stop_latched_ = false;
    return feedback_;
}

DualSimulationCycle DualSimulationRunner::Step()
{
    if (!started_)
        throw std::logic_error("DualSimulationRunner::Step before Start");
    if (stop_latched_)
        throw std::logic_error(
            "DualSimulationRunner::Step after the shared stop latched — a "
            "further tick would fabricate evidence from a run the stop "
            "ladder already condemned; Reset and Start to run again");

    DualSimulationCycle cycle;

    // One tick, one Mount state, shared by both arms; per-arm joint
    // measurements from the previous exchange reply (wrapped wire shape).
    // The pose that goes to the plant and the twist that goes to the
    // cores come from the same prescription, evaluated once here.
    const double dt_s = config_.control_dt_s; // exact 2 ms ticks
    const double t_s = static_cast<double>(tick_) * dt_s;
    const MountTruth mount = PrescribedMount(config_, t_s);
    const WorldSample world =
        IdealWorldSample(mount.world_T_mount, mount.world_V_mount,
                         static_cast<std::uint64_t>(tick_) + 1);

    ArmExecutionInput right_in;
    right_in.dt_s = dt_s;
    right_in.t_s = t_s;
    right_in.measured_position_deg = feedback_.right.measured_position_deg;
    right_in.measured_velocity_deg_s =
        feedback_.right.measured_velocity_deg_s;
    right_in.world = world;

    ArmExecutionInput left_in;
    left_in.dt_s = dt_s;
    left_in.t_s = t_s;
    left_in.measured_position_deg = feedback_.left.measured_position_deg;
    left_in.measured_velocity_deg_s = feedback_.left.measured_velocity_deg_s;
    left_in.world = world;

    // Both commands are computed before physics advances.
    cycle.right = core_right_->Step(right_in);
    cycle.left = core_left_->Step(left_in);

    // The transmit: one 0.002 s control tick of zero-order-held controls,
    // with the Mount held at the same pose this tick's WorldSample
    // reported — the same MountPrescription value, so plant and sample
    // cannot describe different Mounts. The twist is sensing only: the
    // plant carries the Mount as a world-welded mocap body with no
    // velocity of its own.
    cycle.next_feedback =
        backend_.Exchange(cycle.right.commanded_deg, cycle.left.commanded_deg,
                          mount.world_T_mount);

    // Send-then-resolve: the stop verdict ranks the reply to THIS cycle's
    // send. All three AdapterHealth facts are honestly false here — the
    // ideal adapter has no fault banks, servoing mode or acknowledgement
    // clock, and no fake health is synthesized.
    const AdapterHealth health{};
    cycle.right_stop = core_right_->ResolveStop(
        cycle.next_feedback.right.measured_position_deg, health);
    cycle.left_stop = core_left_->ResolveStop(
        cycle.next_feedback.left.measured_position_deg, health);
    cycle.shared_stop =
        StopRequested(cycle.right_stop) || StopRequested(cycle.left_stop);

    stop_latched_ = cycle.shared_stop;
    feedback_ = cycle.next_feedback;
    ++tick_;
    return cycle;
}

void DualSimulationRunner::Reset()
{
    // ArmExecutionCore::Seed is one-shot by contract: reset means
    // reconstruct. Re-emplacement destroys each core's reference source
    // (dropping any captured hold pose) and integrator state.
    core_right_.emplace(kinematics_right_, config_.right_execution,
                        mailbox_right_, config_.control_dt_s);
    core_left_.emplace(kinematics_left_, config_.left_execution,
                       mailbox_left_, config_.control_dt_s);
    feedback_ = SimArmFeedback{};
    tick_ = 0;
    started_ = false;
    stop_latched_ = false;
}
