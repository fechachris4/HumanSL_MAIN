//
// Targets — desired positions, their SPSC mailbox, and the reference source.
// Never talks to the robot; does no control math.
//

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Config.h"
#include "State.h"
#include "TrajectoryProfile.h"

// One base_link target. A parsed 7-field line carries orientation, but it
// is only reachable while config::kAcceptOrientationTargets is true — see
// ParsePoseTarget; PoseTargetSource still clears it every activation.
struct PoseTarget {
    Eigen::Vector3d p_desired;               // metres, right-arm base frame
    std::optional<Eigen::Matrix3d> rotation; // set only when the 7-field
                                              // grammar parses; see above
};

// Rotation matrix from roll/pitch/yaw RADIANS, composed R = Rz·Ry·Rx.
Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw);

// Parse public runtime input: either "x y z" (three finite metre
// coordinates, base_link) or "x y z qx qy qz qw" (adds a unit quaternion,
// xyzw). The 7-field form is rejected with an error naming
// config::kAcceptOrientationTargets while that gate is false — orientation
// is never silently dropped.
std::optional<PoseTarget> ParsePoseTarget(const std::string& line,
                                          std::string& error);

// Exactly eight entries, with the stdin thread as sole producer and the
// 500 Hz source as sole consumer. Release/acquire publication means the
// source never locks, allocates, or performs I/O in Get.
class PoseTargetMailbox
{
public:
    static constexpr std::size_t kCapacity = 8;

    bool Enqueue(const PoseTarget& target);
    std::optional<PoseTarget> TryDequeue();

private:
    std::array<PoseTarget, kCapacity> entries_{};
    alignas(64) std::atomic<std::uint64_t> read_index_{0};
    alignas(64) std::atomic<std::uint64_t> write_index_{0};
};

// Declared, not included: TrajectoryGeneration/include/utils.h defines a
// different global struct of this name, and planner_bridge translation units
// include both that and this header. Callers that touch the trajectory's
// contents include JointTrajectory.h themselves.
struct JointTrajectory;

// Single-slot, latest-wins handoff for whole joint trajectories. The input
// thread is the only producer and validates every block BEFORE publishing,
// so the RT consumer only ever sees a valid trajectory. Publish deletes the
// block it displaces; Take is one atomic exchange, and the caller owns what
// it receives — the one bounded free per plan activation is the accepted RT
// exception recorded in the plan header.
class JointTrajectoryMailbox
{
public:
    JointTrajectoryMailbox() = default;
    ~JointTrajectoryMailbox();
    JointTrajectoryMailbox(const JointTrajectoryMailbox&) = delete;
    JointTrajectoryMailbox& operator=(const JointTrajectoryMailbox&) = delete;

    void Publish(std::unique_ptr<JointTrajectory> traj); // input thread
    std::unique_ptr<JointTrajectory> Take();             // RT thread; may be null

private:
    std::atomic<JointTrajectory*> slot_{nullptr};
};

// Reads target lines from a named pipe, surviving writer disconnects:
// on EOF the pipe is reopened, so each bridge invocation may open, write,
// and close independently. `stop` is the only exit. The fd-level loop is
// the tested RunTargetInput, unchanged.
void RunTargetInputFromPipe(PoseTargetMailbox& pose_mailbox,
                            JointTrajectoryMailbox& traj_mailbox,
                            const std::atomic<bool>& stop,
                            const std::string& pipe_path);

// Same input loop over a borrowed POSIX file descriptor. Kept separate so the
// pipe entry point stays simple and the partial-pipe teardown contract is
// testable without robot dependencies.
//
// Line routing: while the accumulator is collecting, every line is a
// trajectory line; while it is idle, a TRAJ_* keyword opens a block and
// anything else is a pose line. A completed block is validated against the
// compiled joint limits and only then published. Any grammar or validation
// error goes to stderr with the offending line and resets the accumulator —
// nothing is silently skipped, and the thread survives.
void RunTargetInput(PoseTargetMailbox& pose_mailbox,
                    JointTrajectoryMailbox& traj_mailbox,
                    const std::atomic<bool>& stop, int input_fd);

// Pose-only entry points, kept for callers that predate joint trajectories.
// They own a throwaway trajectory mailbox, so a block arriving on such a
// stream is validated and then dropped rather than reaching any consumer.
void RunPoseTargetInputFromPipe(PoseTargetMailbox& mailbox,
                                const std::atomic<bool>& stop,
                                const std::string& pipe_path);

void RunPoseTargetInputFromFd(PoseTargetMailbox& mailbox,
                              const std::atomic<bool>& stop, int input_fd);

// Begins with a profile from the measured takeover position to the compiled
// terminal target (sequence 0). It holds the active target until Runner
// reports an arrival edge; the next Get consumes exactly one queued target, if
// any, and increments the sequence. If empty, it stays ready for a target
// later enqueued by the producer.
class PoseTargetSource
{
public:
    PoseTargetSource(
        Eigen::Vector3d startup_position_m, PoseTarget terminal_target,
        PoseTargetMailbox& mailbox,
        CartesianMotionLimits profile_limits = {
            config::kProfileMaxSpeedMps,
            config::kProfileMaxAccelerationMps2,
            config::kProfileMaxJerkMps3},
        double target_hold_s = config::kTargetHoldS);

    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status);
    void OnArrived();

private:
    enum class Phase {
        kAwaitTerminalArrival,
        kDwell,
        kReadyForNext,
        kFollowingProfile,
    };

    bool ValidDt(double dt_s) const;
    void ActivateOneQueuedTarget();
    Reference StationaryReference() const;

    PoseTargetMailbox& mailbox_;
    PoseTarget active_target_;
    CartesianMotionLimits profile_limits_;
    double target_hold_s_ = 0.0;
    std::uint64_t sequence_ = 0;
    Phase phase_ = Phase::kAwaitTerminalArrival;
    double dwell_elapsed_s_ = 0.0;
    double profile_elapsed_s_ = 0.0;
    std::optional<CartesianSegmentProfile> active_profile_;
};

// Follows whole joint trajectories published by the input thread.
//
// Startup holds the measured takeover q at zero velocity. Each RT cycle it
// Takes any newly published trajectory: if the first point is within
// config::kTrajStartToleranceDeg of the measured q on EVERY joint it is
// activated with its clock at zero, otherwise it is deleted, the rejection
// is reported in ControllerStatus, and the current reference keeps holding —
// a trajectory that fails the splice guard is never partially followed.
// While active, Get returns the Hermite sample; past the end it holds the
// final point and reports arrival once.
//
// Never blocks, never does I/O and never allocates: Take is one atomic
// exchange, and the bounded delete of a rejected or displaced trajectory is
// the same accepted RT exception as the mailbox itself.
//
// CALLER CONTRACT: `dt_s` must be the Runner's already-clamped cycle time
// (ClampedCycleDt). Non-positive and non-finite values are ignored here, but
// a large positive dt is trusted and would jump the sample clock — the loop's
// single dt clamp is what bounds it, and duplicating that bound here would
// mean two places deciding what a cycle may last.
class JointTrajectorySource
{
public:
    JointTrajectorySource(Eigen::Matrix<double, 7, 1> hold_q_rad,
                          JointTrajectoryMailbox& mailbox);
    ~JointTrajectorySource();
    JointTrajectorySource(const JointTrajectorySource&) = delete;
    JointTrajectorySource& operator=(const JointTrajectorySource&) = delete;

    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status);

private:
    JointTrajectoryMailbox& mailbox_;
    Eigen::Matrix<double, 7, 1> hold_q_rad_;
    // Incomplete here on purpose (see the forward declaration above), so the
    // destructor is defined in the .cpp.
    std::unique_ptr<JointTrajectory> active_;
    double elapsed_s_ = 0.0;
    bool complete_reported_ = false;
};

// The hardware loop calls this bridge with its per-cycle status. Keeping the
// edge gate in pure target code gives hardware-free coverage of the exact
// arrival-to-queue handoff contract.
void NotifyPoseTargetSourceOnArrivalEdge(PoseTargetSource& source,
                                         const ControllerStatus& status);
