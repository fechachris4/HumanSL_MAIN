//
// Targets — desired positions, their SPSC mailbox, and the reference source.
// Never talks to the robot; does no control math.
//

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
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

// Reads target lines from a named pipe, surviving writer disconnects:
// on EOF the pipe is reopened, so each bridge invocation may open, write,
// and close independently. `stop` is the only exit. The fd-level loop is
// the tested RunPoseTargetInputFromFd, unchanged.
void RunPoseTargetInputFromPipe(PoseTargetMailbox& mailbox,
                                const std::atomic<bool>& stop,
                                const std::string& pipe_path);

// Same input loop over a borrowed POSIX file descriptor. Kept separate so the
// pipe entry point stays simple and the partial-pipe teardown contract is
// testable without robot dependencies.
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

// The hardware loop calls this bridge with its per-cycle status. Keeping the
// edge gate in pure target code gives hardware-free coverage of the exact
// arrival-to-queue handoff contract.
void NotifyPoseTargetSourceOnArrivalEdge(PoseTargetSource& source,
                                         const ControllerStatus& status);
