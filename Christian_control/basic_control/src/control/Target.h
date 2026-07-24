//
// Target: the operator's desired end-effector position — parsing,
// latest-value store, and the stdin input thread. Never talks to the robot.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_TARGET_H
#define HUMANSL_MASTERS_PROJECT_2025_TARGET_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Dense>

// Parse one stdin line as a desired end-effector position: exactly 3 finite
// numbers — x y z in METERS, in the robot BASE frame. Returns the position,
// or std::nullopt with the reason in `error`. There is deliberately no
// workspace/reachability check in this version: an unreachable target makes
// the controller push toward it until something stops it (operator, fault).
std::optional<Eigen::Vector3d> ParseCartesianTarget(const std::string& line,
                                                    std::string& error);

// The single shared desired position: the input thread stores, the control
// loop snapshots. The mutex is held only to copy 3 doubles + a counter, so
// the loop's worst-case wait is bounded and tiny.
class TargetStore
{
public:
    struct Snapshot {
        Eigen::Vector3d p_desired;  // meters, base frame
        std::uint64_t sequence;     // increments on every Store — "is it new?"
    };

    void Store(const Eigen::Vector3d& p_desired);
    Snapshot Get() const;

private:
    mutable std::mutex mutex_;
    Eigen::Vector3d p_desired_{0.0, 0.0, 0.0};
    std::uint64_t sequence_ = 0;
};

// Thread body: wait for stdin lines, parse, store the latest valid desired
// position. Polls stdin with a timeout so it can observe `stop` (a blocking
// getline could not be interrupted portably). Prints accept/reject messages
// — this thread is not the control loop, terminal output is fine here.
void RunTargetInput(TargetStore& store, const std::atomic<bool>& stop);

// --- Pose targets (the reactive-pose controller) ---------------------------

// One parsed pose-target line. `rotation` is empty when the operator typed
// only a position — the stored orientation target is then kept as-is.
struct PoseTarget {
    Eigen::Vector3d p_desired;                // meters, base frame
    std::optional<Eigen::Matrix3d> rotation;  // base frame; nullopt = keep
};

// Parse one stdin line as a desired end-effector pose: exactly 3 finite
// numbers (x y z, METERS, base frame — orientation target unchanged) or
// exactly 6 (x y z roll pitch yaw, RADIANS, R = Rz(yaw)·Ry(pitch)·Rx(roll)
// — the simulation's convention). As with ParseCartesianTarget, there is
// deliberately no reachability check.
std::optional<PoseTarget> ParsePoseTarget(const std::string& line,
                                          std::string& error);

// The single shared desired pose, same pattern as TargetStore. StorePosition
// updates the position while keeping the stored orientation — both under one
// lock, so a concurrent snapshot never sees a torn pose.
class PoseTargetStore
{
public:
    struct Snapshot {
        Eigen::Vector3d p_desired;   // meters, base frame
        Eigen::Matrix3d rotation;    // desired orientation, base frame
        std::uint64_t sequence;      // increments on every store
    };

    void Store(const Eigen::Vector3d& p_desired, const Eigen::Matrix3d& rotation);
    void StorePosition(const Eigen::Vector3d& p_desired);
    Snapshot Get() const;

private:
    mutable std::mutex mutex_;
    Eigen::Vector3d p_desired_{0.0, 0.0, 0.0};
    Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
    std::uint64_t sequence_ = 0;
};

// Thread body for pose targets — same polling/stop behaviour as
// RunTargetInput.
void RunPoseTargetInput(PoseTargetStore& store, const std::atomic<bool>& stop);

#endif // HUMANSL_MASTERS_PROJECT_2025_TARGET_H
