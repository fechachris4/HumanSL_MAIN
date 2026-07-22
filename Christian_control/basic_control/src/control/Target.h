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

#endif // HUMANSL_MASTERS_PROJECT_2025_TARGET_H
