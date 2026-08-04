//
// Targets — the operator's desired pose: line parsing, the shared store,
// the stdin and file-watch input threads, and the reference source that
// serves them. Never talks to the robot; does no control math.
//

#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "State.h"

// One parsed target line. `rotation` is empty when only a position was
// given — the orientation target is then left as it was.
struct PoseTarget {
    Eigen::Vector3d p_desired;               // meters, right-arm base frame
    std::optional<Eigen::Matrix3d> rotation; // base frame; nullopt = keep
};

// Rotation matrix from roll/pitch/yaw RADIANS, composed R = Rz·Ry·Rx —
// the one place that convention is written down. Mirrors the simulation's
// controller/transforms.py rotation_from_rpy, so a compiled target and a
// typed line and the Python sim all mean the same thing by an rpy triple.
Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw);

// Parse one line: exactly 3 finite numbers (x y z METERS, right-arm base
// frame — orientation unchanged) or 6 (x y z + roll pitch yaw RADIANS,
// R = Rz(yaw)·Ry(pitch)·Rx(roll), the simulation's convention). There is
// deliberately no reachability check: an unreachable target makes the
// controller push toward it until something stops it (operator, fault).
std::optional<PoseTarget> ParsePoseTarget(const std::string& line,
                                          std::string& error);

// The shared desired pose: input threads store, the source snapshots. The
// mutex is held only for the copy, so the loop's worst-case wait is tiny.
// The rotation stays empty until an operator supplies one — a
// position-only target never invents an orientation.
class PoseTargetStore
{
public:
    struct Snapshot {
        Eigen::Vector3d p_desired;               // meters, base frame
        std::optional<Eigen::Matrix3d> rotation; // empty until one is stored
        std::uint64_t sequence;                  // increments on every store
    };

    void Store(const Eigen::Vector3d& p_desired, const Eigen::Matrix3d& rotation);
    void StorePosition(const Eigen::Vector3d& p_desired);
    Snapshot Get() const;

private:
    mutable std::mutex mutex_;
    Eigen::Vector3d p_desired_{0.0, 0.0, 0.0};
    std::optional<Eigen::Matrix3d> rotation_;
    std::uint64_t sequence_ = 0;
};

// Thread body: read stdin lines, parse, store the latest valid pose. Polls
// with a timeout so it can observe `stop` (a blocking getline could not be
// interrupted portably). Prints accept/reject — not the control loop, so
// terminal output is fine here.
void RunPoseTargetInput(PoseTargetStore& store, const std::atomic<bool>& stop);

//
// The operator's targets as a ReferenceSource. Reset snapshots the store's
// sequence, so anything stored BEFORE takeover is discarded — except
// `initial_target` (Config.h kUseFixedTarget), which Main passes
// deliberately and which applies from the first cycle. That target is a
// full PoseTarget: its rotation commands an orientation, and leaving the
// rotation empty keeps the takeover orientation. Until any target exists
// the source returns an empty reference and the controller holds where
// takeover happened.
//
class PoseTargetSource : public ReferenceSource
{
public:
    explicit PoseTargetSource(
        const PoseTargetStore& store,
        std::optional<PoseTarget> initial_target = std::nullopt);

    void Reset(const RobotState& state) override;
    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status) override;

private:
    const PoseTargetStore& store_;
    std::optional<PoseTarget> initial_target_;
    std::uint64_t baseline_sequence_ = 0;
};
