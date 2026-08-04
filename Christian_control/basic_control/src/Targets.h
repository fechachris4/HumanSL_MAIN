//
// Targets — the desired pose and the reference source that serves it.
// Never talks to the robot; does no control math.
//

#pragma once

#include <optional>

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
// controller/transforms.py rotation_from_rpy, so a compiled target and
// the Python sim mean the same thing by an rpy triple.
Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw);

//
// The fixed target as a ReferenceSource. `initial_target` (Config.h
// kFixedTargetM), which Main passes
// deliberately, applies from the first cycle. That target is a
// full PoseTarget: its rotation commands an orientation, and leaving the
// rotation empty keeps the takeover orientation. Until any target exists
// the source returns an empty reference and the controller holds where
// takeover happened.
//
class PoseTargetSource : public ReferenceSource
{
public:
    explicit PoseTargetSource(
        std::optional<PoseTarget> initial_target = std::nullopt);

    void Reset(const RobotState& state) override;
    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status) override;

private:
    std::optional<PoseTarget> initial_target_;
};
