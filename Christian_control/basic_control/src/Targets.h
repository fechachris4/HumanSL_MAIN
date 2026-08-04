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
// The fixed target's reference source: hands the compiled target (Config.h
// kFixedTargetM) to the controller every cycle, from the first cycle after
// takeover. The target is a full PoseTarget: its rotation commands an
// orientation, and leaving the rotation empty keeps the takeover
// orientation. One Get per cycle — pure computation: no I/O, no
// allocation, no blocking.
//
class PoseTargetSource
{
public:
    explicit PoseTargetSource(PoseTarget target);

    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status);

private:
    PoseTarget target_;
};
