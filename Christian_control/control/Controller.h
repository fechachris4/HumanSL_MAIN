// Controller — the sole world-frame Cartesian pose/twist law.
//
// Measurement is explicit and happens once per cycle. DesiredVelocity then
// consumes only a WORLD PoseReference and maps reference-minus-measurement
// errors through the existing reactive DLS/null-space solver. It performs no
// frame selection, joint-reference tracking, I/O, command integration, or
// safety bypass.

#pragma once

#include <cstdint>
#include <memory>

#include <Eigen/Dense>

#include "Arrival.h"
#include "ReactiveLaw.h"
#include "State.h"

class DualArmKinematics;
struct KinematicsWorkspace;
struct ExecutionConfig;

class TrackingController
{
public:
    // Every gain, limit and tolerance is copied from the snapshot at
    // construction; no config:: value is read again at runtime.
    TrackingController(DualArmKinematics& model,
                       const ExecutionConfig& config);
    // Production convenience: identical to injecting
    // ProductionExecutionConfig() (the runner is rewired to explicit
    // injection in Plan 01 Task 4).
    explicit TrackingController(DualArmKinematics& model);
    ~TrackingController();

    // T_W_E, J_W and V_W_E from measured q/qdot and the latest Mount
    // pose/twist. No allocation, locks, or I/O.
    MeasuredCartesianState Measure(const RobotState& state);

    // Raw desired joint velocity before the shared clamp/integration/safety
    // path. Exact law:
    // V_task = Kp poseError(T_W_E,d,T_W_E) + Kd(V_W_E,d - V_W_E).
    Eigen::Matrix<double, 7, 1> DesiredVelocity(
        const RobotState& state,
        const MeasuredCartesianState& measured,
        const PoseReference& reference,
        double dt_s,
        ControllerStatus& status);

private:
    DualArmKinematics& model_;
    std::unique_ptr<KinematicsWorkspace> workspace_;
    ReactivePoseGains gains_;
    Eigen::Matrix<double, 7, 1> limit_rad_;
    double zone_rad_ = 0.0;
    // Immutable after construction (ExecutionConfig snapshot values).
    double arrival_position_tolerance_m_ = 0.0;
    double arrival_orientation_tolerance_rad_ = 0.0;
    double null_ramp_duration_s_ = 0.0;

    std::uint64_t last_trajectory_id_ = 0;
    bool trajectory_seen_ = false;
    bool arrival_reported_ = true;
    ArrivalSettlingMonitor arrival_monitor_;
    ArrivalTimeoutMonitor timeout_monitor_;
};
