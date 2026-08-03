//
// Controller-facing robot state. Eigen only — no Kortex, no vendor types,
// ever.
//
// HARD RULE (Christian, 2026-07-22): a field belongs in RobotState only if
// the Runner can fill it validly EVERY cycle from arm feedback alone.
// External sensing (Vicon, anything future) never goes here — it reaches a
// controller by store injection: a mutex-protected latest-value store with
// a timestamp, written by its own thread, passed by const reference to
// only the controllers that need it (TargetStore is the pattern to copy).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H
#define HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H

#include <limits>

#include <Eigen/Dense>

struct RobotState {
    Eigen::Matrix<double, 7, 1> q_rad;      // measured joint positions
    Eigen::Matrix<double, 7, 1> qdot_rad_s; // measured joint velocities
    double t_s = 0.0;                       // time since takeover
};

// Per-cycle status a controller surfaces for telemetry and operator UX —
// data only; the Runner decides what to print or log (controllers do no
// I/O). The Cartesian fields feed the log's p_desired/p_current columns;
// a future joint-space controller leaves them at their defaults.
struct ControllerStatus {
    Eigen::Vector3d p_desired = Eigen::Vector3d::Zero(); // current target
    Eigen::Vector3d p_current = Eigen::Vector3d::Zero(); // FK this cycle
    bool arrived_edge = false;    // first crossing under the tolerance
    double arrival_error_m = 0.0; // error norm at that crossing
    // Smallest singular value of the task Jacobian (decision 13) — the
    // proximity-to-singularity signal, logged every cycle. NaN when the
    // law has no task Jacobian.
    double sigma_min = std::numeric_limits<double>::quiet_NaN();
    // Norm of the rotation-log error, rad — orientation convergence
    // evidence, logged every cycle. NaN when the law does not control
    // orientation (e.g. ResolvedRate).
    double rot_error_rad = std::numeric_limits<double>::quiet_NaN();
    // MEASURED right-tool orientation from this cycle's dual-model FK:
    // flange frame (no TCP offset), expressed in the right-arm base frame.
    // Hamilton convention, unit norm, hemisphere-fixed to
    // w >= 0 so logged trajectories never jump sign. Telemetry only; NaN
    // coefficients when the law exposes no tool frame.
    Eigen::Quaterniond tool_quat{
        std::numeric_limits<double>::quiet_NaN(),  // w
        std::numeric_limits<double>::quiet_NaN(),  // x
        std::numeric_limits<double>::quiet_NaN(),  // y
        std::numeric_limits<double>::quiet_NaN()}; // z

    // Trajectory playback telemetry (control/TrajectoryPlayback.h). The
    // reference is what this cycle's integrated command should land on —
    // logged next to commanded_deg and measured_deg so planned, commanded
    // and measured motion stay comparable per cycle. Other laws leave the
    // defaults (NaN reference / state 0).
    Eigen::Matrix<double, 7, 1> q_ref_deg =
        Eigen::Matrix<double, 7, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    double playback_t_s = std::numeric_limits<double>::quiet_NaN();
    int playback_state = 0;   // 0 none, 1 playing, 2 done-holding, 3 refused
    bool playback_done_edge = false;    // first cycle past the last sample
    bool playback_refused_edge = false; // first cycle after a Reset refusal
};

// The controller: one Reset at takeover, then one DesiredVelocity per
// cycle. Contract: pure computation — no Kortex types (none are reachable
// from this header), no servoing or configuration changes, no printing, no
// file I/O, no allocation, no blocking beyond a bounded store lock
// (TargetStore is the pattern).
class Controller
{
public:
    virtual ~Controller() = default;

    // T5 of the takeover sequence (loop/Runner.h): called once, after
    // Actuation::Prepare and before the first DesiredVelocity; must leave
    // the controller commanding "hold here".
    virtual void Reset(const RobotState& state) = 0;

    // Desired joint velocity BEFORE clamping, rad/s. dt_s is the Runner's
    // measured, clamped cycle time (an input; a control law may ignore it).
    virtual Eigen::Matrix<double, 7, 1>
    DesiredVelocity(const RobotState& state, double dt_s,
                    ControllerStatus& status) = 0;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H
