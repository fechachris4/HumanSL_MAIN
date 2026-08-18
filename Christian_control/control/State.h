//
// State — the fixed records that cross module boundaries, and the contract
// that produces references. Eigen only: no Kortex, no Pinocchio.
//
// CartesianReference produces one world pose/twist; Controller maps its
// error to qdot; Runner sends that through the one shared safety/actuation
// path. GPMP2 joint states never cross this boundary.
//

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Measured state assembled once by Runner from arm feedback and the latest
// coherent Vicon sample. References are separate and never mutate this data.
struct RobotState {
    Eigen::Matrix<double, 7, 1> q_rad;      // measured joint positions
    Eigen::Matrix<double, 7, 1> qdot_rad_s; // measured joint velocities
    double t_s = 0.0;                       // time since takeover

    // World attachment: the Vicon
    // Mount segment's pose in Vicon-world, zero-order-held from the
    // 100 Hz stream, plus this cycle's trust verdict. The controller
    // composes world_T_base = world_T_mountseg · mount_T_base(model) via
    // Frames.h. The tracked segment is currently assumed to coincide with
    // the model Mount; a distinct MSeg_T_mount calibration is not represented.
    Eigen::Vector3d world_p_mountseg = Eigen::Vector3d::Zero();
    Eigen::Matrix3d world_R_mountseg = Eigen::Matrix3d::Identity();
    Eigen::Vector3d world_v_mountseg_m_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d world_w_mountseg_rad_s = Eigen::Vector3d::Zero();
    bool world_mount_twist_valid = false;
    std::uint64_t world_sequence = 0;
    bool world_fresh = false; // valid Mount + age <= kWorldFreshMaxAgeS
};

// The RAW Vicon world sample exactly as the adapter's wait-free slot read
// delivers it at cycle start, BEFORE any freshness classification — the
// input record of the execution core's world block (ExecutionCore.h). The
// classification itself (fresh verdict, ZOH pose freeze, stale-twist
// decay) writes the world_* fields of RobotState above; keeping the raw
// sample and the classified state as separate records is what lets the
// characterization fixture drive the classification instead of feeding it
// its own outputs. Pose in Vicon-world metres, quaternion x,y,z,w in the
// Vicon wire order; twist in world axes at the Mount segment point. NaN /
// sequence 0 mean "no sample has ever arrived".
struct WorldSample {
    bool mount_valid = false; // the Mount segment resolved this frame
    Eigen::Vector3d mount_position_m{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    double mount_quat_xyzw[4] = {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN()};
    std::uint64_t sequence = 0; // slot publish counter; 0 = never
    double age_s = std::numeric_limits<double>::quiet_NaN(); // at cycle start
    bool mount_twist_valid = false; // estimator produced a derivative
    Eigen::Vector3d mount_linear_world_m_s{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    Eigen::Vector3d mount_angular_world_rad_s{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
};

// Per-cycle telemetry the source and controller surface. Data only — the
// Runner decides what to print or log. NaN means "this cycle did not
// compute it" (e.g. takeover rows before Cartesian measurement).
struct ControllerStatus {
    Eigen::Vector3d p_desired = Eigen::Vector3d::Zero(); // current target
    Eigen::Vector3d p_current = Eigen::Vector3d::Zero(); // FK this cycle
    bool arrived_edge = false;    // first crossing under the tolerance
    double arrival_error_m = 0.0; // error norm at that crossing
    bool not_reached_edge = false; // set the cycle the non-arrival timeout fires
    double sigma_min = std::numeric_limits<double>::quiet_NaN();     // σ_min(J)
    double rot_error_rad = std::numeric_limits<double>::quiet_NaN(); // |log3|

    // The law's two terms BEFORE summation (rad/s), and the linear speed of
    // the leak twist J·q̇_null the damped projector lets into task space.
    // NaN default = "this cycle ran no law" (takeover hold). Added after the
    // 2026-08-05 stall, where these two terms silently cancelled at 218 mm
    // from the target and no printed or logged value could show it.
    Eigen::Matrix<double, 7, 1> qdot_task_rad_s =
        Eigen::Matrix<double, 7, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 7, 1> qdot_null_rad_s =
        Eigen::Matrix<double, 7, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    double null_leak_m_s = std::numeric_limits<double>::quiet_NaN();

    // World-Cartesian trajectory source evidence. Edges are true for one
    // cycle; IDs/time describe the reference returned on this cycle.
    bool cartesian_traj_activated = false;
    bool cartesian_traj_rejected = false;
    bool cartesian_traj_complete_edge = false;
    double cartesian_traj_start_position_error_m = 0.0;
    double cartesian_traj_start_orientation_error_rad = 0.0;
    int cartesian_traj_points = 0;
    double cartesian_traj_duration_s = 0.0;
    std::uint64_t cartesian_trajectory_id = 0;
    std::uint64_t planner_vicon_sequence = 0;
    double cartesian_reference_time_s = 0.0;
    bool request_replan_edge = false;
    bool cartesian_traj_cancelled_edge = false;

    // MEASURED tool orientation in Vicon world.
    // Hamilton convention, hemisphere-fixed to w >= 0 so logs never jump
    // sign. Telemetry only.
    Eigen::Quaterniond tool_quat{
        std::numeric_limits<double>::quiet_NaN(),  // w
        std::numeric_limits<double>::quiet_NaN(),  // x
        std::numeric_limits<double>::quiet_NaN(),  // y
        std::numeric_limits<double>::quiet_NaN()}; // z
};

// A Cartesian velocity — the reference twist a source commands, or a
// measured one. Mirrors controller/state.py Twist. Both halves default to
// zero, which is what Python spells Twist.zero(): a source that sets
// nothing commands a stationary target.
struct Twist {
    Eigen::Vector3d linear_m_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_rad_s = Eigen::Vector3d::Zero();
};

struct CartesianPose {
    Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

// One coherent measurement calculated once per control cycle from the latest
// Mount pose/twist and measured arm joints. Every quantity is expressed in
// Vicon world W; the controller law never performs another frame conversion.
struct MeasuredCartesianState {
    CartesianPose ee_pose_world;
    Eigen::Matrix<double, 6, 7> jacobian_world =
        Eigen::Matrix<double, 6, 7>::Zero();
    Twist ee_twist_world;
};

// The controller's only reference: desired end-effector pose and twist in
// Vicon world W. There is deliberately no frame selector, joint posture, or
// optional orientation at this boundary.
struct PoseReference {
    CartesianPose ee_pose_world;
    Twist ee_twist_world;
    std::uint64_t trajectory_id = 0;
    std::uint64_t planner_vicon_sequence = 0;
    double t_from_start_s = 0.0;
    // A profile may pass through its endpoint before it is permitted to
    // advance the target state machine.  Only a stationary terminal sample
    // is eligible to generate the controller's arrival edge.
    bool arrival_eligible = true;
};
