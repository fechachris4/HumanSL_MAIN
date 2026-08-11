//
// Kinematics — the Pinocchio robot model: frame FK and Jacobians, and the
// explicit 14-DoF-model / 7-joint-controller dual-arm adapter. The
// controller that uses this model lives in Controller.h/.cpp.
//
// Separated from the pure-Eigen headers so the portable tests need not link
// Pinocchio.
//

#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "Config.h"
#include "State.h"
#include "Dynamics.h" // Pinocchio model wrapper (our copy of TrajectoryExecution's)

// ---------------------------------------------------------------
// Kinematics — generic Pinocchio FK and Jacobian
// ---------------------------------------------------------------

//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//




// Pose of one frame of the robot, expressed in the Pinocchio model root frame.
struct Pose {
    Eigen::Vector3d position; // meters
    Eigen::Matrix3d rotation; // orientation as a rotation matrix
};

// Forward kinematics: where is `frame_name` when the joints are at q_pin?
// q_pin is the Pinocchio configuration vector (from measure_configuration).
// Frame names come from the URDF, e.g. "EndEffector_Link", "gripper_link",
// "Bracelet_Link".
Pose forward_kinematics(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                        const std::string& frame_name = "EndEffector_Link");

// Preallocated workspace for the per-cycle kinematics: the full 6×nv frame
// Jacobian lives here so the cyclic loop never allocates. Construct once,
// outside the loop, from the model that will be queried.
struct KinematicsWorkspace {
    explicit KinematicsWorkspace(const Dynamics& dynamics)
        : jacobian_full(6, dynamics.model_.nv)
    {
        jacobian_full.setZero(); // getFrameJacobian only writes nonzeros
    }
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_full;
};

// Cartesian pose and translational Jacobian result. Position, rotation, and
// Jacobian rows must use the same declared frame; DualArmKinematics fills
// this in base_link axes. Pose and Jacobian must describe the same
// configuration q_pin.
struct PositionJacobian {
    Eigen::Vector3d position;                 // meters, declared Cartesian frame
    Eigen::Matrix3d rotation;                 // orientation in that frame
    Eigen::Matrix<double, 3, 7> jacobian_p;   // rows: x,y,z; cols: joints 1-7
};

// Full-pose counterpart of PositionJacobian. All six Jacobian rows
// ([linear; angular]) use the same declared axes as position and rotation.
struct PoseJacobian {
    Eigen::Vector3d position;                // meters, declared Cartesian frame
    Eigen::Matrix3d rotation;                // orientation in that frame
    Eigen::Matrix<double, 6, 7> jacobian;    // [linear; angular] x joints 1-7
};

// ---------------------------------------------------------------
// DualArmKinematics — the 14-model/7-controller adapter
// ---------------------------------------------------------------

//
// DualArmKinematics: explicit 14-DoF model / 7-controller adapter.
//
// The Pinocchio model contains both mounted Gen3 arms (14 velocity DoFs).
// Kinova's continuous joints 1/3/5/7 each use Pinocchio's two-value
// (cos(angle), sin(angle)) configuration representation, so model.nq is 22.
// The controller and Kortex command path remain seven-wide, for exactly one
// CONTROLLED arm, chosen at construction and fixed for the adapter's life —
// one basic_control process drives one arm (Main.cpp's RunOneArm; a
// --arm=both run is two processes, each with its own adapter). This adapter
// assembles q_full = [controlled measured, other nominal] by JOINT NAME,
// computes the full mounted-model kinematics, expresses the controlled
// arm's tool pose and Jacobian in ITS base frame, and selects its seven
// Jacobian columns in Kortex actuator order. The other (uncontrolled) arm
// has no connection in this process; its slots are seeded once at
// construction from other_arm_nominal_rad and never updated, so its
// reported FK is model-only, not measured — see config::ArmConfig.
//




// Which arm a mount-frame query refers to. The controller still commands
// only the right arm; this selects which branch of the model to READ.
enum class Arm { kRight, kLeft };

class DualArmKinematics
{
public:
    static constexpr int kArmDofs = 7;
    static constexpr int kFullDofs = 14;
    static constexpr int kFullConfigurationSize = 22;
    static constexpr std::array<int, 7> kJointConfigurationSizes{
        2, 1, 2, 1, 2, 1, 2
    };

    // other_arm_nominal_rad: the UNCONTROLLED arm's assumed pose
    // (config::ArmConfig::other_arm_nominal_rad — kLeftNominalRad when
    // controlled_arm is kRight, kRightNominalRad when it is kLeft).
    DualArmKinematics(
        Dynamics& dynamics, Arm controlled_arm,
        const JointVector& other_arm_nominal_rad,
        const std::string& right_base_frame,
        const std::string& right_end_effector_frame,
        const std::string& left_base_frame = config::kLeftBaseFrame,
        const std::string& left_end_effector_frame = config::kLeftEndEffectorFrame);

    Arm controlled_arm() const { return controlled_arm_; }
    const JointVector& other_arm_nominal_rad() const { return other_arm_nominal_rad_; }

    // Pose and Jacobian of the CONTROLLED arm's tool, in ITS OWN base frame,
    // at the given measured configuration. The per-cycle hot path
    // (Controller.cpp) — the only kinematics call in the control loop.
    PositionJacobian ControlledPositionAndJacobian(
        const Eigen::Matrix<double, 7, 1>& controlled_q_rad,
        KinematicsWorkspace& workspace);
    PoseJacobian ControlledPoseAndJacobian(
        const Eigen::Matrix<double, 7, 1>& controlled_q_rad,
        KinematicsWorkspace& workspace);

    // ---------------------------------------------------------------
    // World frame
    // ---------------------------------------------------------------
    //
    // `mount` is the URDF root, so Pinocchio's oMf placements ARE mount-frame
    // poses; nothing extra is computed to reach mount. The base-frame methods
    // above are the ones doing extra work, converting INTO base_link, and they
    // remain the controller's interface — see docs/decisions/custom-urdf.md.

    // T_mount_base for either arm. Constant: both mounts are fixed joints, so
    // this does not depend on the configuration. Cached at construction.
    const pinocchio::SE3& MountFromBase(Arm arm) const;

    // The legacy-point conversion: p_mount = T_mount_base * p_base, and its
    // inverse. Use these to lift anything still expressed in an arm's
    // base_link frame (recorded targets, older logs) into mount, or to push a
    // mount-frame target down into the frame the control loop still uses.
    Eigen::Vector3d PointBaseToMount(Arm arm, const Eigen::Vector3d& p_base) const;
    Eigen::Vector3d PointMountToBase(Arm arm, const Eigen::Vector3d& p_mount) const;

    // Tool pose of one arm in the WORLD frame, at a configuration given for
    // BOTH arms. Both are required because one Pinocchio model holds both
    // branches; passing both angles explicitly keeps the caller honest about
    // what the uncontrolled arm was assumed to be doing.
    Pose ToolPoseInMount(Arm arm,
                         const Eigen::Matrix<double, 7, 1>& right_q_rad,
                         const Eigen::Matrix<double, 7, 1>& left_q_rad);

    // Same, with the CONTROLLED arm at controlled_q_rad and the other arm at
    // its compiled other_arm_nominal_rad — the live case, where the other
    // arm is model-only and has no feedback to report.
    Pose ToolPoseInMount(Arm arm, const Eigen::Matrix<double, 7, 1>& controlled_q_rad);

    // Exposed for hardware-free structural tests. The returned reference is
    // owned by this adapter and is overwritten by the next call.
    const Eigen::VectorXd& FullConfigurationForControlled(
        const Eigen::Matrix<double, 7, 1>& controlled_q_rad);

    // Both arms' angles into the full q, by joint name. Same ownership rule.
    const Eigen::VectorXd& FullConfiguration(
        const Eigen::Matrix<double, 7, 1>& right_q_rad,
        const Eigen::Matrix<double, 7, 1>& left_q_rad);

    Dynamics& dynamics() { return dynamics_; }
    pinocchio::FrameIndex right_base_frame_id() const { return right_base_frame_id_; }
    pinocchio::FrameIndex right_frame_id() const { return right_frame_id_; }
    const std::array<int, 7>& right_q_indices() const { return right_q_indices_; }
    const std::array<int, 7>& right_v_indices() const { return right_v_indices_; }
    const std::array<int, 7>& left_q_indices() const { return left_q_indices_; }
    const std::array<int, 7>& left_v_indices() const { return left_v_indices_; }

private:
    void UpdateFullKinematics(const Eigen::Matrix<double, 7, 1>& controlled_q_rad,
                              KinematicsWorkspace& workspace);

    Dynamics& dynamics_;
    Arm controlled_arm_;
    pinocchio::FrameIndex right_base_frame_id_;
    pinocchio::FrameIndex right_frame_id_;
    pinocchio::FrameIndex left_base_frame_id_;
    pinocchio::FrameIndex left_frame_id_;
    pinocchio::SE3 mount_from_right_base_;
    pinocchio::SE3 mount_from_left_base_;
    std::array<int, 7> right_q_indices_{};
    std::array<int, 7> right_v_indices_{};
    std::array<int, 7> left_q_indices_{};
    std::array<int, 7> left_v_indices_{};
    JointVector other_arm_nominal_rad_;
    Eigen::VectorXd q_full_;
};
