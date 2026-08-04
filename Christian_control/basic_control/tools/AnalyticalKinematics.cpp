//
// AnalyticalKinematics — implementation. See AnalyticalKinematics.h.
//

#include "AnalyticalKinematics.h"

#include <cmath>

namespace
{
    // One URDF <joint> transform: a fixed origin (translation then
    // fixed-axis roll/pitch/yaw, URDF's own convention) followed by the
    // joint's own rotation about its local z-axis. Every Actuator joint in
    // the Gen3 chain has axis="0 0 1" in its own frame, so this single
    // helper covers all seven.
    Eigen::Isometry3d JointTransform(double origin_x, double origin_y,
                                     double origin_z, double roll,
                                     double pitch, double yaw,
                                     double joint_angle_rad)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = Eigen::Vector3d(origin_x, origin_y, origin_z);
        transform.linear() =
            (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        return transform * Eigen::AngleAxisd(joint_angle_rad,
                                             Eigen::Vector3d::UnitZ());
    }
} // namespace

Eigen::Isometry3d AnalyticalForwardKinematics(
    const Eigen::Matrix<double, 7, 1>& q_rad)
{
    // Link offsets and fixed origin rotations, copied VERBATIM (not
    // "cleaned up" to exact pi/2) from config/GEN3_dual_mounted.urdf's
    // right-arm joints (Actuator1..7, then the fixed EndEffector joint) —
    // see the header comment for the upstream cross-check that verified
    // these numbers. The URDF itself stores roll as the truncated decimal
    // 1.5708 rather than exact pi/2 (Kinova's own export precision), and
    // Pinocchio parses that literal string; using idealized pi/2 here
    // instead would silently diverge from both the model and the
    // factory-calibrated real robot by ~3.6e-6 rad per joint.
    const Eigen::Isometry3d t1 =
        JointTransform(0, 0, 0.15643, 3.1416, 0, 0, q_rad[0]);
    const Eigen::Isometry3d t2 =
        JointTransform(0, 0.005375, -0.12838, 1.5708, 0, 0, q_rad[1]);
    const Eigen::Isometry3d t3 =
        JointTransform(0, -0.21038, -0.006375, -1.5708, 0, 0, q_rad[2]);
    const Eigen::Isometry3d t4 =
        JointTransform(0, 0.006375, -0.21038, 1.5708, 0, 0, q_rad[3]);
    const Eigen::Isometry3d t5 =
        JointTransform(0, -0.20843, -0.006375, -1.5708, 0, 0, q_rad[4]);
    const Eigen::Isometry3d t6 =
        JointTransform(0, 0.00017505, -0.10593, 1.5708, 0, 0, q_rad[5]);
    const Eigen::Isometry3d t7 =
        JointTransform(0, -0.10593, -0.00017505, -1.5708, 0, 0, q_rad[6]);

    // EndEffector: fixed joint (no joint-angle rotation). This one URDF
    // entry uses full double precision for roll, unlike the Actuator
    // joints above — copied as written.
    const Eigen::Isometry3d t_flange =
        JointTransform(0, 0, -0.0615250000000001, 3.14159265358979, 0, 0,
                       0.0);

    // ConfiguredTool: the mounted tool's offset past the bare flange, read
    // from the robot's own ControlConfig::GetToolConfiguration on
    // 2026-08-05 (see the URDF comment on ConfiguredTool_Link for details
    // and the re-read-if-the-tool-changes caveat).
    const Eigen::Isometry3d t_configured_tool =
        JointTransform(0, 0, 0.12, 0, 0, 0, 0.0);

    return t1 * t2 * t3 * t4 * t5 * t6 * t7 * t_flange * t_configured_tool;
}
