//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//

#include "Kinematics.h"

#include "Measure.h"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>

Pose forward_kinematics(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                        const std::string& frame_name)
{
    if (!dynamics.model_.existFrame(frame_name))
        throw std::runtime_error("forward_kinematics: no frame named '" + frame_name
                                 + "' in the model (check the URDF link names)");

    // Walk the kinematic tree: place every joint for configuration q_pin,
    // then update the frames attached to them.
    pinocchio::framesForwardKinematics(dynamics.model_, dynamics.data_, q_pin);

    // oMf = "origin-to-frame" transform: the frame's pose in the base frame.
    const auto frame_id = dynamics.model_.getFrameId(frame_name);
    const pinocchio::SE3& oMf = dynamics.data_.oMf[frame_id];

    return Pose{oMf.translation(), oMf.rotation()};
}

void report_fk_vs_robot(Dynamics& dynamics,
                        k_api::Base::BaseClient* base,
                        k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                        std::ostream& out)
{
    JointReading joints = measure_configuration(base_cyclic, dynamics);
    Pose ee = forward_kinematics(dynamics, joints.q_pin);

    // The robot reports its pose at the tool center point (TCP), which for
    // the 2F-85 gripper sits ~0.12 m past the flange along the tool z-axis.
    // The URDF has no TCP frame, so add that offset to compare like-for-like.
    Eigen::Vector3d tcp = ee.position + ee.rotation * Eigen::Vector3d(0, 0, 0.12);

    auto kinova_pose = base->GetMeasuredCartesianPose();
    out << "FK flange (Pinocchio):  x=" << ee.position.x()
        << "  y=" << ee.position.y() << "  z=" << ee.position.z() << "  [m]\n"
        << "FK + 0.12m TCP:         x=" << tcp.x()
        << "  y=" << tcp.y() << "  z=" << tcp.z() << "  [m]\n"
        << "Kinova reports (TCP):   x=" << kinova_pose.x()
        << "  y=" << kinova_pose.y() << "  z=" << kinova_pose.z() << "  [m]\n";
}
