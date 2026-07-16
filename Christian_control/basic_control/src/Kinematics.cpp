//
// Kinematics: forward kinematics via Pinocchio (model already loaded in Dynamics).
//

#include "Kinematics.h"

#include "Measure.h"
#include "Fwd_kinematics.h"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>

Pose forward_kinematics(Dynamics& dynamics, const Eigen::VectorXd& q_pin,
                        const std::string& frame_name)
{
    // Use the manual forward kinematics from TrajectoryExecution (Fwd_kinematics.cpp).
    // Note: this implementation specifically targets the end effector and ignores frame_name.
    Eigen::VectorXd q = dynamics.convertConfigToJointAngles(q_pin);
    auto [p, T] = Fwd_kinematics::forward(q);

    return Pose{T.block<3, 1>(0, 3), T.block<3, 3>(0, 0)};
}

void report_fk_vs_robot(Dynamics& dynamics,
                        k_api::Base::BaseClient* base,
                        k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                        std::ostream& out)
{
    JointReading joints = measure_configuration(base_cyclic, dynamics);
    Pose ee = forward_kinematics(dynamics, joints.q_pin);

    // Fwd_kinematics::forward already includes a tool offset (T_7end).
    // The robot reports its pose at the tool center point (TCP).
    Eigen::Vector3d tcp = ee.position; 

    auto kinova_pose = base->GetMeasuredCartesianPose();
    out << "FK EE (Manual):         x=" << ee.position.x()
        << "  y=" << ee.position.y() << "  z=" << ee.position.z() << "  [m]\n"
        << "Kinova reports (TCP):   x=" << kinova_pose.x()
        << "  y=" << kinova_pose.y() << "  z=" << kinova_pose.z() << "  [m]\n";
}
