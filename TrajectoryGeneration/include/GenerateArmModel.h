#pragma once

#include "utils.h"
#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>


class ArmModel{
    private:

        // Collision spheres for the gpmp2 arm. Along-link sphere stations
        // scale with the DH d lengths in `d` (generated from the URDF), so
        // a URDF geometry change moves the collision model with the links
        // instead of leaving it at stale absolute offsets. Lateral (girth)
        // offsets stay absolute. Throws if a link length's sign flips or
        // collapses — that means the frame geometry changed and the sphere
        // layout must be re-derived by hand, not silently rescaled.
        gpmp2::BodySphereVector generateArmSpheres(
            size_t arm_id_offset,
            double sphere_radius,
            const gtsam::Vector& d);


    public:
        ArmModel();

        std::unique_ptr<gpmp2::ArmModel> createArmModel(
            const gtsam::Pose3& base_pose,
            const DHParameters& dh_params);
};



