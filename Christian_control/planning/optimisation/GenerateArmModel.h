#pragma once

#include "utils.h"
#include "../src/StaticScene.h"
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
        //
        // has_tool: joint 6's authored stations (see the .cpp) were hand
        // fitted to the RIGHT arm's mounted tool/gripper — d7 there is the
        // wrist-to-tool distance, and the finger-spread spheres model the
        // gripper's physical footprint. The left arm ends at a bare flange
        // (no gripper), so has_tool = false drops those finger spheres
        // rather than scaling geometry that is not physically present;
        // the wrist-to-flange link itself (the non-finger joint-6 spheres)
        // is kept, scaled to the flange's own (shorter) d7.
        gpmp2::BodySphereVector generateArmSpheres(
            size_t arm_id_offset,
            double sphere_radius,
            const gtsam::Vector& d,
            bool has_tool);


    public:
        ArmModel();

        std::unique_ptr<gpmp2::ArmModel> createArmModel(
            const gtsam::Pose3& base_pose,
            const DHParameters& dh_params,
            bool has_tool = true,
            std::vector<CollisionSphereGroup>* sphere_groups = nullptr,
            gpmp2::BodySphereVector* authored_spheres = nullptr);
};
