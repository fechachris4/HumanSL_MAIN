// Consistency check: PlannerModel::ToolPoseInBaseLink (Pinocchio, via
// utils.cpp's forwardKinematics) against a direct call to the same
// Pinocchio adapter. Both share one Pinocchio/URDF backend, so this does
// not catch shared coding-logic bugs the way comparing two
// independently-implemented FK chains would — but it still catches a wrong
// frame name, URDF path, or config-mapping bug in the composition identity
// forwardKinematics uses (base_pose_in_world * DhRootInBaseLink()^-1 *
// base_link_M_tool), which a direct adapter call bypasses entirely.
#undef NDEBUG

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include "Config.h"
#include "PinocchioKinematicsAdapter.h"
#include "PlannerModel.h"

int main(int argc, char** argv) {
    assert(argc == 2 && "usage: test_planner_model <dh_params_tool.yaml>");
    const PlannerModel model = LoadPlannerModel(argv[1]);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);
    double worst_mm = 0.0, worst_deg = 0.0;
    for (int trial = 0; trial < 200; ++trial) {
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
        if (trial > 0)
            for (int j = 0; j < 7; ++j) q[j] = dist(rng);
        const gtsam::Pose3 p_planner = ToolPoseInBaseLink(model, q);
        const auto p_direct = pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
            q, config::kRightEndEffectorFrame);
        worst_mm = std::max(worst_mm,
                            (p_planner.translation() - p_direct.position).norm() * 1000.0);
        const Eigen::Matrix3d r_err =
            p_planner.rotation().matrix().transpose() * p_direct.rotation;
        const double angle_rad =
            std::acos(std::clamp((r_err.trace() - 1.0) / 2.0, -1.0, 1.0));
        worst_deg = std::max(worst_deg, angle_rad * 180.0 / M_PI);
    }
    std::printf("worst PlannerModel-vs-adapter disagreement: %.9f mm, %.9f deg\n",
                worst_mm, worst_deg);
    assert(worst_mm < 1e-6 &&
           "PlannerModel::ToolPoseInBaseLink must exactly match the direct adapter call");
    assert(worst_deg < 1e-6 &&
           "PlannerModel::ToolPoseInBaseLink must exactly match the direct adapter call");
    return 0;
}
