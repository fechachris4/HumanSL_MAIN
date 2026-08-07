// Consistency check: PlannerModel::ToolPoseInMount (Pinocchio, via
// utils.cpp's forwardKinematics, with the arm built at DhRootInMount) against
// the same Pinocchio adapter called directly in base_link and then carried
// into mount by MountFromBase.
//
// Both sides share one Pinocchio/URDF backend, so this does not catch shared
// coding-logic bugs the way comparing two independently-implemented FK chains
// would. What it does catch: a wrong frame name, URDF path, or config-mapping
// bug in the composition identity forwardKinematics uses
// (base_pose * DhRootInBaseLink()^-1 * base_link_M_tool), which a direct
// adapter call bypasses entirely — and, since the move into `mount`, the
// mounting composition itself, including the SIGN of the +-1.2085 rad roll.
//
// Both arms are exercised. The left arm's mount is the mirror of the right's
// and was previously untested anywhere in the planner, which is exactly where
// a copy-pasted right-hand transform would hide.
#undef NDEBUG

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "Config.h"
#include "PinocchioKinematicsAdapter.h"
#include "PlannerModel.h"

namespace {

// Worst position (mm) and orientation (deg) disagreement over `trials`
// configurations, between the planner's FK and the adapter carried into mount.
void CheckArm(const char* label, const std::string& dh_yaml, bool has_tool) {
    const PlannerModel model = LoadPlannerModel(dh_yaml, has_tool);
    const bool left_arm = !has_tool;
    const char* end_effector_frame =
        has_tool ? config::kRightEndEffectorFrame : config::kLeftEndEffectorFrame;
    const Eigen::Isometry3d mount_from_base =
        pinocchio_kinematics_adapter::MountFromBase(left_arm);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);
    double worst_mm = 0.0, worst_deg = 0.0;
    for (int trial = 0; trial < 200; ++trial) {
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
        if (trial > 0)
            for (int j = 0; j < 7; ++j) q[j] = dist(rng);

        const gtsam::Pose3 p_planner = ToolPoseInMount(model, q);
        const auto p_base = pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
            q, end_effector_frame, left_arm);
        const Eigen::Vector3d expected_position = mount_from_base * p_base.position;
        const Eigen::Matrix3d expected_rotation = mount_from_base.linear() * p_base.rotation;

        worst_mm = std::max(worst_mm,
                            (p_planner.translation() - expected_position).norm() * 1000.0);
        const Eigen::Matrix3d r_err =
            p_planner.rotation().matrix().transpose() * expected_rotation;
        const double angle_rad =
            std::acos(std::clamp((r_err.trace() - 1.0) / 2.0, -1.0, 1.0));
        worst_deg = std::max(worst_deg, angle_rad * 180.0 / M_PI);
    }
    std::printf("%-6s planner-vs-adapter-through-mount: %.9f mm, %.9f deg\n", label,
                worst_mm, worst_deg);
    assert(worst_mm < 1e-6 &&
           "ToolPoseInMount must match the adapter carried through MountFromBase");
    assert(worst_deg < 1e-6 &&
           "ToolPoseInMount must match the adapter carried through MountFromBase");
}

}  // namespace

int main(int argc, char** argv) {
    assert(argc == 3 &&
           "usage: test_planner_model <dh_params_tool.yaml> <dh_params_flange.yaml>");
    CheckArm("right", argv[1], /*has_tool=*/true);
    CheckArm("left", argv[2], /*has_tool=*/false);

    // An anchor independent of the consistency checks above. Those compare
    // two paths that both read MountFromBase, so a right-hand transform
    // copy-pasted into the left slot would satisfy both and still be wrong.
    // The URDF mounts the bases mirrored about the mount XZ plane, so their
    // origins must be exact negatives of each other.
    const Eigen::Vector3d right_origin =
        pinocchio_kinematics_adapter::MountFromBase(false).translation();
    const Eigen::Vector3d left_origin =
        pinocchio_kinematics_adapter::MountFromBase(true).translation();
    std::printf("base origins in mount: right [%.6f %.6f %.6f], left [%.6f %.6f %.6f]\n",
                right_origin.x(), right_origin.y(), right_origin.z(), left_origin.x(),
                left_origin.y(), left_origin.z());
    assert((right_origin + left_origin).norm() < 1e-12 &&
           "the two base origins must be mirrored about the mount origin");
    assert(right_origin.norm() > 1e-6 &&
           "the bases must be separated — a zero transform would pass every "
           "consistency check above");
    return 0;
}
