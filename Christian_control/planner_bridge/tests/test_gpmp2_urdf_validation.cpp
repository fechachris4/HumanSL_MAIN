// Tripwire: gpmp2::Arm's DH FK (built from the BUILD-GENERATED
// dh_params_tool.yaml via LoadPlannerModel — the model PlanSolver.cpp
// actually optimizes trajectories against) compared to Pinocchio/URDF FK at
// the same joint configs.
//
// The YAML is generated from the URDF by generate_dh_params at build time
// (see planner_bridge/CMakeLists.txt), which already self-verifies its
// derivation. This test is the independent end-to-end guard on top: it
// exercises the ACTUAL consumer chain (YAML file -> createDHParams ->
// gpmp2::Arm FK), so a stale artifact, a loader bug, or a gpmp2 convention
// mismatch all trip it even if the generator's internal checks passed.
//
// Tolerances: the URDF writes joint rotations to 5 significant figures
// (rpy "1.5708"), giving a ~2e-5 m / ~3e-5 rad floor between exact-pi DH
// convention constants and Pinocchio's parse. 0.1 mm / 0.01 deg sits ~4x
// above that floor and 10x tighter than the hand-YAML era.
#undef NDEBUG

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include "Config.h"
#include "PinocchioKinematicsAdapter.h"
#include "PlannerModel.h"
#include "utils.h"

namespace {
void CheckChain(const char* label, const std::string& yaml_path,
                const char* end_effector_frame, bool left_arm, bool has_tool) {
    const Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
    const PlannerModel model =
        LoadPlannerModel(yaml_path, has_tool, world_T_mount);
    // Identity T_W_M makes world coincide with Mount in this regression;
    // GPMP2 FK therefore comes out in both frames here.
    // The Pinocchio adapter is permanently base_link (deliberately — see
    // Config.h), so it is carried up by the same URDF-read mounting transform
    // the model used. An exact rigid transform, so it adds no error of its
    // own beyond the URDF's rounding, which the tolerances already clear.
    const Eigen::Isometry3d mount_from_base =
        pinocchio_kinematics_adapter::MountFromBase(left_arm);

    std::mt19937 rng(7);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);

    double worst_pos_mm = 0.0, worst_rot_deg = 0.0;
    for (int trial = 0; trial < 200; ++trial) {
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
        if (trial > 0)
            for (int j = 0; j < 7; ++j) q[j] = dist(rng);

        const gtsam::Pose3 p_gpmp2 = createPoseFromConf(*model.arm_model, gtsam::Vector(q));
        const auto p_base = pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
            q, end_effector_frame, left_arm);
        const Eigen::Vector3d pin_position = mount_from_base * p_base.position;
        const Eigen::Matrix3d pin_rotation = mount_from_base.linear() * p_base.rotation;

        const double pos_err_mm = (p_gpmp2.translation() - pin_position).norm() * 1000.0;
        const Eigen::Matrix3d r_err = p_gpmp2.rotation().matrix().transpose() * pin_rotation;
        const double angle_rad = std::acos(std::clamp((r_err.trace() - 1.0) / 2.0, -1.0, 1.0));

        worst_pos_mm = std::max(worst_pos_mm, pos_err_mm);
        worst_rot_deg = std::max(worst_rot_deg, angle_rad * 180.0 / M_PI);
    }

    std::printf(
        "gpmp2::Arm (%s) vs Pinocchio/URDF: worst position %.4f mm, worst rotation %.4f deg\n",
        label, worst_pos_mm, worst_rot_deg);
    assert(worst_pos_mm < 0.1 &&
           "generated DH parameters must agree with the URDF under 0.1 mm");
    assert(worst_rot_deg < 0.01 &&
           "generated DH parameters must agree with the URDF under 0.01 deg");
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 2 && argc != 3) {
        std::fprintf(stderr,
                     "usage: test_gpmp2_urdf_validation <dh_params_tool.yaml> "
                     "[dh_params_flange.yaml]\n");
        return 1;
    }
    CheckChain("dh_params_tool.yaml, right", argv[1], config::kRightEndEffectorFrame,
              /*left_arm=*/false, /*has_tool=*/true);
    if (argc == 3)
        CheckChain("dh_params_flange.yaml, left", argv[2], config::kLeftEndEffectorFrame,
                  /*left_arm=*/true, /*has_tool=*/false);
    return 0;
}
