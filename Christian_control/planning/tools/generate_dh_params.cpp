//
// generate_dh_params — build-step tool: derive a GPMP2 DH parameter YAML
// from the canonical URDF (GEN3_dual_mounted.urdf, via Pinocchio), so the
// planner's kinematic model is generated from the URDF, never maintained by
// hand. Runs at build time, once per arm (see planner_bridge/CMakeLists.txt):
// the default (no --arm) derives the right chain, ending at
// right_tool_link (the mounted tool); --arm left derives the left
// chain, ending at left_end_effector_link (the bare flange — no tool is
// mounted on the left arm). A failed derivation exits non-zero and fails
// the build loudly.
//
// The DH structural convention below (alpha/theta_offset/base = Rx(pi)) is
// the SAME fixed frame convention for both chains: both Gen3 units are the
// same physical arm, structurally identical joint-for-joint, mounted only
// via each side's own fixed base joint (handled separately, outside this
// per-arm DH-root-to-own-base_link relationship — see DhRootInBaseLink).
// Steps 1 and 3 below re-verify that assumption against the URDF for
// whichever chain this run derives, rather than trusting it — a URDF where
// the left chain does not fit the convention fails the build here, not
// silently at the collision model.
//
// What is derived vs fixed:
//   - The DH *structure* — a = 0 for every joint, alpha = pi/2 for joints
//     1-6 and pi for joint 7, theta_offset = 0 then pi x6, base = Rx(pi) —
//     is a fixed frame CONVENTION, not measured data. It must stay fixed:
//     GenerateArmModel.cpp's collision-sphere offsets are expressed in the
//     DH link frames this convention defines, so a convention change would
//     silently displace the collision model. Steps 1 and 3 below verify the
//     URDF still fits the convention and abort otherwise.
//   - The seven link lengths d1..d7 are the physical quantities that live
//     in the URDF. With all a = 0, tool POSITION is linear in the d values
//     (each link contributes R_prefix * (0,0,d_i), and rotations are
//     d-independent), so they are recovered by least squares against
//     Pinocchio forward kinematics and then re-verified on fresh samples.
//
// Tolerances: the URDF writes its joint rotations to 5 significant figures
// (rpy "1.5708", "3.1416"), so the exact-pi convention can only agree with
// Pinocchio to ~4e-5 rad / ~2e-5 m. Thresholds sit ~4-10x above that floor;
// a genuinely wrong convention constant misses them by orders of magnitude.
//
// Uses only PinocchioKinematicsAdapter.h (Eigen-only): gtsam and Pinocchio
// headers must never share a translation unit (colliding
// boost::serialization overloads for Eigen::Matrix).
//

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Config.h"
#include "PinocchioKinematicsAdapter.h"

namespace {

constexpr int kJointCount = 7;
constexpr double kRotationCheckTolFrobenius = 1e-3; // floor ~4e-5
constexpr double kPositionTolM = 2e-4;              // floor ~2e-5
constexpr double kRotationTolRad = 5e-4;            // floor ~3e-5
constexpr double kConditionFloor = 1e-6;

const double kAlpha[kJointCount] = {M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2,
                                    M_PI / 2, M_PI / 2, M_PI};
const double kThetaOffset[kJointCount] = {0, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};
// Kinova joints 1/3/5/7 are continuous, 2/4/6 bounded — createDHParams
// ignores these fields; kept for human readability of the generated file.
const char* kJointTypes[kJointCount] = {"continuous", "revolute", "continuous",
                                        "revolute",   "continuous", "revolute",
                                        "continuous"};

// Everything that differs between the two chains: which arm's branch
// Pinocchio resolves against (left_arm), the URDF end-effector frame that
// terminates it, and the joint-name labels the generated YAML carries for
// readability (see the header comment — never used to resolve anything).
struct ArmDhConfig {
    bool left_arm;
    const char* end_effector_frame;
    const char* joint_names[kJointCount];
};

const ArmDhConfig kRightArmDh{
    false, config::kRightEndEffectorFrame,
    {"right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4",
     "right_joint_5", "right_joint_6", "right_joint_7_to_configured_tool"}};
const ArmDhConfig kLeftArmDh{
    true, config::kLeftEndEffectorFrame,
    {"left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4",
     "left_joint_5", "left_joint_6", "left_joint_7_to_flange"}};

Eigen::Matrix3d RotZ(double angle) {
    return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
Eigen::Matrix3d RotX(double angle) {
    return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

// Rotation of the DH chain after joint i (0-based, inclusive), from the
// convention constants. Independent of every d (all a = 0).
Eigen::Matrix3d ConventionRotationThrough(const Eigen::Matrix<double, 7, 1>& q,
                                          int last_joint) {
    Eigen::Matrix3d rotation =
        pinocchio_kinematics_adapter::DhRootInBaseLink().rotation();
    for (int i = 0; i <= last_joint; ++i)
        rotation = rotation * RotZ(q[i] + kThetaOffset[i]) * RotX(kAlpha[i]);
    return rotation;
}

// Full-chain DH forward kinematics from the convention + candidate d values,
// in base_link. Mirrors gpmp2::Arm's chain: base * prod_i DH(a=0, alpha_i,
// d_i, theta_i + q_i).
void ConventionForwardKinematics(const Eigen::Matrix<double, 7, 1>& q,
                                 const Eigen::Matrix<double, 7, 1>& d,
                                 Eigen::Vector3d& position_out,
                                 Eigen::Matrix3d& rotation_out) {
    Eigen::Matrix3d rotation =
        pinocchio_kinematics_adapter::DhRootInBaseLink().rotation();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    for (int i = 0; i < kJointCount; ++i) {
        rotation = rotation * RotZ(q[i] + kThetaOffset[i]);
        position += rotation * Eigen::Vector3d(0, 0, d[i]);
        rotation = rotation * RotX(kAlpha[i]);
    }
    position_out = position;
    rotation_out = rotation;
}

Eigen::Matrix<double, 7, 1> RandomConfiguration(std::mt19937& rng) {
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);
    Eigen::Matrix<double, 7, 1> q;
    for (int i = 0; i < kJointCount; ++i) q[i] = dist(rng);
    return q;
}

double RotationAngleBetween(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    const Eigen::Matrix3d error = a.transpose() * b;
    const double cos_angle = std::min(1.0, std::max(-1.0, (error.trace() - 1.0) / 2.0));
    return std::acos(cos_angle);
}

// Derives one chain's DH YAML and writes it to output_path. Throws
// std::runtime_error naming which of the four steps failed and why; the
// caller (main) turns that into a nonzero exit.
void DeriveAndEmit(const ArmDhConfig& arm, const std::string& output_path) {
    // Fixed seed: rebuilds are byte-identical, and the derivation is
    // fully re-verified on every run regardless of the sample draw.
    std::mt19937 rng(2026);

    // --- Step 1: the convention's rotation chain must match the URDF.
    double worst_rotation_frobenius = 0.0;
    for (int trial = 0; trial < 64; ++trial) {
        const auto q = RandomConfiguration(rng);
        const auto pinocchio_pose =
            pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
                q, arm.end_effector_frame, arm.left_arm);
        const Eigen::Matrix3d convention_rotation =
            ConventionRotationThrough(q, kJointCount - 1);
        worst_rotation_frobenius =
            std::max(worst_rotation_frobenius,
                     (convention_rotation - pinocchio_pose.rotation).norm());
    }
    if (worst_rotation_frobenius > kRotationCheckTolFrobenius)
        throw std::runtime_error(
            "URDF is no longer DH-compatible with the fixed convention "
            "(rotation chain mismatch, Frobenius " +
            std::to_string(worst_rotation_frobenius) +
            "). The alpha/theta/base constants — and the collision-sphere "
            "frames that depend on them — must be re-derived by hand.");

    // --- Step 2: solve the seven d values, linear least squares.
    // p(q) = sum_i [R_prefix_i(q) * z_hat] * d_i, columns = joint axes.
    constexpr int kSolveSamples = 32;
    Eigen::MatrixXd system(3 * kSolveSamples, kJointCount);
    Eigen::VectorXd rhs(3 * kSolveSamples);
    for (int sample = 0; sample < kSolveSamples; ++sample) {
        const auto q = RandomConfiguration(rng);
        const auto pinocchio_pose =
            pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
                q, arm.end_effector_frame, arm.left_arm);
        for (int i = 0; i < kJointCount; ++i) {
            // Accumulated rotation up to and including RotZ(q_i+theta_i):
            // that frame's z-axis is joint i's translation direction.
            Eigen::Matrix3d prefix =
                pinocchio_kinematics_adapter::DhRootInBaseLink().rotation();
            for (int k = 0; k < i; ++k)
                prefix = prefix * RotZ(q[k] + kThetaOffset[k]) * RotX(kAlpha[k]);
            prefix = prefix * RotZ(q[i] + kThetaOffset[i]);
            system.block<3, 1>(3 * sample, i) = prefix.col(2);
        }
        rhs.segment<3>(3 * sample) = pinocchio_pose.position;
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        system, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const double condition = svd.singularValues()(kJointCount - 1) /
                             svd.singularValues()(0);
    if (condition < kConditionFloor)
        throw std::runtime_error(
            "d-solve system is rank-deficient (sigma_min/sigma_max = " +
            std::to_string(condition) + ") — sample configurations degenerate");
    const Eigen::Matrix<double, 7, 1> d = svd.solve(rhs);

    const double solve_residual = (system * d - rhs).cwiseAbs().maxCoeff();
    if (solve_residual > kPositionTolM)
        throw std::runtime_error(
            "URDF is no longer DH-compatible with the fixed convention "
            "(d-solve residual " + std::to_string(solve_residual) +
            " m — a lateral offset or nonzero a-parameter exists that the "
            "DH structure cannot express).");

    // --- Step 3: verify the derived chain on fresh configurations.
    double worst_position_m = 0.0, worst_rotation_rad = 0.0;
    for (int trial = 0; trial < 64; ++trial) {
        const auto q = RandomConfiguration(rng);
        const auto pinocchio_pose =
            pinocchio_kinematics_adapter::ToolPoseAndJacobianInBaseLink(
                q, arm.end_effector_frame, arm.left_arm);
        Eigen::Vector3d dh_position;
        Eigen::Matrix3d dh_rotation;
        ConventionForwardKinematics(q, d, dh_position, dh_rotation);
        worst_position_m = std::max(
            worst_position_m, (dh_position - pinocchio_pose.position).norm());
        worst_rotation_rad = std::max(
            worst_rotation_rad,
            RotationAngleBetween(dh_rotation, pinocchio_pose.rotation));
    }
    if (worst_position_m > kPositionTolM || worst_rotation_rad > kRotationTolRad)
        throw std::runtime_error(
            "derived DH chain failed verification against Pinocchio (" +
            std::to_string(worst_position_m) + " m, " +
            std::to_string(worst_rotation_rad) + " rad)");

    // --- Step 4: emit. Temp file + rename so a failed run can never
    // leave a plausible half-written YAML behind.
    const std::string temp_path = output_path + ".tmp";
    {
        std::ofstream out(temp_path);
        if (!out)
            throw std::runtime_error("cannot open " + temp_path + " for writing");
        out.precision(std::numeric_limits<double>::max_digits10);
        out << "# GENERATED — DO NOT EDIT.\n"
               "# Derived from the canonical URDF by generate_dh_params\n"
               "# (planner_bridge/tools/) at build time:\n"
               "#   source: " << GEN3_DUAL_URDF_PATH << "\n"
               "#   arm:    " << (arm.left_arm ? "left" : "right") << "\n"
               "#   frame:  " << arm.end_effector_frame << "\n"
               "# The a/alpha/theta_offset values are the fixed DH frame\n"
               "# convention (see the generator's header comment); only the\n"
               "# d values are derived from the URDF. Verified against\n"
               "# Pinocchio FK before this file was written; the ongoing\n"
               "# guard is test_gpmp2_urdf_validation.\n"
               "\n"
               "dh_parameters:\n";
        for (int i = 0; i < kJointCount; ++i) {
            out << "  - joint_id: " << (i + 1) << "\n"
                << "    joint_name: \"" << arm.joint_names[i] << "\"\n"
                << "    joint_type: \"" << kJointTypes[i] << "\"\n"
                << "    a: 0.0\n"
                << "    alpha: " << kAlpha[i] << "\n"
                << "    d: " << d[i] << "\n"
                << "    theta_offset: " << kThetaOffset[i] << "\n";
        }
        if (!out.good())
            throw std::runtime_error("write to " + temp_path + " failed");
    }
    if (std::rename(temp_path.c_str(), output_path.c_str()) != 0)
        throw std::runtime_error("cannot rename " + temp_path + " onto " +
                                 output_path);

    std::fprintf(stderr,
                 "generate_dh_params (%s): derived d = [%.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f]\n",
                 arm.left_arm ? "left" : "right", d[0], d[1], d[2], d[3], d[4],
                 d[5], d[6]);
    std::fprintf(stderr,
                 "generate_dh_params (%s): rotation check %.2e (Frobenius), "
                 "solve residual %.2e m, verify worst %.2e m / %.2e rad\n",
                 arm.left_arm ? "left" : "right", worst_rotation_frobenius,
                 solve_residual, worst_position_m, worst_rotation_rad);
}

} // namespace

int main(int argc, char** argv) {
    std::string output_path;
    bool left_arm = false;
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--arm") {
            if (++i >= argc) {
                std::fprintf(stderr, "--arm needs a value\n");
                return 1;
            }
            const std::string value = argv[i];
            if (value == "left") left_arm = true;
            else if (value != "right") {
                std::fprintf(stderr, "--arm must be 'right' or 'left'\n");
                return 1;
            }
        } else if (output_path.empty()) {
            output_path = flag;
        } else {
            std::fprintf(stderr,
                         "usage: generate_dh_params <output.yaml> [--arm right|left]\n");
            return 1;
        }
    }
    if (output_path.empty()) {
        std::fprintf(stderr,
                     "usage: generate_dh_params <output.yaml> [--arm right|left]\n");
        return 1;
    }

    try {
        DeriveAndEmit(left_arm ? kLeftArmDh : kRightArmDh, output_path);
        return 0;
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "generate_dh_params: FAILED: %s\n", exception.what());
        return 1;
    }
}
