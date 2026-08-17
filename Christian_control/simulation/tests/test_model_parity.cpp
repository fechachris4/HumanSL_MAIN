//
// Model parity gate (Plan 02 Task 3): MuJoCo world kinematics of the
// generated dual-arm MJCF against the PRODUCTION Pinocchio kinematics
// (DualArmKinematics on GEN3_dual_mounted.urdf) — two independent FK
// pipelines over the same declared frame chain.
//
// Physical question: does the simulated plant have exactly the HumanSL
// kinematics? For each arm i and configuration q:
//
//   world_T_base[i] = world_T_mount * mount_T_base[i]
//   world_T_tcp[i]  = world_T_base[i] * base_T_tcp[i](q[i])
//
// with mount_T_base[i] = DualArmKinematics::MountFromBase(i) (the runtime
// authority) and base_T_tcp from the URDF chain. MuJoCo's side: the Mount
// mocap row (mocap_pos/mocap_quat) carries world_T_mount, body/site frames
// give world_T_base and world_T_tcp. Pinocchio's side: ToolPoseInMount
// (mount is the URDF root) composed with the same world_T_mount.
//
// Jacobian gate: the production world Jacobian columns (LOCAL_WORLD_ALIGNED
// at the tool point, base axes from ControlledPoseAndJacobian, rotated to
// world by R_W_M * R_M_B) against forward differences of the MuJoCo site
// frame, h = 1e-7 rad per joint. Angular columns are recovered as
// vee((R(q+h) R(q)^T - I)/h) in world axes — the quantity comparable to
// LOCAL_WORLD_ALIGNED angular columns (element-wise rotation differencing
// would not be).
//
// Tolerances (model_parity_cases.h): position <=1e-8 m, orientation-log
// norm <=1e-8 rad, finite-difference Jacobian <=1e-6 per component.
//
// Mutation evidence (pre-registered in the accepted analysis packet):
//   1. swapped joints 2/4 fed to MuJoCo  -> TCP position error >1e-2 m;
//   2. inverted right_joint_5 axis       -> rejected by ModelContract, and
//      with that first line bypassed the FD column flips sign (error
//      ~2*||column|| >> 1e-6);
//   3. left/right mount placement swap   -> ACCEPTED by ModelContract
//      (structural gates are placement-blind, Task 2 probe) but base-pose
//      parity fails with ~2*0.0567075 = 0.1134 m and ~2*1.2085 = 2.417 rad.
//
// Evidence class: unit test (kinematic parity; nothing is stepped, no
// robot-facing code exists in this project).
//

#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include "Config.h"
#include "RobotModel.h"
#include "Kinematics.h"
#include "ModelContract.h"
#include "model_parity_cases.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::printf("FAIL: %s\n", what.c_str());
            ++failures;
        }
    }

    std::string VecText(const Eigen::VectorXd& v)
    {
        std::ostringstream text;
        text << "[";
        for (int i = 0; i < v.size(); ++i)
            text << (i ? " " : "") << std::scientific << v[i];
        text << "]";
        return text.str();
    }

    Eigen::Matrix3d RowMajor3(const double* m)
    {
        Eigen::Matrix3d rotation;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                rotation(row, col) = m[3 * row + col]; // mjData *_xmat is row-major
        return rotation;
    }

    // Geodesic angle between two rotations: || log(a * b^T) ||, via the
    // relative quaternion (2*atan2(|vec|, |w|)), robust near identity.
    double RelativeAngleRad(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b)
    {
        const Eigen::Quaterniond relative(a * b.transpose());
        return 2.0 * std::atan2(relative.vec().norm(), std::abs(relative.w()));
    }

    struct ParityCaseState {
        std::string name;
        Eigen::Matrix<double, 7, 1> right_q_rad;
        Eigen::Matrix<double, 7, 1> left_q_rad;
        Eigen::Vector3d world_p_mount_m;
        Eigen::Quaterniond world_q_mount; // normalized before use
    };

    std::vector<ParityCaseState> BuildCases()
    {
        std::vector<ParityCaseState> cases;
        for (const parity_cases::Case& fixed : parity_cases::kFixedCases) {
            ParityCaseState state;
            state.name = fixed.name;
            for (int i = 0; i < 7; ++i) {
                state.right_q_rad[i] = fixed.right_q_rad[i];
                state.left_q_rad[i] = fixed.left_q_rad[i];
            }
            state.world_p_mount_m = Eigen::Vector3d(fixed.mount_position_m[0],
                                                    fixed.mount_position_m[1],
                                                    fixed.mount_position_m[2]);
            state.world_q_mount =
                Eigen::Quaterniond(fixed.mount_quat_wxyz[0],
                                   fixed.mount_quat_wxyz[1],
                                   fixed.mount_quat_wxyz[2],
                                   fixed.mount_quat_wxyz[3])
                    .normalized();
            cases.push_back(state);
        }

        // Seeded deterministic randoms (model_parity_cases.h documents the
        // sampling ranges). One generator, fixed draw order.
        std::mt19937 rng(parity_cases::kRandomSeed);
        std::uniform_real_distribution<double> continuous(-2.0 * M_PI,
                                                          2.0 * M_PI);
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        std::uniform_real_distribution<double> position(-0.8, 1.2);
        for (int n = 0; n < parity_cases::kRandomCaseCount; ++n) {
            ParityCaseState state;
            state.name = "seeded_random_" + std::to_string(n);
            for (Eigen::Matrix<double, 7, 1>* q :
                 {&state.right_q_rad, &state.left_q_rad}) {
                for (int i = 0; i < 7; ++i) {
                    const double range = parity_cases::kBoundedRangeRad[i];
                    if (range == 0.0) {
                        (*q)[i] = continuous(rng);
                    } else {
                        const double half =
                            range - parity_cases::kBoundedMarginRad;
                        (*q)[i] = half * unit(rng);
                    }
                }
            }
            state.world_p_mount_m = Eigen::Vector3d(
                position(rng), position(rng), position(rng));
            Eigen::Vector4d raw;
            do {
                raw = Eigen::Vector4d(unit(rng), unit(rng), unit(rng),
                                      unit(rng));
            } while (raw.norm() < 1e-3);
            raw.normalize();
            state.world_q_mount = Eigen::Quaterniond(raw[0], raw[1], raw[2],
                                                     raw[3]); // already unit
            cases.push_back(state);
        }
        return cases;
    }

    // Writes one case into the Mount's mocap row (position, then the
    // quaternion in MuJoCo's w,x,y,z order) and mjData.qpos (hinges by
    // the contract's resolved addresses), then recomputes pure forward
    // kinematics — nothing is stepped. mj_kinematics reads mocap_pos /
    // mocap_quat as the Mount body's world pose, so this places the Mount
    // exactly as the backend's per-tick write does.
    void ApplyState(const mjModel& model, mjData& data,
                    const DualModelContract& ids, const ParityCaseState& c)
    {
        mjtNum* mocap_pos = data.mocap_pos + 3 * ids.mount_mocap_id;
        mocap_pos[0] = c.world_p_mount_m[0];
        mocap_pos[1] = c.world_p_mount_m[1];
        mocap_pos[2] = c.world_p_mount_m[2];
        mjtNum* mocap_quat = data.mocap_quat + 4 * ids.mount_mocap_id;
        mocap_quat[0] = c.world_q_mount.w();
        mocap_quat[1] = c.world_q_mount.x();
        mocap_quat[2] = c.world_q_mount.y();
        mocap_quat[3] = c.world_q_mount.z();
        for (int i = 0; i < 7; ++i) {
            data.qpos[ids.right.joint_qpos_adr[i]] = c.right_q_rad[i];
            data.qpos[ids.left.joint_qpos_adr[i]] = c.left_q_rad[i];
        }
        mj_kinematics(&model, &data);
    }

    double max_position_error_m = 0.0;
    double max_orientation_error_rad = 0.0;
    double max_jacobian_error = 0.0;

    void ComparePose(const std::string& label, const Eigen::Vector3d& mj_p,
                     const Eigen::Matrix3d& mj_R, const Eigen::Vector3d& pin_p,
                     const Eigen::Matrix3d& pin_R)
    {
        const double position_error = (mj_p - pin_p).norm();
        const double orientation_error = RelativeAngleRad(mj_R, pin_R);
        if (position_error > max_position_error_m)
            max_position_error_m = position_error;
        if (orientation_error > max_orientation_error_rad)
            max_orientation_error_rad = orientation_error;
        if (position_error > parity_cases::kPositionToleranceM) {
            std::ostringstream message;
            message << label << ": position error " << std::scientific
                    << position_error << " m exceeds "
                    << parity_cases::kPositionToleranceM << "; mujoco "
                    << VecText(mj_p) << " pinocchio " << VecText(pin_p);
            Check(false, message.str());
        }
        if (orientation_error > parity_cases::kOrientationToleranceRad) {
            std::ostringstream message;
            message << label << ": orientation-log error " << std::scientific
                    << orientation_error << " rad exceeds "
                    << parity_cases::kOrientationToleranceRad;
            Check(false, message.str());
        }
    }

    // Forward-difference world Jacobian of one arm's TCP site from MuJoCo
    // alone (the independent pipeline). Assumes data already holds the case
    // state with kinematics computed; leaves it restored.
    Eigen::Matrix<double, 6, 7> FiniteDifferenceWorldJacobian(
        const mjModel& model, mjData& data, const ArmModelIds& arm)
    {
        const double h = parity_cases::kFiniteDifferenceStepRad;
        const Eigen::Vector3d p0(data.site_xpos + 3 * arm.tcp_site_id);
        const Eigen::Matrix3d R0 =
            RowMajor3(data.site_xmat + 9 * arm.tcp_site_id);

        Eigen::Matrix<double, 6, 7> jacobian;
        for (int i = 0; i < 7; ++i) {
            const int adr = arm.joint_qpos_adr[i];
            const double saved = data.qpos[adr];
            data.qpos[adr] = saved + h;
            mj_kinematics(&model, &data);
            const Eigen::Vector3d p1(data.site_xpos + 3 * arm.tcp_site_id);
            const Eigen::Matrix3d R1 =
                RowMajor3(data.site_xmat + 9 * arm.tcp_site_id);
            data.qpos[adr] = saved;

            jacobian.block<3, 1>(0, i) = (p1 - p0) / h;
            // Angular column: vee of the skew part of (R1 R0^T - I)/h —
            // the world-axes angular velocity per unit joint rate, the
            // quantity LOCAL_WORLD_ALIGNED angular rows report.
            const Eigen::Matrix3d A = (R1 * R0.transpose() -
                                       Eigen::Matrix3d::Identity()) /
                                      h;
            jacobian(3, i) = 0.5 * (A(2, 1) - A(1, 2));
            jacobian(4, i) = 0.5 * (A(0, 2) - A(2, 0));
            jacobian(5, i) = 0.5 * (A(1, 0) - A(0, 1));
        }
        mj_kinematics(&model, &data); // restore baseline frames
        return jacobian;
    }

    // Production world Jacobian: base-axes columns from
    // ControlledPoseAndJacobian rotated into world by R_W_B = R_W_M * R_M_B.
    Eigen::Matrix<double, 6, 7> AnalyticWorldJacobian(
        DualArmKinematics& kinematics,
        const Eigen::Matrix<double, 7, 1>& q_rad,
        const Eigen::Matrix3d& world_R_mount, KinematicsWorkspace& workspace)
    {
        const PoseJacobian base = kinematics.ControlledPoseAndJacobian(
            q_rad, workspace);
        const Eigen::Matrix3d world_R_base =
            world_R_mount *
            kinematics.MountFromBase(kinematics.controlled_arm()).rotation();
        Eigen::Matrix<double, 6, 7> world;
        for (int i = 0; i < 7; ++i) {
            world.block<3, 1>(0, i) =
                world_R_base * base.jacobian.block<3, 1>(0, i);
            world.block<3, 1>(3, i) =
                world_R_base * base.jacobian.block<3, 1>(3, i);
        }
        return world;
    }

    // Compares FD and analytic columns component-wise; returns the largest
    // absolute error so mutation checks can assert on magnitude.
    double CompareJacobians(const std::string& label,
                            const Eigen::Matrix<double, 6, 7>& fd,
                            const Eigen::Matrix<double, 6, 7>& analytic,
                            bool report)
    {
        double worst = 0.0;
        for (int joint = 0; joint < 7; ++joint) {
            const Eigen::Matrix<double, 6, 1> error =
                fd.col(joint) - analytic.col(joint);
            const double column_worst = error.cwiseAbs().maxCoeff();
            if (column_worst > worst)
                worst = column_worst;
            if (report && column_worst > parity_cases::kJacobianTolerance) {
                std::ostringstream message;
                message << label << " joint " << (joint + 1)
                        << ": max component error " << std::scientific
                        << column_worst << " exceeds "
                        << parity_cases::kJacobianTolerance << "; expected "
                        << VecText(analytic.col(joint)) << " actual (FD) "
                        << VecText(fd.col(joint));
                Check(false, message.str());
            }
        }
        if (report && worst > max_jacobian_error)
            max_jacobian_error = worst;
        return worst;
    }

    // ----- mutated-model loading (same technique as test_model_contract) --

    std::string ReadFile(const std::string& path)
    {
        std::ifstream stream(path);
        std::ostringstream buffer;
        buffer << stream.rdbuf();
        return buffer.str();
    }

    int Replace(std::string& text, const std::string& from,
                const std::string& to, int count)
    {
        int done = 0;
        std::size_t pos = 0;
        while ((pos = text.find(from, pos)) != std::string::npos) {
            text.replace(pos, from.size(), to);
            pos += to.size();
            ++done;
            if (count != 0 && done == count)
                break;
        }
        return done;
    }

    mjModel* LoadFromText(std::string xml_text, std::string* error_out)
    {
        const int redirected =
            Replace(xml_text, "meshdir=\"assets\"",
                    "meshdir=\"" SIM_MODEL_DIR "/assets\"", 1);
        if (redirected != 1) {
            *error_out = "meshdir attribute not found in model text";
            return nullptr;
        }
        const std::string temp_path = "mutated_model_parity.xml";
        std::ofstream(temp_path) << xml_text;
        char error[1024] = {0};
        mjModel* model =
            mj_loadXML(temp_path.c_str(), nullptr, error, sizeof(error));
        *error_out = error;
        return model;
    }
} // namespace

int main()
{
    // --- production Pinocchio side (the kinematic authority) -------------
    RobotModel robot_model(SIM_URDF_PATH);
    DualArmKinematics kinematics_right(
        robot_model, Arm::kRight, config::kLeftNominalRad,
        config::kRightBaseFrame, config::kRightEndEffectorFrame);
    DualArmKinematics kinematics_left(
        robot_model, Arm::kLeft, config::kRightNominalRad,
        config::kRightBaseFrame, config::kRightEndEffectorFrame);
    KinematicsWorkspace workspace(robot_model);

    // --- MuJoCo side ------------------------------------------------------
    char error[1024] = {0};
    mjModel* model =
        mj_loadXML(SIM_MODEL_XML_PATH, nullptr, error, sizeof(error));
    if (model == nullptr) {
        std::printf("FAIL: model failed to load: %s\n", error);
        return 1;
    }
    DualModelContract ids{};
    try {
        ids = ValidateAndResolveModel(*model);
    } catch (const std::runtime_error& rejection) {
        std::printf("FAIL: contract rejected the model: %s\n",
                    rejection.what());
        mj_deleteModel(model);
        return 1;
    }
    mjData* data = mj_makeData(model);

    // --- parity over all cases --------------------------------------------
    const std::vector<ParityCaseState> cases = BuildCases();
    for (const ParityCaseState& c : cases) {
        ApplyState(*model, *data, ids, c);
        const Eigen::Matrix3d world_R_mount =
            c.world_q_mount.toRotationMatrix();

        const Pose right_tool = kinematics_right.ToolPoseInMount(
            Arm::kRight, c.right_q_rad, c.left_q_rad);
        const Pose left_tool = kinematics_right.ToolPoseInMount(
            Arm::kLeft, c.right_q_rad, c.left_q_rad);

        struct ArmSide {
            const char* name;
            const ArmModelIds* mj;
            Arm arm;
            const Pose* tool;
            DualArmKinematics* kinematics;
            const Eigen::Matrix<double, 7, 1>* q;
        };
        const ArmSide sides[2] = {
            {"right", &ids.right, Arm::kRight, &right_tool, &kinematics_right,
             &c.right_q_rad},
            {"left", &ids.left, Arm::kLeft, &left_tool, &kinematics_left,
             &c.left_q_rad},
        };
        for (const ArmSide& side : sides) {
            const std::string label = "case " + c.name + " arm " + side.name;

            // Base pose: world_T_mount * mount_T_base vs the MJCF body.
            const pinocchio::SE3& mount_T_base =
                kinematics_right.MountFromBase(side.arm);
            ComparePose(
                label + " base",
                Eigen::Vector3d(data->xpos + 3 * side.mj->base_body_id),
                RowMajor3(data->xmat + 9 * side.mj->base_body_id),
                world_R_mount * mount_T_base.translation() +
                    c.world_p_mount_m,
                world_R_mount * mount_T_base.rotation());

            // TCP pose: world_T_mount * mount_T_tcp vs the MJCF site.
            ComparePose(
                label + " tcp",
                Eigen::Vector3d(data->site_xpos + 3 * side.mj->tcp_site_id),
                RowMajor3(data->site_xmat + 9 * side.mj->tcp_site_id),
                world_R_mount * side.tool->position + c.world_p_mount_m,
                world_R_mount * side.tool->rotation);

            // Jacobian: MuJoCo forward differences vs the production
            // world columns.
            const Eigen::Matrix<double, 6, 7> fd =
                FiniteDifferenceWorldJacobian(*model, *data, *side.mj);
            const Eigen::Matrix<double, 6, 7> analytic =
                AnalyticWorldJacobian(*side.kinematics, *side.q,
                                      world_R_mount, workspace);
            CompareJacobians(label + " jacobian", fd, analytic, true);
        }
    }
    std::printf("parity maxima over %zu cases: position %.3e m, "
                "orientation-log %.3e rad, FD-Jacobian component %.3e\n",
                cases.size(), max_position_error_m,
                max_orientation_error_rad, max_jacobian_error);

    // --- mutation 1: swapped joints 2/4 fed to MuJoCo ----------------------
    // Emulates a wrong joint-order mapping (prediction: error 1e-2..1e0 m
    // at a non-degenerate configuration).
    {
        ParityCaseState swapped = cases[1]; // retracted_translated_yawed_mount
        std::swap(swapped.right_q_rad[1], swapped.right_q_rad[3]);
        ApplyState(*model, *data, ids, swapped);
        const Pose tool = kinematics_right.ToolPoseInMount(
            Arm::kRight, cases[1].right_q_rad, cases[1].left_q_rad);
        const Eigen::Matrix3d world_R_mount =
            cases[1].world_q_mount.toRotationMatrix();
        const double position_error =
            (Eigen::Vector3d(data->site_xpos + 3 * ids.right.tcp_site_id) -
             (world_R_mount * tool.position + cases[1].world_p_mount_m))
                .norm();
        std::printf("mutation swapped-joints-2/4 TCP error: %.3e m\n",
                    position_error);
        Check(position_error > 1e-2,
              "swapped joints 2/4 must break TCP parity by >1e-2 m "
              "(observed " + std::to_string(position_error) + " m)");
    }

    const std::string base_xml = ReadFile(SIM_MODEL_XML_PATH);
    Check(!base_xml.empty(), "model XML is readable");

    // --- mutation 2: inverted right_joint_5 axis ---------------------------
    {
        std::string mutated = base_xml;
        const int done = Replace(
            mutated, "<joint name=\"right_joint_5\" axis=\"0 0 1\" />",
            "<joint name=\"right_joint_5\" axis=\"0 0 -1\" />", 1);
        Check(done == 1, "inverted-axis mutation pattern matched once");
        std::string load_error;
        mjModel* bad = LoadFromText(mutated, &load_error);
        Check(bad != nullptr,
              "inverted-axis model loads (mutation targets parity, not the "
              "compiler): " + load_error);
        if (bad != nullptr) {
            // First line of defense: the structural contract rejects it.
            bool rejected = false;
            try {
                (void)ValidateAndResolveModel(*bad);
            } catch (const std::runtime_error& rejection) {
                rejected = std::string(rejection.what()).find(
                               "right_joint_5") != std::string::npos;
            }
            Check(rejected, "ModelContract rejects the inverted axis "
                            "naming right_joint_5");

            // Second line: bypass the contract (ids from the pristine
            // model are layout-identical — only an attribute changed) and
            // show THIS gate catches it: the FD column flips sign,
            // error ~2*||column||.
            mjData* bad_data = mj_makeData(bad);
            ApplyState(*bad, *bad_data, ids, cases[1]);
            const Eigen::Matrix<double, 6, 7> fd =
                FiniteDifferenceWorldJacobian(*bad, *bad_data, ids.right);
            const Eigen::Matrix<double, 6, 7> analytic =
                AnalyticWorldJacobian(
                    kinematics_right, cases[1].right_q_rad,
                    cases[1].world_q_mount.toRotationMatrix(), workspace);
            const double joint5_error =
                (fd.col(4) - analytic.col(4)).cwiseAbs().maxCoeff();
            std::printf("mutation inverted-axis joint-5 max component "
                        "error: %.3e\n", joint5_error);
            Check(joint5_error > 0.5,
                  "inverted right_joint_5 axis must flip the FD column "
                  "(expected error ~2*||column||, observed " +
                      std::to_string(joint5_error) + ")");
            mj_deleteData(bad_data);
            mj_deleteModel(bad);
        }
    }

    // --- mutation 3: left/right mount placement swap -----------------------
    // The Task 2 structural gates are placement-blind, so the contract
    // ACCEPTS this model; only this parity gate can catch it. Pre-registered
    // magnitudes: 2*0.0567075 = 0.1134 m, 2*1.2085 = 2.417 rad.
    {
        const std::string right_placement =
            "pos=\"0 -0.0567075 0\" "
            "quat=\"0.8229284378208791 0.5681450397791815 0 0\"";
        const std::string left_placement =
            "pos=\"0 0.0567075 0\" "
            "quat=\"0.8229284378208791 -0.5681450397791815 0 0\"";
        std::string mutated = base_xml;
        int done = Replace(mutated, right_placement, "@@SWAP@@", 1);
        done += Replace(mutated, left_placement, right_placement, 1);
        done += Replace(mutated, "@@SWAP@@", left_placement, 1);
        Check(done == 3, "mount-swap mutation patterns matched (3 edits)");
        std::string load_error;
        mjModel* bad = LoadFromText(mutated, &load_error);
        Check(bad != nullptr, "mount-swapped model loads: " + load_error);
        if (bad != nullptr) {
            bool accepted = true;
            try {
                (void)ValidateAndResolveModel(*bad);
            } catch (const std::runtime_error&) {
                accepted = false;
            }
            Check(accepted, "structural contract is placement-blind and "
                            "accepts the mount swap (Task 2 probe held)");

            mjData* bad_data = mj_makeData(bad);
            ApplyState(*bad, *bad_data, ids, cases[0]);
            const pinocchio::SE3& mount_T_right =
                kinematics_right.MountFromBase(Arm::kRight);
            const double position_error =
                (Eigen::Vector3d(bad_data->xpos +
                                 3 * ids.right.base_body_id) -
                 mount_T_right.translation())
                    .norm(); // case 0 mount = identity
            const double orientation_error = RelativeAngleRad(
                RowMajor3(bad_data->xmat + 9 * ids.right.base_body_id),
                mount_T_right.rotation());
            std::printf("mutation mount-swap right base errors: %.6f m, "
                        "%.6f rad\n", position_error, orientation_error);
            Check(position_error > 0.11 && position_error < 0.12,
                  "mount swap base position error ~0.1134 m (observed " +
                      std::to_string(position_error) + ")");
            Check(orientation_error > 2.40 && orientation_error < 2.44,
                  "mount swap base orientation error ~2.417 rad (observed " +
                      std::to_string(orientation_error) + ")");
            mj_deleteData(bad_data);
            mj_deleteModel(bad);
        }
    }

    mj_deleteData(data);
    mj_deleteModel(model);

    if (failures == 0) {
        std::printf("PASS: MuJoCo and production Pinocchio kinematics agree "
                    "within 1e-8 m / 1e-8 rad / 1e-6, and all mutations are "
                    "detected\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", failures);
    return 1;
}
