//
// MujocoBackend tests (Plan 02 Task 4): the physics-stepping and
// Kortex-wire-format adapter between the execution core and the MuJoCo
// plant.
//
// What is under test, per the accepted shape proposal:
//   - construction-time validation (control period, substep count),
//     validated once, never per tick;
//   - Seed = keyframe reset + Mount prescription + forward, time stays 0;
//   - one Exchange advances simulated time by exactly one 0.002 s control
//     tick regardless of substep count, with the 14 controls written once
//     (deg -> rad at the adapter boundary, contract section 2) and held
//     constant across all substeps (ZOH, the analogue of the hardware
//     low-level servo holding the last 500 Hz frame);
//   - feedback ordering follows the resolved model contract (command on
//     right joint 3 moves right joint 3, not any left joint);
//   - measured degrees come back in BOTH hardware shapes: wrapped
//     actuator-reported [0, 360) (what ArmExecutionInput/ResolveStop eat)
//     and continuous degrees (validation), consistent modulo 360;
//   - the Mount is a world-welded mocap body: its pose is exactly the
//     pose written for the tick, with no drift at all, and the arms hang
//     off it with REAL gravity load — each servo settles at the droop
//     -qfrc_bias/kp predicted from MuJoCo's own bias torques and the
//     model's own gains. Both are properties the superseded freejoint
//     prescription could not have shown: it drifted 2.45e-6 m inside
//     every substep and left the arms in free fall (a held command moved
//     joint 2 by exactly 0.0 rad, README probe 2026-08-17);
//   - a Mount that MOVES between ticks is carried into the plant on the
//     tick it is written, not the next one: a constant-velocity
//     translation offsets every body by exactly p(t_k) against an
//     otherwise identical stationary run, and a rotating, translating
//     Mount reproduces world_T_mount(t_k) * mount_T_tcp(q) computed by
//     the production Pinocchio kinematics. Both also record what a
//     one-tick-late Mount write would have cost (|v|*dt, about 1 mm),
//     because every other check here holds the Mount still and would not
//     have noticed;
//   - non-finite commands/Mount inputs are rejected before any physics
//     step, and a diverged (non-finite or > mjMAXVAL) state throws after
//     the very substep that produced it — for substeps > 1 too, before
//     any further substep could run (MuJoCo would otherwise silently
//     auto-reset on its own mjWARN_BADQPOS check at the start of the
//     next substep and all later feedback would be garbage);
//   - repeated runs on the same build and substep setting produce
//     byte-identical qpos/qvel traces (pre-registered prediction: nothing
//     in the loop reads wall-clock time).
//
// Evidence class: unit test (generic position-servo plant; kp/kv are
// declared generic mechanics, not Kinova parameters).
//

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Config.h"
#include "Dynamics.h"
#include "Kinematics.h" // production FK, the independent check on the moving Mount
#include "ModelContract.h"
#include "MujocoBackend.h"
#include "State.h"

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

    constexpr double kControlDt = 0.002; // s, the 500 Hz control tick
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    std::string ModelPath() { return SIM_MODEL_XML_PATH; }

    // Home keyframe joint angles in continuous degrees, read from the
    // loaded model itself (key_qpos at the contract's resolved addresses)
    // so the expectation does not repeat the backend's feedback path.
    struct HomeState {
        JointVector right_deg{};
        JointVector left_deg{};
    };

    HomeState ReadHomeDegrees(const MujocoBackend& backend)
    {
        const mjModel& model = backend.Model();
        const DualModelContract& ids = backend.Contract();
        HomeState home;
        const int key = 0; // the one "home" keyframe
        for (int i = 0; i < 7; ++i) {
            home.right_deg[i] =
                model.key_qpos[key * model.nq + ids.right.joint_qpos_adr[i]] *
                kRadToDeg;
            home.left_deg[i] =
                model.key_qpos[key * model.nq + ids.left.joint_qpos_adr[i]] *
                kRadToDeg;
        }
        return home;
    }

    CartesianPose IdentityPose() { return CartesianPose{}; }

    CartesianPose OffsetRotatedPose()
    {
        CartesianPose pose;
        pose.position_m = Eigen::Vector3d(0.3, -0.2, 0.5);
        pose.rotation =
            Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX())
                .toRotationMatrix();
        return pose;
    }

    double Mod360(double deg)
    {
        double wrapped = std::fmod(deg, 360.0);
        if (wrapped < 0.0)
            wrapped += 360.0;
        if (wrapped >= 360.0)
            wrapped = 0.0;
        return wrapped;
    }

    // Every feedback invariant that must hold on EVERY exchange: finite
    // values, wrapped position in [0, 360), wrapped == continuous mod 360.
    void CheckFeedbackInvariants(const SimArmFeedback& fb,
                                 const std::string& where)
    {
        const SimJointFeedback* arms[2] = {&fb.right, &fb.left};
        const char* names[2] = {"right", "left"};
        for (int a = 0; a < 2; ++a) {
            for (int i = 0; i < 7; ++i) {
                const double wrapped = arms[a]->measured_position_deg[i];
                const double continuous = arms[a]->continuous_position_deg[i];
                const double velocity = arms[a]->measured_velocity_deg_s[i];
                Check(std::isfinite(wrapped) && std::isfinite(continuous) &&
                          std::isfinite(velocity),
                      where + ": " + names[a] + " joint " +
                          std::to_string(i + 1) + " feedback finite");
                Check(wrapped >= 0.0 && wrapped < 360.0,
                      where + ": " + names[a] + " joint " +
                          std::to_string(i + 1) + " wrapped in [0,360)");
                Check(std::abs(wrapped - Mod360(continuous)) < 1e-9,
                      where + ": " + names[a] + " joint " +
                          std::to_string(i + 1) +
                          " wrapped == continuous mod 360");
            }
        }
    }

    template <typename Fn>
    bool Throws(Fn&& fn, const char* type)
    {
        try {
            fn();
        } catch (const std::invalid_argument&) {
            return std::string(type) == "invalid_argument";
        } catch (const std::logic_error&) {
            // invalid_argument derives from logic_error; reached only for
            // other logic errors.
            return std::string(type) == "logic_error";
        } catch (const std::runtime_error&) {
            return std::string(type) == "runtime_error";
        }
        return false;
    }

    // Runs fn, returning the std::runtime_error message it threw, or ""
    // if it did not throw one — lets a check assert WHICH guard branch
    // fired, not merely that something did.
    template <typename Fn>
    std::string RuntimeErrorMessage(Fn&& fn)
    {
        try {
            fn();
        } catch (const std::runtime_error& error) {
            return error.what();
        }
        return "";
    }
} // namespace

int main()
{
    const HomeState home = [] {
        MujocoBackend probe(ModelPath(), kControlDt, 1);
        return ReadHomeDegrees(probe);
    }();

    // ---- Construction-time validation (validate once, at the boundary) ----
    Check(Throws([] { MujocoBackend b(ModelPath(), 0.0, 1); },
                 "invalid_argument"),
          "control_dt_s = 0 rejected");
    Check(Throws([] { MujocoBackend b(ModelPath(), -0.002, 1); },
                 "invalid_argument"),
          "negative control_dt_s rejected");
    Check(Throws(
              [] {
                  MujocoBackend b(ModelPath(),
                                  std::numeric_limits<double>::quiet_NaN(), 1);
              },
              "invalid_argument"),
          "NaN control_dt_s rejected");
    Check(Throws([] { MujocoBackend b(ModelPath(), kControlDt, 0); },
                 "invalid_argument"),
          "physics_substeps = 0 rejected");
    Check(Throws([] { MujocoBackend b(ModelPath(), kControlDt, -4); },
                 "invalid_argument"),
          "negative physics_substeps rejected");
    Check(Throws([] { MujocoBackend b("/nonexistent/model.xml", kControlDt,
                                      1); },
                 "runtime_error"),
          "unloadable model path rejected");

    // ---- Exchange before Seed is rejected ----
    {
        MujocoBackend backend(ModelPath(), kControlDt, 1);
        Check(Throws(
                  [&] {
                      backend.Exchange(home.right_deg, home.left_deg,
                                       IdentityPose());
                  },
                  "logic_error"),
              "Exchange before Seed rejected");
    }

    // ---- Seed: keyframe state, prescribed Mount, time stays 0 ----
    for (const int substeps : {1, 4}) {
        MujocoBackend backend(ModelPath(), kControlDt, substeps);
        Check(backend.physics_substeps() == substeps, "substeps stored");
        Check(std::abs(backend.timestep_s() - kControlDt / substeps) <
                  1e-18,
              "opt.timestep = control_dt_s / physics_substeps");

        const CartesianPose mount = OffsetRotatedPose();
        const SimArmFeedback fb = backend.Seed(mount);
        Check(fb.simulation_time_s == 0.0,
              "Seed does not advance time (substeps " +
                  std::to_string(substeps) + ")");
        Check(backend.Data().time == 0.0, "mjData.time stays 0 after Seed");
        CheckFeedbackInvariants(fb, "seed");
        for (int i = 0; i < 7; ++i) {
            Check(std::abs(fb.right.continuous_position_deg[i] -
                           home.right_deg[i]) < 1e-12,
                  "Seed right joint " + std::to_string(i + 1) +
                      " continuous deg == home keyframe");
            Check(std::abs(fb.left.continuous_position_deg[i] -
                           home.left_deg[i]) < 1e-12,
                  "Seed left joint " + std::to_string(i + 1) +
                      " continuous deg == home keyframe");
            Check(fb.right.measured_velocity_deg_s[i] == 0.0 &&
                      fb.left.measured_velocity_deg_s[i] == 0.0,
                  "Seed velocities zero");
        }
        // Home joint 4 is -130 deg continuous -> 230 deg actuator-reported.
        Check(std::abs(fb.right.measured_position_deg[3] -
                       (home.right_deg[3] + 360.0)) < 1e-9,
              "bounded joint 4 reports wrapped 230 deg for continuous -130");

        // The Mount mocap row holds the prescribed pose: position, then
        // unit quaternion in MuJoCo's w,x,y,z order (State.h/Vicon wire
        // order is x,y,z,w — this is the order trap the backend owns).
        // Seeding must also overwrite the keyframe's identity mocap pose.
        const DualModelContract& ids = backend.Contract();
        const double* mocap_pos =
            backend.Data().mocap_pos + 3 * ids.mount_mocap_id;
        const double* mocap_quat =
            backend.Data().mocap_quat + 4 * ids.mount_mocap_id;
        const Eigen::Quaterniond expected(mount.rotation);
        Check(mocap_pos[0] == mount.position_m[0] &&
                  mocap_pos[1] == mount.position_m[1] &&
                  mocap_pos[2] == mount.position_m[2],
              "Seed writes Mount position");
        const Eigen::Vector4d observed_quat(mocap_quat[0], mocap_quat[1],
                                            mocap_quat[2], mocap_quat[3]);
        const Eigen::Vector4d expected_quat(expected.w(), expected.x(),
                                            expected.y(), expected.z());
        Check(std::min((observed_quat - expected_quat).norm(),
                       (observed_quat + expected_quat).norm()) < 1e-15,
              "Seed writes Mount quaternion in w,x,y,z");

        // ... and the arms actually hang off that pose: the base bodies
        // must sit at world_T_mount * mount_T_base, which for this Mount
        // is 0.3, -0.2, 0.5 plus a rotated 0.0567 m offset. Checking a
        // body world position (not just the mocap row) is what proves the
        // mocap write is read by the kinematics at all.
        const double* right_base =
            backend.Data().xpos + 3 * ids.right.base_body_id;
        Check(std::abs((Eigen::Vector3d(right_base[0], right_base[1],
                                        right_base[2]) -
                        mount.position_m)
                           .norm() -
                       0.0567075) < 1e-12,
              "right base body hangs 0.0567075 m off the seeded Mount pose");
    }

    // ---- One Exchange advances exactly one control tick; controls are
    //      written once (deg -> rad) and held across substeps ----
    for (const int substeps : {1, 4}) {
        MujocoBackend backend(ModelPath(), kControlDt, substeps);
        backend.Seed(IdentityPose());

        JointVector right_cmd = home.right_deg;
        JointVector left_cmd = home.left_deg;
        for (int i = 0; i < 7; ++i) {
            right_cmd[i] += 0.5 * (i + 1); // distinct per joint and arm
            left_cmd[i] -= 0.25 * (i + 1);
        }
        const SimArmFeedback fb =
            backend.Exchange(right_cmd, left_cmd, IdentityPose());
        Check(std::abs(fb.simulation_time_s - kControlDt) <= 1e-15,
              "one Exchange advances 0.002 s (substeps " +
                  std::to_string(substeps) + ")");
        CheckFeedbackInvariants(fb, "first exchange");

        // ctrl read back AFTER all substeps: proves both the deg -> rad
        // boundary conversion and that nothing rewrote the controls
        // mid-tick (single write, no callbacks).
        const DualModelContract& ids = backend.Contract();
        for (int i = 0; i < 7; ++i) {
            Check(backend.Data().ctrl[ids.right.actuator_id[i]] ==
                      right_cmd[i] * kDegToRad,
                  "right ctrl " + std::to_string(i + 1) +
                      " == deg2rad(command), constant across substeps");
            Check(backend.Data().ctrl[ids.left.actuator_id[i]] ==
                      left_cmd[i] * kDegToRad,
                  "left ctrl " + std::to_string(i + 1) +
                      " == deg2rad(command), constant across substeps");
        }

        double time = fb.simulation_time_s;
        for (int k = 0; k < 4; ++k)
            time = backend.Exchange(right_cmd, left_cmd, IdentityPose())
                       .simulation_time_s;
        Check(std::abs(time - 5.0 * kControlDt) <= 1e-14,
              "five Exchanges advance 0.010 s");
    }

    // ---- The Mount does not drift AT ALL: welded to world, it is moved
    //      only by the backend's own write, so its body pose after any
    //      number of ticks is bit-identical to the requested pose. The
    //      superseded freejoint had to be re-pinned every substep and
    //      still drifted g*(dt/N)^2 = 2.45e-6 m inside each one; here the
    //      tolerance is exact equality, which that plant could not have
    //      met. The BODY frame is checked (not the mocap row) so the test
    //      would catch a write that never reached the kinematics. ----
    {
        MujocoBackend backend(ModelPath(), kControlDt, 4);
        const CartesianPose mount = OffsetRotatedPose();
        backend.Seed(mount);
        const int mount_body = backend.Contract().mount_body_id;
        bool exact = true;
        for (int k = 0; k < 100; ++k) {
            backend.Exchange(home.right_deg, home.left_deg, mount);
            const double* xpos = backend.Data().xpos + 3 * mount_body;
            if (xpos[0] != mount.position_m[0] ||
                xpos[1] != mount.position_m[1] ||
                xpos[2] != mount.position_m[2])
                exact = false;
            const Eigen::Matrix3d observed =
                Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                    backend.Data().xmat + 9 * mount_body);
            if ((observed - mount.rotation).cwiseAbs().maxCoeff() > 1e-15)
                exact = false;
        }
        Check(exact,
              "Mount body pose stays exactly the prescribed pose over 100 "
              "ticks (nothing in the physics can move a world-welded body)");
    }

    // ---- A MOVING Mount, stepped: the tick's pose reaches the plant on
    //      THAT tick ----
    //
    //      Every other check in this file holds the Mount still, so a
    //      Mount write that arrived one tick late — or after the substeps
    //      instead of before them — would pass all of them and silently
    //      bias every moving-base result by |v|*dt. Two checks, both
    //      pre-registered with the magnitude that would expose the bug.
    //
    //      A. Constant-velocity translation, against a stationary twin.
    //      A frame translating at constant velocity is inertial, and a
    //      mocap Mount transmits no force to the links, so the two runs
    //      solve the same joint problem: identical joint trajectories,
    //      and every body of the moving run sitting exactly
    //      p(t_k) = p0 + v*t_k away from its stationary twin. A one-tick
    //      lag would leave the offset at p(t_{k-1}), short by
    //      |v|*dt = 1.14e-3 m — six orders above the 1e-9 m gate. (This
    //      is Galilean invariance, not an artifact: at CONSTANT velocity
    //      the real rig would also feel nothing. Under acceleration it
    //      would, and this plant still would not — the documented mocap
    //      limitation, not something this check licenses.)
    {
        const Eigen::Vector3d start(0.30, -0.20, 0.70);   // m
        const Eigen::Vector3d velocity(0.40, -0.30, 0.25); // m/s
        const auto TranslatedPose = [&](int tick) {
            CartesianPose pose;
            pose.position_m =
                start + velocity * (static_cast<double>(tick) * kControlDt);
            return pose;
        };

        MujocoBackend fixed(ModelPath(), kControlDt, 4);
        MujocoBackend moving(ModelPath(), kControlDt, 4);
        fixed.Seed(IdentityPose());
        moving.Seed(TranslatedPose(0));
        const DualModelContract& ids = moving.Contract();
        const int sites[2] = {ids.right.tcp_site_id, ids.left.tcp_site_id};
        const int bodies[2] = {ids.right.base_body_id, ids.left.base_body_id};

        double max_joint_gap_deg = 0.0;
        double max_offset_error_m = 0.0;
        double min_late_error_m = std::numeric_limits<double>::max();
        for (int k = 0; k < 50; ++k) {
            const SimArmFeedback fb_fixed =
                fixed.Exchange(home.right_deg, home.left_deg, IdentityPose());
            const SimArmFeedback fb_moving = moving.Exchange(
                home.right_deg, home.left_deg, TranslatedPose(k));
            for (int i = 0; i < 7; ++i)
                max_joint_gap_deg = std::max(
                    {max_joint_gap_deg,
                     std::abs(fb_fixed.right.continuous_position_deg[i] -
                              fb_moving.right.continuous_position_deg[i]),
                     std::abs(fb_fixed.left.continuous_position_deg[i] -
                              fb_moving.left.continuous_position_deg[i])});

            const Eigen::Vector3d expected = TranslatedPose(k).position_m;
            const Eigen::Vector3d late =
                TranslatedPose(std::max(k - 1, 0)).position_m;
            for (int s : sites) {
                const Eigen::Vector3d offset =
                    Eigen::Vector3d(moving.Data().site_xpos + 3 * s) -
                    Eigen::Vector3d(fixed.Data().site_xpos + 3 * s);
                max_offset_error_m =
                    std::max(max_offset_error_m, (offset - expected).norm());
                if (k > 0)
                    min_late_error_m = std::min(min_late_error_m,
                                                (offset - late).norm());
            }
            for (int b : bodies) {
                const Eigen::Vector3d offset =
                    Eigen::Vector3d(moving.Data().xpos + 3 * b) -
                    Eigen::Vector3d(fixed.Data().xpos + 3 * b);
                max_offset_error_m =
                    std::max(max_offset_error_m, (offset - expected).norm());
            }
        }
        std::printf("  moving Mount (translation): joint gap %.3e deg, "
                    "carry error %.3e m, one-tick-late error %.3e m\n",
                    max_joint_gap_deg, max_offset_error_m, min_late_error_m);
        Check(max_joint_gap_deg < 1e-9,
              "a constant-velocity Mount leaves the joint trajectory "
              "unchanged (inertial frame, no force from a mocap body)");
        Check(max_offset_error_m < 1e-9,
              "every arm body/TCP is carried by exactly the Mount "
              "translation of the CURRENT tick");
        Check(min_late_error_m > 1e-4,
              "the same check against the PREVIOUS tick's Mount pose fails "
              "by |v|*dt, so a one-tick-late Mount write cannot pass");
    }

    //      B. Rotating and translating Mount, against the production
    //      Pinocchio kinematics — the independent pipeline the Task 3
    //      parity gate uses, but now with physics running and the Mount
    //      pose changing every tick:
    //
    //          world_T_tcp = world_T_mount(t_k) * mount_T_tcp(q)
    //
    //      Rotation matters on its own: it changes the gravity direction
    //      in the arms' frame, so unlike case A the joint trajectory is
    //      genuinely different and a wrong composition ORDER (or a stale
    //      Mount) cannot be hidden by symmetry. Site frames are read from
    //      a forwarded copy of mjData so they correspond to the same
    //      post-tick joint angles the feedback reports.
    {
        Dynamics dynamics(SIM_URDF_PATH);
        DualArmKinematics kinematics(dynamics, Arm::kRight,
                                     config::kLeftNominalRad,
                                     config::kRightBaseFrame,
                                     config::kRightEndEffectorFrame);

        const Eigen::Vector3d start(0.15, -0.10, 0.80);      // m
        const Eigen::Vector3d velocity(0.20, 0.10, -0.05);   // m/s
        const Eigen::Vector3d axis =
            Eigen::Vector3d(0.3, -0.5, 1.0).normalized();    // world axis
        const double rate_rad_s = 0.8;
        const auto MovingPose = [&](int tick) {
            const double t = static_cast<double>(tick) * kControlDt;
            CartesianPose pose;
            pose.position_m = start + velocity * t;
            pose.rotation =
                Eigen::AngleAxisd(rate_rad_s * t, axis).toRotationMatrix();
            return pose;
        };

        MujocoBackend backend(ModelPath(), kControlDt, 1);
        backend.Seed(MovingPose(0));
        const DualModelContract& ids = backend.Contract();
        mjData* probe = mj_makeData(&backend.Model());

        double max_parity_error_m = 0.0;
        double min_late_error_m = std::numeric_limits<double>::max();
        for (int k = 0; k < 60; ++k) {
            const CartesianPose mount = MovingPose(k);
            const SimArmFeedback fb =
                backend.Exchange(home.right_deg, home.left_deg, mount);
            mj_copyData(probe, &backend.Model(), &backend.Data());
            mj_forward(&backend.Model(), probe);

            Eigen::Matrix<double, 7, 1> right_q, left_q;
            for (int i = 0; i < 7; ++i) {
                right_q[i] = fb.right.continuous_position_deg[i] * kDegToRad;
                left_q[i] = fb.left.continuous_position_deg[i] * kDegToRad;
            }
            const CartesianPose late = MovingPose(std::max(k - 1, 0));
            const Arm arms[2] = {Arm::kRight, Arm::kLeft};
            const int tcp_sites[2] = {ids.right.tcp_site_id,
                                      ids.left.tcp_site_id};
            for (int a = 0; a < 2; ++a) {
                const Pose tool =
                    kinematics.ToolPoseInMount(arms[a], right_q, left_q);
                const Eigen::Vector3d observed(probe->site_xpos +
                                               3 * tcp_sites[a]);
                const Eigen::Vector3d expected =
                    mount.rotation * tool.position + mount.position_m;
                max_parity_error_m =
                    std::max(max_parity_error_m, (observed - expected).norm());
                if (k > 0) {
                    const Eigen::Vector3d stale =
                        late.rotation * tool.position + late.position_m;
                    min_late_error_m = std::min(min_late_error_m,
                                                (observed - stale).norm());
                }
            }
        }
        mj_deleteData(probe);
        std::printf("  moving Mount (rotation+translation): parity %.3e m, "
                    "one-tick-late error %.3e m\n",
                    max_parity_error_m, min_late_error_m);
        Check(max_parity_error_m < 1e-9,
              "stepped world TCP == world_T_mount(t_k) * mount_T_tcp(q) "
              "under a rotating, translating Mount (production Pinocchio "
              "kinematics, independent of MuJoCo)");
        Check(min_late_error_m > 1e-4,
              "the same composition with the PREVIOUS tick's Mount pose "
              "fails, so a stale Mount write cannot pass this check");
    }

    // ---- Feedback/command ordering follows the contract: a command on
    //      right joint 3 moves right joint 3 and no left joint ----
    {
        MujocoBackend backend(ModelPath(), kControlDt, 1);
        backend.Seed(IdentityPose());
        JointVector right_cmd = home.right_deg;
        right_cmd[2] += 5.0;
        SimArmFeedback fb{};
        for (int k = 0; k < 150; ++k)
            fb = backend.Exchange(right_cmd, home.left_deg, IdentityPose());
        Check(fb.right.continuous_position_deg[2] - home.right_deg[2] > 2.0,
              "right joint 3 tracked its +5 deg command (observed +" +
                  std::to_string(fb.right.continuous_position_deg[2] -
                                 home.right_deg[2]) +
                  " deg)");
        for (int i = 0; i < 7; ++i)
            Check(std::abs(fb.left.continuous_position_deg[i] -
                           home.left_deg[i]) < 1.0,
                  "left joint " + std::to_string(i + 1) +
                      " undisturbed by the right command");
        CheckFeedbackInvariants(fb, "ordering");
    }

    // ---- Continuous joint crossing 0: continuous goes negative, wrapped
    //      stays in [0, 360) like the hardware wire ----
    {
        MujocoBackend backend(ModelPath(), kControlDt, 1);
        backend.Seed(IdentityPose());
        JointVector right_cmd = home.right_deg;
        right_cmd[0] = home.right_deg[0] - 10.0; // joint 1 home = 0 deg
        SimArmFeedback fb{};
        for (int k = 0; k < 300; ++k)
            fb = backend.Exchange(right_cmd, home.left_deg, IdentityPose());
        Check(fb.right.continuous_position_deg[0] < -5.0,
              "continuous joint 1 measured negative continuous degrees");
        Check(fb.right.measured_position_deg[0] > 345.0 &&
                  fb.right.measured_position_deg[0] < 360.0,
              "joint 1 actuator-reported degrees wrapped into [345, 360)");
        CheckFeedbackInvariants(fb, "wrap");
    }

    // ---- Non-finite inputs are rejected before any physics step ----
    {
        MujocoBackend backend(ModelPath(), kControlDt, 4);
        backend.Seed(IdentityPose());
        const double time_before =
            backend.Exchange(home.right_deg, home.left_deg, IdentityPose())
                .simulation_time_s;

        JointVector bad_right = home.right_deg;
        bad_right[2] = std::numeric_limits<double>::quiet_NaN();
        Check(Throws(
                  [&] {
                      backend.Exchange(bad_right, home.left_deg,
                                       IdentityPose());
                  },
                  "invalid_argument"),
              "NaN command rejected");
        JointVector bad_left = home.left_deg;
        bad_left[6] = std::numeric_limits<double>::infinity();
        Check(Throws(
                  [&] {
                      backend.Exchange(home.right_deg, bad_left,
                                       IdentityPose());
                  },
                  "invalid_argument"),
              "infinite command rejected");
        CartesianPose bad_pose;
        bad_pose.position_m[1] = std::numeric_limits<double>::quiet_NaN();
        Check(Throws(
                  [&] {
                      backend.Exchange(home.right_deg, home.left_deg,
                                       bad_pose);
                  },
                  "invalid_argument"),
              "NaN Mount position rejected");
        Check(backend.Data().time == time_before,
              "rejected inputs did not advance physics");
        const double time_after =
            backend.Exchange(home.right_deg, home.left_deg, IdentityPose())
                .simulation_time_s;
        Check(std::abs(time_after - time_before - kControlDt) <= 1e-15,
              "backend still healthy after rejected inputs");
    }

    // ---- Diverged state stops before another step: an absurd control
    //      period lets gravity integrate the arm joints past mjMAXVAL
    //      (1e10). With the mocap Mount the joints are the only state
    //      left, and they need two steps of 1e6 s to get there (probe
    //      2026-08-17: joint 1 reaches 8.2e4 rad after step 1, -1.1e15
    //      after step 2), so the second Exchange is the one that must
    //      throw — rather than let MuJoCo's own BADQPOS check silently
    //      auto-reset the state on the step after that. ----
    {
        MujocoBackend backend(ModelPath(), 1e6, 1);
        backend.Seed(IdentityPose());
        backend.Exchange(home.right_deg, home.left_deg, IdentityPose());
        Check(Throws(
                  [&] {
                      backend.Exchange(home.right_deg, home.left_deg,
                                       IdentityPose());
                  },
                  "runtime_error"),
              "diverged post-step state throws");
    }

    // ---- With substeps > 1 the scan runs after EVERY substep: the same
    //      absurd period at 4 substeps (per-substep dt = 2.5e5 s) diverges
    //      during substep 2, so the throw must come from the qpos scan
    //      with exactly two substeps run and no MuJoCo warning raised. A
    //      post-loop scan would instead let mj_step notice the corpse at
    //      the start of substep 3, auto-reset it (raising a warning
    //      counter) and run the remaining substeps on the reset state. ----
    {
        MujocoBackend backend(ModelPath(), 1e6, 4);
        backend.Seed(IdentityPose());
        const std::string message = RuntimeErrorMessage([&] {
            backend.Exchange(home.right_deg, home.left_deg,
                             IdentityPose());
        });
        Check(!message.empty(),
              "diverged state with 4 substeps throws runtime_error");
        Check(message.find("qpos[") != std::string::npos,
              "throw came from the post-substep qpos scan, not the "
              "warning-counter branch (got: " + message + ")");
        Check(backend.Data().time == 2.0 * backend.timestep_s(),
              "no substep ran after the one that diverged");
        Check(backend.Data().warning[mjWARN_BADQPOS].number == 0 &&
                  backend.Data().warning[mjWARN_BADQVEL].number == 0 &&
                  backend.Data().warning[mjWARN_BADQACC].number == 0,
              "MuJoCo never saw (so never auto-reset) the diverged state");
    }

    // The warning-counter branch of RequireHealthyState is deliberately
    // left unexercised here: its previous trigger was a finite but
    // beyond-mjMAXVAL Mount TWIST written into the freejoint's qvel, and
    // a mocap Mount has no velocity to write, so no input this boundary
    // accepts can reach it (see the MujocoBackend header note).

    // ---- Seed is a full deterministic reset ----
    {
        MujocoBackend backend(ModelPath(), kControlDt, 2);
        const SimArmFeedback first = backend.Seed(OffsetRotatedPose());
        std::vector<double> qpos_first(
            backend.Data().qpos, backend.Data().qpos + backend.Model().nq);
        for (int k = 0; k < 20; ++k)
            backend.Exchange(home.right_deg, home.left_deg,
                             OffsetRotatedPose());
        const SimArmFeedback again = backend.Seed(OffsetRotatedPose());
        Check(again.simulation_time_s == 0.0, "re-Seed resets time to 0");
        Check(std::memcmp(qpos_first.data(), backend.Data().qpos,
                          sizeof(double) * backend.Model().nq) == 0,
              "re-Seed restores the identical qpos state");
        for (int i = 0; i < 7; ++i)
            Check(again.right.continuous_position_deg[i] ==
                          first.right.continuous_position_deg[i] &&
                      again.left.continuous_position_deg[i] ==
                          first.left.continuous_position_deg[i],
                  "re-Seed feedback identical to first Seed");
    }

    // ---- Determinism: byte-identical qpos/qvel traces across two runs,
    //      for one and four substeps ----
    for (const int substeps : {1, 4}) {
        auto run = [&](std::vector<double>& trace) {
            MujocoBackend backend(ModelPath(), kControlDt, substeps);
            backend.Seed(OffsetRotatedPose());
            for (int k = 0; k < 100; ++k) {
                JointVector right_cmd = home.right_deg;
                JointVector left_cmd = home.left_deg;
                for (int i = 0; i < 7; ++i) {
                    right_cmd[i] += 2.0 * std::sin(0.05 * k + i);
                    left_cmd[i] -= 1.5 * std::sin(0.07 * k + i);
                }
                backend.Exchange(right_cmd, left_cmd, OffsetRotatedPose());
                trace.insert(trace.end(), backend.Data().qpos,
                             backend.Data().qpos + backend.Model().nq);
                trace.insert(trace.end(), backend.Data().qvel,
                             backend.Data().qvel + backend.Model().nv);
            }
        };
        std::vector<double> trace_a;
        std::vector<double> trace_b;
        run(trace_a);
        run(trace_b);
        Check(trace_a.size() == trace_b.size() &&
                  std::memcmp(trace_a.data(), trace_b.data(),
                              trace_a.size() * sizeof(double)) == 0,
              "byte-identical qpos/qvel trace across repeated runs "
              "(substeps " +
                  std::to_string(substeps) + ")");
    }

    // ---- The arms are gravity-loaded (the point of the mocap Mount) ----
    //      Under the superseded freejoint prescription the rig fell
    //      freely inside every substep, so a held command moved joint 2
    //      by exactly 0.0 rad and every arm qacc was below 1e-12 rad/s^2
    //      (README probe, 2026-08-17). Welded to world, each servo must
    //      instead settle where its force balances gravity:
    //
    //          kp * (ctrl - q) = qfrc_bias   =>   droop = -bias / kp
    //
    //      The expectation is read from MuJoCo's OWN bias torques and the
    //      model's OWN gains, not from a number this test invented, so a
    //      plant that merely moved a bit would still fail. 5 % covers the
    //      bias changing slightly at the drooped posture.
    {
        MujocoBackend backend(ModelPath(), kControlDt, 1);
        const SimArmFeedback seeded = backend.Seed(IdentityPose());
        const mjModel& model = backend.Model();
        const DualModelContract& ids = backend.Contract();

        double max_seed_qacc = 0.0;
        for (int i = 0; i < 7; ++i)
            max_seed_qacc = std::max(
                {max_seed_qacc,
                 std::abs(backend.Data().qacc[ids.right.joint_dof_adr[i]]),
                 std::abs(backend.Data().qacc[ids.left.joint_dof_adr[i]])});
        Check(max_seed_qacc > 1.0,
              "arm joints accelerate under gravity at the seed (observed "
              "max |qacc| " + std::to_string(max_seed_qacc) +
                  " rad/s^2; the free-falling freejoint plant gave < 1e-12)");

        // Predicted steady droop per joint, from bias and gain.
        JointVector predicted_right{}, predicted_left{};
        for (int i = 0; i < 7; ++i) {
            const double kp_right =
                model.actuator_gainprm[mjNGAIN * ids.right.actuator_id[i]];
            const double kp_left =
                model.actuator_gainprm[mjNGAIN * ids.left.actuator_id[i]];
            predicted_right[i] =
                -backend.Data().qfrc_bias[ids.right.joint_dof_adr[i]] /
                kp_right * kRadToDeg;
            predicted_left[i] =
                -backend.Data().qfrc_bias[ids.left.joint_dof_adr[i]] /
                kp_left * kRadToDeg;
        }

        SimArmFeedback settled{};
        for (int k = 0; k < 250; ++k) // 0.5 s, well past the servo settle
            settled = backend.Exchange(home.right_deg, home.left_deg,
                                       IdentityPose());
        for (int i = 0; i < 7; ++i) {
            const double observed_right =
                settled.right.continuous_position_deg[i] -
                seeded.right.continuous_position_deg[i];
            const double observed_left =
                settled.left.continuous_position_deg[i] -
                seeded.left.continuous_position_deg[i];
            std::printf("  joint %d droop: right %+.6f deg (predicted "
                        "%+.6f), left %+.6f deg (predicted %+.6f)\n",
                        i + 1, observed_right, predicted_right[i],
                        observed_left, predicted_left[i]);
            Check(std::abs(observed_right - predicted_right[i]) <
                      0.05 * std::abs(predicted_right[i]) + 1e-4,
                  "right joint " + std::to_string(i + 1) +
                      " settles at the gravity droop -bias/kp");
            Check(std::abs(observed_left - predicted_left[i]) <
                      0.05 * std::abs(predicted_left[i]) + 1e-4,
                  "left joint " + std::to_string(i + 1) +
                      " settles at the gravity droop -bias/kp");
        }
    }

    if (failures == 0) {
        std::printf("PASS: MujocoBackend seed/exchange/substep/wire-format/"
                    "non-finite/determinism checks all hold\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", failures);
    return 1;
}
