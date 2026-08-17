#include "MujocoBackend.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>

namespace
{
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    // Actuator-reported wrap: the Kinova wire reports every joint in
    // [0, 360). fmod can land exactly on 360.0 for a tiny negative input
    // (360 - 1e-16 rounds to 360 in double), hence the final fold to 0.
    double WrapDeg360(double continuous_deg)
    {
        double wrapped = std::fmod(continuous_deg, 360.0);
        if (wrapped < 0.0)
            wrapped += 360.0;
        if (wrapped >= 360.0)
            wrapped = 0.0;
        return wrapped;
    }

    void RequireFiniteJoints(const JointVector& values, const char* what)
    {
        for (int i = 0; i < 7; ++i) {
            if (!std::isfinite(values[i])) {
                std::ostringstream message;
                message << "MujocoBackend: non-finite " << what << " (joint "
                        << (i + 1) << ")";
                throw std::invalid_argument(message.str());
            }
        }
    }

    void RequireFiniteMount(const CartesianPose& world_T_mount)
    {
        if (!world_T_mount.position_m.allFinite() ||
            !world_T_mount.rotation.allFinite())
            throw std::invalid_argument(
                "MujocoBackend: non-finite world_T_mount");
    }
} // namespace

MujocoBackend::MujocoBackend(const std::string& model_path,
                             double control_dt_s, int physics_substeps)
{
    // Validated once, here; never re-checked in the cycle (the workflow's
    // validate-at-the-boundary rule, matching ArmExecutionCore's
    // construction-time pattern).
    if (!std::isfinite(control_dt_s) || control_dt_s <= 0.0)
        throw std::invalid_argument(
            "MujocoBackend: control_dt_s must be finite and > 0");
    if (physics_substeps < 1)
        throw std::invalid_argument(
            "MujocoBackend: physics_substeps must be >= 1");
    physics_substeps_ = physics_substeps;

    char error[1024] = {0};
    model_.reset(mj_loadXML(model_path.c_str(), nullptr, error,
                            sizeof(error)));
    if (!model_)
        throw std::runtime_error("MujocoBackend: cannot load '" +
                                 model_path + "': " + error);

    // The one structural validation and name->index resolution pass
    // (ModelContract owns it); every later access uses the stored ids.
    contract_ = ValidateAndResolveModel(*model_);

    // The generated MJCF deliberately carries no timestep; the backend
    // owns timing. Substeps divide the control period exactly by
    // construction.
    model_->opt.timestep = control_dt_s / physics_substeps_;

    // The keyframe is the seed state (mechanics source gen3.xml "home");
    // resolved once here — a backend concern, not a kinematic-structure
    // concern, so it is not part of DualModelContract.
    home_key_id_ = mj_name2id(model_.get(), mjOBJ_KEY, "home");
    if (home_key_id_ < 0)
        throw std::runtime_error(
            "MujocoBackend: keyframe 'home' not found in the model");

    data_.reset(mj_makeData(model_.get()));
    if (!data_)
        throw std::runtime_error("MujocoBackend: mj_makeData failed");
}

MujocoBackend::~MujocoBackend() = default;

double MujocoBackend::timestep_s() const { return model_->opt.timestep; }

SimArmFeedback MujocoBackend::Seed(const CartesianPose& world_T_mount)
{
    RequireFiniteMount(world_T_mount);
    // Keyframe reset restores qpos, qvel (zero), the keyframe ctrl (so an
    // un-commanded plant holds home) AND the keyframe's mocap pose; the
    // write below then puts the Mount where this run wants it. Time is
    // reset to 0 and stays 0: mj_forward computes derived quantities
    // without integrating.
    mj_resetDataKeyframe(model_.get(), data_.get(), home_key_id_);
    SetMountPose(world_T_mount);
    mj_forward(model_.get(), data_.get());
    seeded_ = true;
    return ReadFeedback();
}

SimArmFeedback MujocoBackend::Exchange(const JointVector& right_position_deg,
                                       const JointVector& left_position_deg,
                                       const CartesianPose& world_T_mount)
{
    if (!seeded_)
        throw std::logic_error("MujocoBackend: Exchange before Seed");
    // Reject non-finite inputs BEFORE any physics step: a NaN written
    // into ctrl or the mocap pose corrupts mjData irreversibly, and the
    // last good state must survive the throw.
    RequireFiniteJoints(right_position_deg, "right command degrees");
    RequireFiniteJoints(left_position_deg, "left command degrees");
    RequireFiniteMount(world_T_mount);

    // The 14 controls, written once per tick (deg -> rad at this
    // boundary, contract section 2) and never touched again until the
    // next Exchange: zero-order hold across all substeps.
    for (int i = 0; i < 7; ++i) {
        data_->ctrl[contract_.right.actuator_id[i]] =
            right_position_deg[i] * kDegToRad;
        data_->ctrl[contract_.left.actuator_id[i]] =
            left_position_deg[i] * kDegToRad;
    }

    // The Mount pose for this tick, written ONCE before the substeps: a
    // mocap body is welded to world, so the physics cannot move it and
    // there is nothing to re-pin between substeps. The Mount is therefore
    // zero-order-held across the tick (a staircase of steps <= v*dt,
    // 0.16 mm at the Plan 03 scenario speeds), which keeps the plant's
    // Mount pose bit-identical to the pose the tick's WorldSample
    // reported.
    SetMountPose(world_T_mount);
    for (int step = 0; step < physics_substeps_; ++step) {
        mj_step(model_.get(), data_.get());
        // Scanned after EVERY substep, not once per tick: mj_step only
        // notices a diverged state at the START of its next call and
        // responds by silently resetting to qpos0, so with substeps > 1
        // a post-loop scan would let the remaining substeps run on that
        // reset state and hide the divergence behind plausible numbers.
        RequireHealthyState();
    }

    return ReadFeedback();
}

void MujocoBackend::SetMountPose(const CartesianPose& world_T_mount)
{
    // The mocap rows: position in world metres, then the unit quaternion
    // in MuJoCo's w,x,y,z order — NOT the Vicon/State.h x,y,z,w wire
    // order. mj_kinematics reads these directly as the Mount body's world
    // pose, so the arms hang off exactly this frame.
    mjtNum* position = data_->mocap_pos + 3 * contract_.mount_mocap_id;
    position[0] = world_T_mount.position_m[0];
    position[1] = world_T_mount.position_m[1];
    position[2] = world_T_mount.position_m[2];
    mjtNum* quaternion = data_->mocap_quat + 4 * contract_.mount_mocap_id;
    const Eigen::Quaterniond rotation(world_T_mount.rotation);
    quaternion[0] = rotation.w();
    quaternion[1] = rotation.x();
    quaternion[2] = rotation.y();
    quaternion[3] = rotation.z();
}

SimArmFeedback MujocoBackend::ReadFeedback() const
{
    SimArmFeedback feedback;
    const ArmModelIds* arms[2] = {&contract_.right, &contract_.left};
    SimJointFeedback* out[2] = {&feedback.right, &feedback.left};
    for (int a = 0; a < 2; ++a) {
        for (int i = 0; i < 7; ++i) {
            const double continuous_deg =
                data_->qpos[arms[a]->joint_qpos_adr[i]] * kRadToDeg;
            out[a]->continuous_position_deg[i] = continuous_deg;
            out[a]->measured_position_deg[i] = WrapDeg360(continuous_deg);
            out[a]->measured_velocity_deg_s[i] =
                data_->qvel[arms[a]->joint_dof_adr[i]] * kRadToDeg;
        }
    }
    feedback.simulation_time_s = data_->time;
    return feedback;
}

void MujocoBackend::RequireHealthyState() const
{
    // MuJoCo's own bad-state criterion (mju_isBad: NaN or |x| > mjMAXVAL),
    // applied after every substep. mj_step runs the same checks itself —
    // qpos/qvel at the START of a step, acceleration mid-step — and
    // responds by silently resetting the state to qpos0 (mjWARN_BADQPOS /
    // BADQVEL / BADQACC), after which every feedback sample would be
    // plausible-looking garbage. The qpos/qvel scans below catch
    // divergence the integrator just produced, before the next substep's
    // own check could reset-and-hide it. The warning counters are the
    // only detector for the complementary case where MuJoCo has ALREADY
    // flagged and reset within this substep (e.g. a finite but
    // beyond-mjMAXVAL prescribed Mount twist trips mj_checkVel at step
    // start): the counters survive MuJoCo's internal reset, while the
    // reset leaves clean qpos/qvel the scans cannot see.
    for (int warning : {mjWARN_BADQPOS, mjWARN_BADQVEL, mjWARN_BADQACC}) {
        if (data_->warning[warning].number > 0) {
            std::ostringstream message;
            message << "MujocoBackend: MuJoCo flagged bad state ("
                    << mju_warningText(warning,
                                       data_->warning[warning].lastinfo)
                    << "); the simulation has diverged";
            throw std::runtime_error(message.str());
        }
    }
    for (int i = 0; i < model_->nq; ++i) {
        if (mju_isBad(data_->qpos[i])) {
            std::ostringstream message;
            message << "MujocoBackend: qpos[" << i
                    << "] non-finite or beyond mjMAXVAL after stepping ("
                    << data_->qpos[i] << "); the simulation has diverged";
            throw std::runtime_error(message.str());
        }
    }
    for (int i = 0; i < model_->nv; ++i) {
        if (mju_isBad(data_->qvel[i])) {
            std::ostringstream message;
            message << "MujocoBackend: qvel[" << i
                    << "] non-finite or beyond mjMAXVAL after stepping ("
                    << data_->qvel[i] << "); the simulation has diverged";
            throw std::runtime_error(message.str());
        }
    }
}
