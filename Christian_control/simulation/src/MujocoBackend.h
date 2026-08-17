//
// MujocoBackend — the boundary adapter between the execution core and the
// MuJoCo plant (Plan 02 Task 4). It owns exactly three concerns:
//
//   1. Physics stepping and timing: opt.timestep = control_dt_s /
//      physics_substeps (validated once, at construction); one Exchange
//      writes the 14 controls once and calls mj_step exactly
//      physics_substeps times, so simulated time advances one 0.002 s
//      control tick per Exchange and the controls are zero-order-held
//      across the tick — the analogue of the hardware low-level servo
//      holding the last 500 Hz frame.
//
//   2. The Kortex wire-format emulation (contract section 2: robot
//      interface degrees convert once at the adapter boundary). Inbound:
//      the core's continuous command degrees -> radian position-servo
//      ctrl. Outbound: MuJoCo hinge qpos is already continuous radians,
//      so continuous degrees are a direct conversion, and the wrapped
//      [0, 360) actuator-reported degrees ArmExecutionInput/ResolveStop
//      consume are derived from the continuous value — exactly what the
//      hardware wire does. (The hardware-side "recover continuous near
//      the previous command" unwrap exists because Kinova only reports
//      wrapped; it is unnecessary here and deliberately absent.)
//
//   3. The Mount pose prescription (the disturbance input): the Mount is
//      a MuJoCo mocap body, welded to world at whatever pose sits in
//      mjData.mocap_pos / mocap_quat, with both arms as its direct
//      jointed children. Exchange writes the tick's world_T_mount there
//      once, before the substeps. The arms therefore carry real gravity
//      and are carried kinematically by the Mount, while the Mount itself
//      is unmoved by anything they do. The Mount is prescribed, never
//      simulated.
//
// What this type deliberately does NOT do: it does not build WorldSample
// records (the ideal Mount source in Task 5 owns sensing), does not
// classify freshness (the execution core owns that), and does not carry
// RobotState in its feedback (the core's deg -> rad boundary and world
// classification must not be duplicated here). It also takes no Mount
// TWIST: a mocap body has no velocity the physics can see, so the plant
// has no honest use for one. Mount velocity is a sensing quantity, owned
// by whoever builds the WorldSample, and must be the derivative of the
// very Mount motion whose pose is handed to Exchange — produced by the
// same evaluation of that motion, never from differencing these mocap
// poses and never configured beside the pose as an independent number.
//
// The one runtime guard (hazard questionnaire in the accepted Task 4
// shape proposal): a non-finite command/Mount input, or a post-step state
// that is non-finite or beyond MuJoCo's own mjMAXVAL bad-state threshold,
// poisons mjData — worse, mj_step's internal checks would silently
// auto-reset the state on the NEXT step (mjWARN_BADQPOS) and every later
// feedback sample would be plausible-looking garbage. Inputs are checked
// at Exchange entry (reject before any step); the state and warning
// counters are scanned after EVERY substep (throw before the next
// substep could consume the corpse — with substeps > 1 a once-per-tick
// scan would run the remaining substeps on MuJoCo's silently reset
// state). In this offline deterministic rig a
// loud stop is the informative outcome; the graceful-degradation rule
// governs live robot behaviour, not a simulation harness.
//
// One honest caveat about that guard after the mocap rework (2026-08-17):
// the warning-counter half covers the case where MuJoCo flags and resets
// a bad state WITHIN a substep, which the post-substep qpos/qvel scans
// cannot see. It is currently unreachable through this boundary — the
// old injection route was a finite but beyond-mjMAXVAL Mount TWIST
// written into the freejoint's qvel, and a mocap Mount has no velocity to
// write. It is kept, unexercised, because MuJoCo's own within-step
// mj_checkAcc reset remains reachable by contact-rich scenes (Plan 03's
// obstacle geometry), and losing it would make such a run return
// plausible garbage in silence.
//
// Units and frames: commands and measurements cross this boundary in
// degrees (the Kortex wire unit); everything inside MuJoCo is radians,
// metres, seconds; world_T_mount is the Mount pose in Vicon-world axes W
// (State.h conventions).
//
// Evidence class for anything computed against this plant: simulation,
// on generic position servos (kp/kv from gen3.xml are declared generic
// mechanics, not Kinova parameters — model/model_provenance.yaml).
//

#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "Config.h" // JointVector
#include "ModelContract.h"
#include "State.h" // CartesianPose

// One arm's measured joint state in both hardware wire shapes. Raw
// records only — no RobotState, no classification.
struct SimJointFeedback {
    // Actuator-reported degrees, wrapped to [0, 360) exactly like the
    // Kinova feedback wire (StopPriority.h assumes this shape).
    JointVector measured_position_deg{};
    JointVector measured_velocity_deg_s{};
    // The same measurement as continuous degrees (MuJoCo hinge qpos never
    // wraps) — validation-side truth for the wrap handling, and the
    // natural seed for the core's continuous command state.
    JointVector continuous_position_deg{};
};

struct SimArmFeedback {
    SimJointFeedback right;
    SimJointFeedback left;
    double simulation_time_s = 0.0; // mjData.time after the substep loop
};

class MujocoBackend {
public:
    // Loads the generated MJCF, validates and resolves it exactly once
    // via ValidateAndResolveModel, and fixes the timing: opt.timestep =
    // control_dt_s / physics_substeps. Throws std::invalid_argument on a
    // physically meaningless period or substep count and
    // std::runtime_error on a model that cannot be loaded or fails the
    // structural contract. No validation is repeated per tick.
    MujocoBackend(const std::string& model_path, double control_dt_s,
                  int physics_substeps);
    ~MujocoBackend();

    MujocoBackend(const MujocoBackend&) = delete;
    MujocoBackend& operator=(const MujocoBackend&) = delete;

    // Full deterministic reset: home keyframe (mj_resetDataKeyframe, which
    // also restores the keyframe ctrl and mocap pose), Mount placed at
    // world_T_mount, one mj_forward for derived quantities. Time stays 0.
    // Calling Seed again is the reset Task 5's Reset will use.
    SimArmFeedback Seed(const CartesianPose& world_T_mount);

    // One 0.002 s control tick: validate inputs finite -> write the 14
    // ctrl values once (deg -> rad) -> write the Mount mocap pose once ->
    // physics_substeps x (mj_step + healthy-state scan) -> assemble
    // feedback. Command degrees are the core's continuous commanded_deg
    // frames. Throws std::logic_error before the first Seed.
    SimArmFeedback Exchange(const JointVector& right_position_deg,
                            const JointVector& left_position_deg,
                            const CartesianPose& world_T_mount);

    // Plain read-only facts (run provenance is assembled in Task 5).
    double timestep_s() const;
    int physics_substeps() const { return physics_substeps_; }
    const DualModelContract& Contract() const { return contract_; }

    // Read-only plant access for tests and the sim-side validation
    // contract. Production control code never sees these (simulator truth
    // must not enter production controller calculations, contract §4).
    const mjModel& Model() const { return *model_; }
    const mjData& Data() const { return *data_; }

private:
    void SetMountPose(const CartesianPose& world_T_mount);
    SimArmFeedback ReadFeedback() const;
    void RequireHealthyState() const;

    struct ModelDeleter {
        void operator()(mjModel* model) const { mj_deleteModel(model); }
    };
    struct DataDeleter {
        void operator()(mjData* data) const { mj_deleteData(data); }
    };

    std::unique_ptr<mjModel, ModelDeleter> model_;
    std::unique_ptr<mjData, DataDeleter> data_;
    DualModelContract contract_{};
    int physics_substeps_ = 0;
    int home_key_id_ = -1;
    bool seeded_ = false;
};
