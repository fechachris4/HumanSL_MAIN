//
// ModelContract — the one owner of MuJoCo name->index resolution and
// structural validation for the dual-arm execution twin.
//
// ValidateAndResolveModel is called once at startup (by tests now, by the
// simulation adapter in later plan tasks). It resolves every joint,
// actuator, body and site of model/humansl_dual_gen3.xml by exact name and
// checks the structure against the declared authorities:
//
//   - kinematic authority: the production URDF
//     (model/GEN3_dual_mounted.urdf) — 7 hinge joints per
//     arm about local z, joints 2/4/6 bounded to the URDF ranges, 1/3/5/7
//     continuous, chain topology mount -> base -> ... -> bracelet, TCP
//     sites right_tcp (configured tool) / left_tcp (bare flange), and the
//     Mount itself a world-welded mocap body so the arms hang off it with
//     real gravity load (decision 2026-08-17, replacing the prescribed
//     freejoint whose free acceleration left the rig in free fall);
//   - mechanics authority: msc_project gen3.xml — actuator ctrlranges
//     (generic position servos, not a Kinova model).
//
// On any mismatch it throws std::runtime_error naming the missing or
// mismatched item with expected and observed values. After it returns,
// all control-loop access is by the stored integer ids and addresses —
// never name lookup, never assumed qpos/dof addresses.
//
// Hazard this guard owns (questionnaire answered in the Task 2 shape
// proposal): a renamed/reordered joint or shifted address would silently
// map commands onto the wrong joints, corrupting every downstream sim
// result. First observable at model load, before any stepping; response
// is rejection at startup. Validation happens once, here, nowhere else.
//

#pragma once

#include <array>

#include <mujoco/mujoco.h>

struct ArmModelIds {
    // Index i (0-based) is joint i+1 of the Gen3 chain, base to wrist.
    std::array<int, 7> joint_qpos_adr; // into mjData.qpos, radians
    std::array<int, 7> joint_dof_adr;  // into mjData.qvel, radians/second
    std::array<int, 7> actuator_id;    // into mjData.ctrl, radians (position servo)
    int base_body_id;                  // <side>_base_link, child of mount
    int tcp_site_id;                   // <side>_tcp production TCP site
};

struct DualModelContract {
    int mount_body_id; // the one Mount body, a world-welded mocap body
    // The Mount's mocap index (resolved here, the one name->index owner,
    // so the backend never repeats the lookup): row into mjData.mocap_pos
    // (3, world metres) and mjData.mocap_quat (4, unit quaternion in
    // MuJoCo's w,x,y,z order — NOT the Vicon x,y,z,w wire order). Writing
    // those two rows moves the Mount; the body is welded to world, so it
    // has no qpos/qvel of its own and no velocity the physics can see.
    int mount_mocap_id;
    ArmModelIds right; // TCP = ConfiguredTool_Link pose (configured tool)
    ArmModelIds left;  // TCP = leftEndEffector_Link pose (bare flange)
};

// Throws std::runtime_error("model contract violation: ...") on the first
// structural mismatch; returns fully resolved ids otherwise.
DualModelContract ValidateAndResolveModel(const mjModel& model);
