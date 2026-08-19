#include "ModelContract.h"

#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
    // Bounded joint ranges in radians, from the production URDF (the
    // kinematic authority recorded in model/model_provenance.yaml):
    // right_joint_2/4/6 limit elements, GEN3_dual_mounted.urdf lines 130, 188,
    // 246 (identical for the left chain). The strings "-2.24 2.24" etc.
    // in the generated MJCF parse to exactly these doubles.
    constexpr double kBoundedRange[7] = {0.0, 2.24, 0.0, 2.57, 0.0, 2.09, 0.0};
    constexpr bool kIsBounded[7] = {false, true, false, true,
                                    false, true, false};

    // Actuator ctrlranges in radians, from gen3.xml (the mechanics
    // authority; generic position servos, not Kinova values): actuator
    // entries for joint_2/4/6.
    constexpr double kCtrlRange[7] = {0.0, 2.2497294058206907, 0.0,
                                      2.5795966344476193, 0.0,
                                      2.0996310901491784, 0.0};

    [[noreturn]] void Fail(const std::string& what)
    {
        throw std::runtime_error("model contract violation: " + what);
    }

    void RequireCount(const char* what, int observed, int expected)
    {
        if (observed != expected) {
            std::ostringstream message;
            message << what << " expected " << expected << ", observed "
                    << observed;
            Fail(message.str());
        }
    }

    int RequireId(const mjModel& model, mjtObj type, const std::string& name,
                  const char* kind)
    {
        const int id = mj_name2id(&model, type, name.c_str());
        if (id < 0)
            Fail(std::string(kind) + " '" + name + "' not found in the model");
        return id;
    }

    ArmModelIds ResolveArm(const mjModel& model, const std::string& side,
                           int mount_body_id)
    {
        ArmModelIds ids{};

        ids.base_body_id =
            RequireId(model, mjOBJ_BODY, side + "_base_link", "body");
        if (model.body_parentid[ids.base_body_id] != mount_body_id)
            Fail("body '" + side + "_base_link' is not a child of 'mount'");

        int previous_body_id = ids.base_body_id;
        for (int i = 0; i < 7; ++i) {
            const std::string name =
                side + "_joint_" + std::to_string(i + 1);
            const int joint_id = RequireId(model, mjOBJ_JOINT, name, "joint");

            if (model.jnt_type[joint_id] != mjJNT_HINGE)
                Fail("joint '" + name + "' is not a hinge");

            // The URDF declares every arm axis as local z exactly. The
            // generator asserts this against the URDF <axis> element and
            // emits it verbatim (generate_dual_mjcf.py, require_z_axis);
            // this check is the second line, catching a hand-edited model.
            const double* axis = model.jnt_axis + 3 * joint_id;
            if (axis[0] != 0.0 || axis[1] != 0.0 || axis[2] != 1.0) {
                std::ostringstream message;
                message << "joint '" << name << "' axis expected [0 0 1], "
                        << "observed [" << axis[0] << " " << axis[1] << " "
                        << axis[2] << "]";
                Fail(message.str());
            }

            // Chain topology: each joint's body hangs directly off the
            // previous joint's body (base for joint 1), pinning joint
            // order to the physical chain rather than name convention.
            const int body_id = model.jnt_bodyid[joint_id];
            if (model.body_parentid[body_id] != previous_body_id)
                Fail("joint '" + name +
                     "' body is not the direct child of the previous "
                     "chain body");
            previous_body_id = body_id;

            const bool limited = model.jnt_limited[joint_id] != 0;
            if (limited != kIsBounded[i])
                Fail("joint '" + name + "' expected " +
                     (kIsBounded[i] ? "bounded (URDF revolute)"
                                    : "continuous (no range)") +
                     ", observed the opposite");
            if (kIsBounded[i]) {
                const double* range = model.jnt_range + 2 * joint_id;
                if (range[0] != -kBoundedRange[i] ||
                    range[1] != kBoundedRange[i]) {
                    std::ostringstream message;
                    message << "joint '" << name << "' range expected ["
                            << -kBoundedRange[i] << ", " << kBoundedRange[i]
                            << "] from the URDF, observed [" << range[0]
                            << ", " << range[1] << "]";
                    Fail(message.str());
                }
            }

            const int actuator_id =
                RequireId(model, mjOBJ_ACTUATOR, name, "actuator");
            if (model.actuator_trntype[actuator_id] != mjTRN_JOINT ||
                model.actuator_trnid[2 * actuator_id] != joint_id)
                Fail("actuator '" + name +
                     "' does not drive the joint of the same name");
            if (kIsBounded[i]) {
                if (model.actuator_ctrllimited[actuator_id] == 0)
                    Fail("actuator '" + name + "' expected a ctrlrange");
                const double* ctrl = model.actuator_ctrlrange + 2 * actuator_id;
                if (ctrl[0] != -kCtrlRange[i] || ctrl[1] != kCtrlRange[i]) {
                    std::ostringstream message;
                    message << "actuator '" << name
                            << "' ctrlrange expected [" << -kCtrlRange[i]
                            << ", " << kCtrlRange[i]
                            << "] from gen3.xml, observed [" << ctrl[0]
                            << ", " << ctrl[1] << "]";
                    Fail(message.str());
                }
            } else if (model.actuator_ctrllimited[actuator_id] != 0) {
                Fail("actuator '" + name +
                     "' of a continuous joint must not be ctrl-limited");
            }

            ids.joint_qpos_adr[i] = model.jnt_qposadr[joint_id];
            ids.joint_dof_adr[i] = model.jnt_dofadr[joint_id];
            ids.actuator_id[i] = actuator_id;
        }

        // Production TCP site, fixed in the last chain body (bracelet).
        ids.tcp_site_id = RequireId(model, mjOBJ_SITE, side + "_tcp", "site");
        if (model.site_bodyid[ids.tcp_site_id] != previous_body_id)
            Fail("site '" + side +
                 "_tcp' is not attached to the bracelet body");

        return ids;
    }
} // namespace

DualModelContract ValidateAndResolveModel(const mjModel& model)
{
    // Sizes first: the Mount is a mocap body, so it contributes no qpos
    // or dof at all — the state is exactly the 14 arm hinges, driven by
    // 14 position actuators, on world + mount + 2 x 8 chain bodies, with
    // exactly the two production TCP sites (pinch_site must not exist).
    RequireCount("nq", model.nq, 14);
    RequireCount("nv", model.nv, 14);
    RequireCount("nu", model.nu, 14);
    RequireCount("njnt", model.njnt, 14);
    RequireCount("nbody", model.nbody, 18);
    RequireCount("nsite", model.nsite, 2);
    RequireCount("nmocap", model.nmocap, 1);

    DualModelContract contract{};
    contract.mount_body_id = RequireId(model, mjOBJ_BODY, "mount", "body");
    if (model.body_parentid[contract.mount_body_id] != 0)
        Fail("body 'mount' is not a direct child of world");

    // The Mount must be the mocap body: welded to world and moved only by
    // writing mjData.mocap_pos/mocap_quat. A jointed Mount would be
    // integrated by the physics instead of prescribed, and (as the
    // superseded freejoint prescription showed) would take the arms into
    // free fall with it.
    contract.mount_mocap_id = model.body_mocapid[contract.mount_body_id];
    if (contract.mount_mocap_id < 0)
        Fail("body 'mount' is not a mocap body (mocap=\"true\")");

    contract.right = ResolveArm(model, "right", contract.mount_body_id);
    contract.left = ResolveArm(model, "left", contract.mount_body_id);
    return contract;
}
