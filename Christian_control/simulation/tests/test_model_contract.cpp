//
// Model contract test for the exact-frame dual-arm MJCF (Plan 02 Task 2).
//
// Pins the generated model's structure against the declared authorities:
//   - kinematic authority: the production URDF
//     (Christian_control/model/GEN3_dual_mounted.urdf) —
//     joint order, bounded ranges, chain topology, mount placement, TCPs;
//   - mechanics authority: msc_project gen3.xml — actuator classes and
//     ctrlranges only (generic plant, explicitly not a Kinova servo model).
//
// Structure asserted (accepted shape proposal, 2026-08-17, as amended by
// the mocap Mount rework of the same day):
//   one Mount body under world, declared mocap="true" so it is welded to
//   world and moved only through mjData.mocap_pos/mocap_quat; nq=nv=14
//   (the arm hinges alone), nu=14, nmocap=1; fourteen uniquely
//   right_/left_-prefixed hinge joints and actuators; joints 2/4/6 bounded
//   with the URDF ranges, 1/3/5/7 continuous; right/left base bodies
//   children of Mount; TCP sites right_tcp (right_tool_link pose) and
//   left_tcp (left_end_effector_link pose, bare flange — asymmetric by
//   design, no pinch_site anywhere).
//
// Mutation cases: a renamed joint, a wrong bounded range, a duplicate name,
// a dropped TCP site, and a Mount that is no longer a mocap body must each
// fail with a message naming the offending item (duplicate names are
// rejected by the MuJoCo compiler itself).
//
// Contact filtering is checked here too, because the mocap Mount changed
// it: MuJoCo skips parent/child contacts unless the parent is welded to
// world, which every <side>_base_link now is. The generated model carries
// two <exclude> pairs restoring the SAME-ARM base/shoulder filtering the
// freejoint model got for free; the checks below show the home posture is
// contact-free WITH them, that the exclude list contains those two pairs
// and nothing else, and (as a mutation) that without them the home
// posture has 8 contacts 12.0 mm deep. Two CROSS-arm pairs that the old
// weld tree filtered are deliberately left live — real interference
// between separate arms, so an ncon == 0 check here is stricter than the
// same line was under the freejoint model.
//
// Evidence class: unit test (model load + structural validation; nothing
// is stepped, no robot-facing code exists in this project).
//

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "ModelContract.h"

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

    // MuJoCo reports recoverable model problems (missing inertials, bad
    // ranges, ...) through mju_user_warning; the plan requires a clean
    // load, so every warning is counted as a failure.
    int warning_count = 0;
    void CountWarning(const char* message)
    {
        std::printf("MuJoCo warning: %s\n", message);
        ++warning_count;
    }

    std::string ReadFile(const std::string& path)
    {
        std::ifstream stream(path);
        std::ostringstream buffer;
        buffer << stream.rdbuf();
        return buffer.str();
    }

    // Replaces the first `count` occurrences (count = 0 means all).
    // Returns the number of replacements actually made so a mutation that
    // no longer matches the generated text fails loudly instead of
    // silently testing nothing.
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

    // Loads a (possibly mutated) model XML from a string by writing it next
    // to the build directory. meshdir is rewritten to the absolute asset
    // path so the temporary copy still finds the audited meshes.
    mjModel* LoadFromText(std::string xml_text, std::string* error_out)
    {
        const int redirected =
            Replace(xml_text, "meshdir=\"assets\"",
                    "meshdir=\"" SIM_MODEL_DIR "/assets\"", 1);
        if (redirected != 1) {
            *error_out = "meshdir attribute not found in model text";
            return nullptr;
        }
        const std::string temp_path = "mutated_model_contract.xml";
        std::ofstream(temp_path) << xml_text;
        char error[1024] = {0};
        mjModel* model =
            mj_loadXML(temp_path.c_str(), nullptr, error, sizeof(error));
        *error_out = error;
        return model;
    }

    // Applies one text mutation and requires the load or the contract to
    // fail with a message containing `expected_substring`.
    void CheckMutationFails(const std::string& base_xml,
                            const std::string& from, const std::string& to,
                            int occurrences,
                            const std::string& expected_substring,
                            const std::string& label)
    {
        std::string mutated = base_xml;
        const int done = Replace(mutated, from, to, occurrences);
        if (occurrences != 0 && done != occurrences) {
            Check(false, label + ": mutation pattern '" + from +
                             "' matched " + std::to_string(done) +
                             " time(s), expected " +
                             std::to_string(occurrences));
            return;
        }
        if (occurrences == 0 && done == 0) {
            Check(false,
                  label + ": mutation pattern '" + from + "' never matched");
            return;
        }

        std::string load_error;
        mjModel* model = LoadFromText(mutated, &load_error);
        if (model == nullptr) {
            Check(load_error.find(expected_substring) != std::string::npos,
                  label + ": load failed but message '" + load_error +
                      "' does not name '" + expected_substring + "'");
            return;
        }
        try {
            (void)ValidateAndResolveModel(*model);
            Check(false, label + ": mutation was accepted by the contract");
        } catch (const std::runtime_error& rejection) {
            const std::string message = rejection.what();
            Check(message.find(expected_substring) != std::string::npos,
                  label + ": rejection '" + message + "' does not name '" +
                      expected_substring + "'");
        }
        mj_deleteModel(model);
    }

    void PrintFrame(const char* label, const double* position,
                    const double* rotation_row_major)
    {
        std::printf("%-28s p [% .9f % .9f % .9f]\n", label, position[0],
                    position[1], position[2]);
        for (int row = 0; row < 3; ++row)
            std::printf("%-28s R [% .9f % .9f % .9f]\n", "",
                        rotation_row_major[3 * row + 0],
                        rotation_row_major[3 * row + 1],
                        rotation_row_major[3 * row + 2]);
    }
} // namespace

int main()
{
    mju_user_warning = CountWarning;

    // --- 1. The generated model loads cleanly ---------------------------
    char error[1024] = {0};
    mjModel* model =
        mj_loadXML(SIM_MODEL_XML_PATH, nullptr, error, sizeof(error));
    Check(model != nullptr, std::string("model failed to load: ") + error);
    if (model == nullptr) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    Check(warning_count == 0, "model load produced MuJoCo warnings");

    // --- 2. The contract accepts it and resolves consistent ids ---------
    DualModelContract contract{};
    bool contract_ok = true;
    try {
        contract = ValidateAndResolveModel(*model);
    } catch (const std::runtime_error& rejection) {
        contract_ok = false;
        Check(false, std::string("contract rejected the generated model: ") +
                         rejection.what());
    }

    if (contract_ok) {
        // Sizes per the accepted shape: the mocap Mount contributes no
        // state at all, so nq = nv = 14 is exactly the arm hinges.
        Check(model->nq == 14, "nq is 14");
        Check(model->nv == 14, "nv is 14");
        Check(model->nu == 14, "nu is 14");
        Check(model->nmocap == 1, "exactly one mocap body");
        Check(model->body_mocapid[contract.mount_body_id] ==
                  contract.mount_mocap_id,
              "the contract's mocap index is the Mount body's own");

        // Joint addresses are read back from the model, never assumed.
        bool seen_qpos[32] = {false};
        bool seen_dof[32] = {false};
        for (const ArmModelIds* arm : {&contract.right, &contract.left}) {
            for (int i = 0; i < 7; ++i) {
                const int qpos_adr = arm->joint_qpos_adr[i];
                const int dof_adr = arm->joint_dof_adr[i];
                Check(qpos_adr >= 0 && qpos_adr < 14,
                      "hinge qpos address " + std::to_string(qpos_adr) +
                          " lies inside the 14-hinge state");
                Check(dof_adr >= 0 && dof_adr < 14,
                      "hinge dof address " + std::to_string(dof_adr) +
                          " lies inside the 14-hinge state");
                Check(!seen_qpos[qpos_adr],
                      "qpos address " + std::to_string(qpos_adr) +
                          " is unique");
                Check(!seen_dof[dof_adr],
                      "dof address " + std::to_string(dof_adr) + " is unique");
                seen_qpos[qpos_adr] = true;
                seen_dof[dof_adr] = true;
                Check(arm->actuator_id[i] >= 0 &&
                          arm->actuator_id[i] < model->nu,
                      "actuator id in range");
            }
        }
        Check(contract.right.base_body_id != contract.left.base_body_id,
              "right and left base bodies are distinct");
        Check(contract.right.tcp_site_id != contract.left.tcp_site_id,
              "right and left TCP sites are distinct");

        // --- 3. Diagnostic frames for the manual FK cross-check ---------
        // qpos0 (mount at identity, all joints 0): base and TCP world
        // frames printed for comparison against print_dual_arm_fk. These
        // prints are the Task 2 review-checkpoint evidence; the binding
        // numeric parity gate is Task 3.
        mjData* data = mj_makeData(model);
        mj_forward(model, data);
        std::printf("\nworld frames at qpos0 (mount = identity, q = 0):\n");
        PrintFrame("right_base_link",
                   data->xpos + 3 * contract.right.base_body_id,
                   data->xmat + 9 * contract.right.base_body_id);
        PrintFrame("left_base_link",
                   data->xpos + 3 * contract.left.base_body_id,
                   data->xmat + 9 * contract.left.base_body_id);
        PrintFrame("right_tcp",
                   data->site_xpos + 3 * contract.right.tcp_site_id,
                   data->site_xmat + 9 * contract.right.tcp_site_id);
        PrintFrame("left_tcp", data->site_xpos + 3 * contract.left.tcp_site_id,
                   data->site_xmat + 9 * contract.left.tcp_site_id);

        // --- 3b. The home posture is contact-free --------------------
        // The seed state every run starts from must have no contact at
        // all: a contact here would push the arms with a force no real
        // arm feels, and would be indistinguishable from a Mount or
        // frame error downstream. The mutation below shows this is a
        // real property of the two <exclude> pairs, not of luck.
        const int home_key = mj_name2id(model, mjOBJ_KEY, "home");
        Check(home_key >= 0, "home keyframe exists");
        if (home_key >= 0) {
            mj_resetDataKeyframe(model, data, home_key);
            mj_forward(model, data);
            Check(data->ncon == 0,
                  "no contact at the home keyframe (observed " +
                      std::to_string(data->ncon) + ")");
        }

        // --- 3c. The exclude list is exactly the two same-arm pairs ---
        // The excludes replace the parent/child filtering each arm lost
        // when its base_link became world-welded. They must not quietly
        // grow: welding the bases to world ALSO un-filtered two
        // cross-arm pairs (left_base_link <-> right_shoulder_link and
        // right_base_link <-> left_shoulder_link), and those are real
        // interference between two separate arms, so they stay live and
        // must never be excluded away to make an ncon == 0 check pass.
        // Measured 2026-08-17 with all geom margins forced to 50 m, so
        // only the filter could reject a pair: 105 collidable body pairs
        // in this model against 103 in a reconstructed freejoint model.
        {
            std::vector<std::string> excluded;
            for (int e = 0; e < model->nexclude; ++e) {
                const int signature = model->exclude_signature[e];
                const char* body1 =
                    mj_id2name(model, mjOBJ_BODY, signature >> 16);
                const char* body2 =
                    mj_id2name(model, mjOBJ_BODY, signature & 0xFFFF);
                excluded.push_back(std::string(body1 ? body1 : "?") + "|" +
                                   std::string(body2 ? body2 : "?"));
            }
            std::sort(excluded.begin(), excluded.end());
            const std::vector<std::string> expected = {
                "left_base_link|left_shoulder_link",
                "right_base_link|right_shoulder_link"};
            std::string listed;
            for (const std::string& pair : excluded)
                listed += (listed.empty() ? "" : ", ") + pair;
            Check(excluded == expected,
                  "contact excludes are exactly the two same-arm "
                  "base/shoulder pairs (observed: " + listed + ")");
        }
        mj_deleteData(data);
    }
    mj_deleteModel(model);
    Check(warning_count == 0, "no MuJoCo warnings after forward pass");

    // --- 4. Mutations must fail with a message naming the defect --------
    const std::string base_xml = ReadFile(SIM_MODEL_XML_PATH);
    Check(!base_xml.empty(), "model XML is readable");
    Check(base_xml.find("pinch_site") == std::string::npos,
          "pinch_site must not appear anywhere in the generated model");

    // 4a. Renamed joint (all occurrences, so the actuator still resolves
    //     and the failure is the contract's, naming the missing joint).
    CheckMutationFails(base_xml, "right_joint_3", "right_jointX_3", 0,
                       "right_joint_3", "renamed joint");

    // 4b. Wrong bounded range on the first (right) joint_2 only.
    CheckMutationFails(base_xml, "range=\"-2.24 2.24\"",
                       "range=\"-2.5 2.5\"", 1, "right_joint_2",
                       "wrong bounded range");

    // 4c. Duplicate name: the left joint renamed onto the right one; the
    //     MuJoCo compiler itself must reject the duplicate.
    CheckMutationFails(base_xml, "left_joint_1", "right_joint_1", 0,
                       "repeated name", "duplicate joint name");

    // 4d. Dropped TCP site.
    CheckMutationFails(base_xml, "name=\"right_tcp\"",
                       "name=\"right_tcp_gone\"", 1, "right_tcp",
                       "dropped TCP site");

    // 4e. Mount no longer a mocap body: it becomes an ordinary static
    //     body pinned at the model's identity pose, so every prescribed
    //     Mount pose would be silently ignored and the arms would hang in
    //     the wrong place with no other symptom. No joint, actuator or
    //     site changes, so only the contract's mocap checks can see it:
    //     the nmocap count fires first, and the per-body check ("body
    //     'mount' is not a mocap body") is the second line for a model
    //     that has a mocap body somewhere else.
    //     (The keyframe's mocap fields go with it: with no mocap body
    //     the MuJoCo compiler rejects an mpos/mquat of any length, and
    //     that early rejection would hide the contract check being
    //     tested here.)
    {
        std::string without_mocap_key = base_xml;
        const int stripped = Replace(without_mocap_key,
                                     " mpos=\"0 0 0\" mquat=\"1 0 0 0\"", "",
                                     1);
        Check(stripped == 1, "keyframe mocap fields found for the mutation");
        CheckMutationFails(without_mocap_key,
                           "<body name=\"mount\" mocap=\"true\">",
                           "<body name=\"mount\">", 1, "nmocap",
                           "Mount stripped of mocap");
    }

    // 4f. Removed contact excludes: with the Mount welded to world, each
    //     <side>_base_link is world-welded too, so MuJoCo stops filtering
    //     its contacts with its own shoulder child. This mutation is the
    //     evidence that the excludes are load-bearing rather than
    //     decorative — it must produce penetrating contacts at the very
    //     posture every run seeds from. (The structural contract accepts
    //     it: contact filtering is not part of the joint/actuator
    //     mapping it owns, so this check, not the contract, is the
    //     defense.)
    {
        std::string mutated = base_xml;
        int done = Replace(mutated,
                           "<exclude body1=\"right_base_link\" "
                           "body2=\"right_shoulder_link\" />",
                           "", 1);
        done += Replace(mutated,
                        "<exclude body1=\"left_base_link\" "
                        "body2=\"left_shoulder_link\" />",
                        "", 1);
        Check(done == 2, "both exclude patterns matched once each");
        std::string load_error;
        mjModel* unfiltered = LoadFromText(mutated, &load_error);
        Check(unfiltered != nullptr,
              "model without the excludes still loads: " + load_error);
        if (unfiltered != nullptr) {
            mjData* unfiltered_data = mj_makeData(unfiltered);
            mj_resetDataKeyframe(unfiltered, unfiltered_data,
                                 mj_name2id(unfiltered, mjOBJ_KEY, "home"));
            mj_forward(unfiltered, unfiltered_data);
            double deepest = 0.0;
            for (int i = 0; i < unfiltered_data->ncon; ++i)
                deepest = std::min(deepest, unfiltered_data->contact[i].dist);
            std::printf("without the base/shoulder excludes: ncon %d, "
                        "deepest penetration %.6f m\n",
                        static_cast<int>(unfiltered_data->ncon), deepest);
            Check(unfiltered_data->ncon > 0,
                  "removing the excludes must produce contacts at home "
                  "(they are load-bearing, not decorative)");
            mj_deleteData(unfiltered_data);
            mj_deleteModel(unfiltered);
        }
    }

    if (failures == 0) {
        std::printf("PASS: model contract holds and all mutations are "
                    "rejected\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", failures);
    return 1;
}
