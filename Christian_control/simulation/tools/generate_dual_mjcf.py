#!/usr/bin/env python3
"""Generate model/humansl_dual_gen3.xml — the exact-frame dual-arm MJCF.

Dev-time tool (never runs at build or runtime). The generated XML is the
only model artifact; hand-editing it is forbidden — every correction is
made here and the file regenerated (the provenance test pins its SHA-256).

Authority split (model/model_provenance.yaml, Plan 02 Task 1):

- Kinematic truth — the production URDF
  (Christian_control/model/GEN3_dual_mounted.urdf): ALL
  placements are derived from it at full double precision — the two
  mount->base fixed joints (the same numbers DualArmKinematics::
  MountFromBase reads through Pinocchio), every chain body origin
  (Actuator1..7 / leftActuator1..7 joint origins), bounded joint ranges,
  and both TCP site poses (right = EndEffector o ConfiguredTool, the
  configured tool; left = leftEndEffector, the bare flange — asymmetric
  by design, there is no left tool transform). gen3.xml's body quats are
  NOT used: they encode exact pi/2 rotations where the URDF writes
  rounded angles (1.5708), a ~3.7e-6 rad difference that would fail the
  Task 3 parity gate (<=1e-8) — and the URDF, not menagerie, is the
  declared authority.

- Simulator mechanics — /home/christian/msc_project/.../gen3.xml:
  inertials, mesh assets, visual/collision classes, generic position-
  actuator classes (kp/kv/forcerange/ctrlrange; declared generic plant,
  not a Kinova servo model), integrator choice, and the home keyframe.

Every input is hash-checked against model_provenance.yaml before use, so
this script refuses to generate from drifted sources. No number in the
output is hand-typed (the hand-carried DH rounding already cost ~2e-5 m
once).

Frame convention (engineering contract section 2): A_T_B = pose of frame B
in frame A. A URDF <origin> composes as Trans(xyz) then Rot(rpy) with
R = Rz(yaw) * Ry(pitch) * Rx(roll) — the same convention Pinocchio uses.
Units: metres, radians (compiler angle="radian"; MJCF would default to
degrees).

Usage:
    python3 Christian_control/simulation/tools/generate_dual_mjcf.py
"""

import copy
import hashlib
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_DIR = REPO_ROOT / "Christian_control" / "simulation"
PROVENANCE_PATH = SIM_DIR / "model" / "model_provenance.yaml"
OUTPUT_XML = SIM_DIR / "model" / "humansl_dual_gen3.xml"
ASSET_DIR = SIM_DIR / "model" / "assets"

# Canonical single-arm body order shared by the URDF chain and gen3.xml.
# Index 0 is the base; indices 1..7 are the bodies moved by joints 1..7.
BODY_NAMES = [
    "base_link",
    "shoulder_link",
    "half_arm_1_link",
    "half_arm_2_link",
    "forearm_link",
    "spherical_wrist_1_link",
    "spherical_wrist_2_link",
    "bracelet_link",
]
GEN3_BODY_NAMES = BODY_NAMES  # gen3.xml uses exactly these names.

# Scene-dressing constants. Visual only — nothing here is collidable, and no
# number below is a measurement of the real rig.
FLOOR_DROP_M = 1.1  # visual ground plane below the Mount anchor
TORSO_HALF_EXTENTS_M = (0.12, 0.18, 0.28)  # proxy, from msc_project scene.xml
TORSO_CENTRE_IN_MOUNT_M = (0.0, 0.0, -0.30)  # box hangs below the mount plate

ARMS = {
    "right": {
        "mount_joint": "right_base_mount",
        "chain_joints": [f"Actuator{i}" for i in range(1, 8)],
        # Right TCP: configured tool frame (ConfiguredTool_Link), reached
        # from Bracelet_Link through two fixed joints.
        "tcp_joints": ["EndEffector", "ConfiguredTool"],
    },
    "left": {
        "mount_joint": "left_base_mount",
        "chain_joints": [f"leftActuator{i}" for i in range(1, 8)],
        # Left TCP: bare flange (leftEndEffector_Link). No tool transform
        # exists for the left arm; inventing one is forbidden.
        "tcp_joints": ["leftEndEffector"],
    },
}

# ---------------------------------------------------------------------------
# Minimal 3x3 matrix / quaternion helpers (double precision throughout).
# ---------------------------------------------------------------------------


def rpy_to_matrix(roll, pitch, yaw):
    """URDF rpy -> rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_mul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def matrix_to_quat(m):
    """Rotation matrix -> unit quaternion (w, x, y, z), w >= 0 canonical."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    quat = [w / norm, x / norm, y / norm, z / norm]
    if quat[0] < 0.0:
        quat = [-c for c in quat]
    return quat


def quat_angle_between(qa, qb):
    """Angle in radians between two unit quaternions (sign-insensitive)."""
    dot = abs(sum(a * b for a, b in zip(qa, qb)))
    return 2.0 * math.acos(min(1.0, dot))


def compose(pose_a, pose_b):
    """(p, R) composition: T_a * T_b."""
    p_a, r_a = pose_a
    p_b, r_b = pose_b
    p = [p_a[i] + mat_vec(r_a, p_b)[i] for i in range(3)]
    return p, mat_mul(r_a, r_b)


def fmt(value):
    """Shortest round-trip decimal for a double; '0' for exact zero."""
    if value == 0.0:
        return "0"
    return repr(float(value))


def fmt_vec(values):
    return " ".join(fmt(v) for v in values)


# ---------------------------------------------------------------------------
# Source loading, hash-gated against the provenance record.
# ---------------------------------------------------------------------------


def sha256_of(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_hash(path, recorded, label):
    actual = sha256_of(path)
    if actual != recorded:
        sys.exit(
            f"REFUSING to generate: {label} '{path}' hash {actual} does not "
            f"match the provenance record {recorded}. Re-audit the source "
            f"and update model_provenance.yaml first."
        )


def load_sources():
    with open(PROVENANCE_PATH, "r") as handle:
        prov = yaml.safe_load(handle)

    urdf_path = REPO_ROOT / prov["source_urdf"]["path"]
    require_hash(urdf_path, prov["source_urdf"]["sha256"], "URDF")

    mjcf_path = Path(prov["source_mjcf"]["path"])
    require_hash(mjcf_path, prov["source_mjcf"]["sha256"], "gen3.xml")

    meshes = []
    for entry in prov["source_meshes"]:
        mesh_path = Path(entry["path"])
        require_hash(mesh_path, entry["sha256"], "mesh")
        meshes.append((mesh_path, entry["sha256"]))

    return urdf_path, mjcf_path, meshes


def parse_urdf_joints(urdf_path):
    """name -> dict(xyz, rpy as float lists; lower/upper as verbatim text;
    axis as verbatim text + float list, URDF default '1 0 0' if absent)."""
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for joint in root.iter("joint"):
        origin = joint.find("origin")
        limit = joint.find("limit")
        axis = joint.find("axis")
        # URDF spec default when <axis> is omitted is (1, 0, 0) — NOT z.
        axis_text = axis.get("xyz") if axis is not None else "1 0 0"
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "xyz": [float(v) for v in origin.get("xyz").split()],
            "rpy": [float(v) for v in origin.get("rpy").split()],
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "lower": limit.get("lower") if limit is not None else None,
            "upper": limit.get("upper") if limit is not None else None,
            "axis_text": axis_text,
            "axis": [float(v) for v in axis_text.split()],
        }
    return joints


def require_z_axis(joint_name, urdf_joint):
    """Every arm joint must rotate about local z, per the URDF <axis>.

    ValidateAndResolveModel and the emitted MJCF both assume local-z hinges;
    this assert anchors that assumption in the kinematic authority itself,
    so a future URDF axis change refuses to generate instead of producing a
    silently wrong model that passes the structural contract."""
    if urdf_joint["axis"] != [0.0, 0.0, 1.0]:
        sys.exit(
            f"REFUSING: joint '{joint_name}' URDF axis is "
            f"'{urdf_joint['axis_text']}', but the generated model and "
            f"ValidateAndResolveModel assume local z [0 0 1]. Update the "
            f"generator and the contract together, deliberately."
        )


def parse_gen3(mjcf_path):
    """Mechanics: defaults, assets, option, per-body inertials, actuators,
    home keyframe, and the source body poses (used only as a pairing sanity
    check against the URDF, never as placement truth)."""
    root = ET.parse(mjcf_path).getroot()

    chain = []
    body = root.find("worldbody").find("body")
    while body is not None:
        chain.append(body)
        body = body.find("body")
    names = [b.get("name") for b in chain]
    if names != GEN3_BODY_NAMES:
        sys.exit(f"REFUSING: gen3.xml chain {names} != expected {GEN3_BODY_NAMES}")

    inertials = {b.get("name"): b.find("inertial") for b in chain}
    source_poses = {
        b.get("name"): (
            [float(v) for v in (b.get("pos") or "0 0 0").split()],
            [float(v) for v in (b.get("quat") or "1 0 0 0").split()],
        )
        for b in chain
    }

    actuators = []
    for position in root.find("actuator").findall("position"):
        actuators.append(
            {
                "joint": position.get("joint"),
                "class": position.get("class"),
                "ctrlrange": position.get("ctrlrange"),
            }
        )
    if [a["joint"] for a in actuators] != [f"joint_{i}" for i in range(1, 8)]:
        sys.exit("REFUSING: gen3.xml actuator order is not joint_1..joint_7")

    home_qpos = None
    for key in root.find("keyframe").findall("key"):
        if key.get("name") == "home":
            home_qpos = key.get("qpos").split()
    if home_qpos is None or len(home_qpos) != 7:
        sys.exit("REFUSING: gen3.xml home keyframe not found or not 7 joints")

    return {
        "root": root,
        "default": root.find("default"),
        "asset": root.find("asset"),
        "option": root.find("option"),
        "inertials": inertials,
        "source_poses": source_poses,
        "actuators": actuators,
        "home_qpos": home_qpos,
    }


# ---------------------------------------------------------------------------
# Model construction.
# ---------------------------------------------------------------------------


def urdf_origin_pose(joint):
    return joint["xyz"], rpy_to_matrix(*joint["rpy"])


def check_pairing(urdf_joint, gen3_pose, label):
    """The URDF chain and the gen3 chain must describe the same geometry to
    within URDF text rounding (~1e-5 rad); a larger gap means the bodies
    were mis-paired and the inertial/mesh would land on the wrong link."""
    pos, rot = urdf_origin_pose(urdf_joint)
    gen3_pos, gen3_quat = gen3_pose
    norm = math.sqrt(sum(c * c for c in gen3_quat))
    gen3_quat = [c / norm for c in gen3_quat]
    for i in range(3):
        if abs(pos[i] - gen3_pos[i]) > 1e-9:
            sys.exit(
                f"REFUSING: {label} position URDF {pos} vs gen3 {gen3_pos}"
            )
    angle = quat_angle_between(matrix_to_quat(rot), gen3_quat)
    if angle > 1e-4:
        sys.exit(
            f"REFUSING: {label} orientation differs from gen3 by "
            f"{angle:.2e} rad (pairing error, not rounding)"
        )


def add_geoms(body_elem, mesh_name):
    ET.SubElement(body_elem, "geom", {"class": "visual", "mesh": mesh_name})
    ET.SubElement(body_elem, "geom", {"class": "collision", "mesh": mesh_name})


def build_arm(mount_elem, side, urdf_joints, gen3):
    cfg = ARMS[side]
    mount_joint = urdf_joints[cfg["mount_joint"]]

    # mount_T_base: the URDF fixed joint DualArmKinematics::MountFromBase
    # reads through Pinocchio — carried here at full double precision.
    pos, rot = urdf_origin_pose(mount_joint)
    base = ET.SubElement(
        mount_elem,
        "body",
        {
            "name": f"{side}_base_link",
            "pos": fmt_vec(pos),
            "quat": fmt_vec(matrix_to_quat(rot)),
        },
    )
    base.append(copy.deepcopy(gen3["inertials"]["base_link"]))
    add_geoms(base, "base_link")

    parent = base
    for index, joint_name in enumerate(cfg["chain_joints"], start=1):
        urdf_joint = urdf_joints[joint_name]
        body_name = GEN3_BODY_NAMES[index]
        check_pairing(
            urdf_joint,
            gen3["source_poses"][body_name],
            f"{side} {body_name}",
        )
        pos, rot = urdf_origin_pose(urdf_joint)
        body = ET.SubElement(
            parent,
            "body",
            {
                "name": f"{side}_{body_name}",
                "pos": fmt_vec(pos),
                "quat": fmt_vec(matrix_to_quat(rot)),
            },
        )
        # Axis comes from the URDF <axis> element (verbatim text), gated
        # by require_z_axis — never a hand-typed literal.
        require_z_axis(joint_name, urdf_joint)
        joint_attrs = {
            "name": f"{side}_joint_{index}",
            "axis": urdf_joint["axis_text"],
        }
        if urdf_joint["type"] == "revolute":
            # Bounded range verbatim from the URDF limit text (the
            # kinematic authority) — identical doubles after parsing.
            joint_attrs["range"] = f"{urdf_joint['lower']} {urdf_joint['upper']}"
        elif urdf_joint["type"] != "continuous":
            sys.exit(f"REFUSING: {joint_name} is {urdf_joint['type']}")
        ET.SubElement(body, "joint", joint_attrs)
        body.append(copy.deepcopy(gen3["inertials"][body_name]))
        mesh = (
            "bracelet_with_vision_link"
            if body_name == "bracelet_link"
            else body_name
        )
        add_geoms(body, mesh)
        parent = body

    # TCP site in the bracelet body: composition of the URDF fixed joints
    # from Bracelet_Link to the production TCP frame.
    tcp_pose = ([0.0, 0.0, 0.0], rpy_to_matrix(0.0, 0.0, 0.0))
    for fixed_name in cfg["tcp_joints"]:
        tcp_pose = compose(tcp_pose, urdf_origin_pose(urdf_joints[fixed_name]))
    ET.SubElement(
        parent,
        "site",
        {
            "name": f"{side}_tcp",
            "pos": fmt_vec(tcp_pose[0]),
            "quat": fmt_vec(matrix_to_quat(tcp_pose[1])),
        },
    )


def build_model(urdf_joints, gen3):
    root = ET.Element("mujoco", {"model": "humansl_dual_gen3"})
    # angle="radian" is load-bearing: MJCF defaults to DEGREES.
    ET.SubElement(
        root, "compiler", {"angle": "radian", "meshdir": "assets"}
    )
    root.append(copy.deepcopy(gen3["option"]))
    root.append(copy.deepcopy(gen3["default"]))
    root.append(copy.deepcopy(gen3["asset"]))

    # Scene dressing. Every element here is VISUAL ONLY: each geom carries
    # contype="0" conaffinity="0", so nothing below is collidable and the
    # dynamics, contact set and every gated number are unchanged. Without
    # it the model has no light and renders black in the viewer.
    #
    # The torso box is a LOOK-AND-SCALE PROXY, not a measured wearer model:
    # its half-extents come from the reference simulation's torso
    # (msc_project/sim/scene.xml), and the real rig's torso has never been
    # measured into this repository. Nothing may use it for clearance,
    # collision or any physical claim — see the engineering contract's rule
    # that Christian owns physical definitions.
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", {
        "ambient": "0.4 0.4 0.4", "diffuse": "0.7 0.7 0.7",
        "specular": "0.1 0.1 0.1",
    })
    ET.SubElement(visual, "rgba", {"haze": "0.15 0.25 0.35 1"})

    scene_asset = root.find("asset")
    ET.SubElement(scene_asset, "texture", {
        "type": "skybox", "builtin": "gradient",
        "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0", "width": "512", "height": "512",
    })
    ET.SubElement(scene_asset, "texture", {
        "name": "floor_tex", "type": "2d", "builtin": "checker",
        "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3",
        "width": "512", "height": "512",
    })
    ET.SubElement(scene_asset, "material", {
        "name": "floor_mat", "texture": "floor_tex",
        "texrepeat": "5 5", "reflectance": "0.1",
    })

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", {
        "directional": "true", "pos": "0 0 3", "dir": "0 0 -1",
        "diffuse": "0.7 0.7 0.7",
    })
    # The rig hangs around the Mount anchor at the world origin, so the
    # floor sits FLOOR_DROP_M below it purely so the view has a ground
    # plane. It is not collidable and the arms never touch it.
    ET.SubElement(worldbody, "geom", {
        "name": "visual_floor", "type": "plane", "size": "3 3 0.05",
        "pos": f"0 0 {fmt(-FLOOR_DROP_M)}", "material": "floor_mat",
        "contype": "0", "conaffinity": "0", "group": "2",
    })
    # No decorative sites here: ModelContract pins nsite to exactly the two
    # production TCP sites, and that strictness is worth more than a marker.
    ET.SubElement(worldbody, "camera", {
        "name": "overview", "pos": "1.8 -1.8 0.9",
        "xyaxes": "0.7 0.7 0 -0.3 0.3 0.9",
    })

    # The Mount is a MOCAP body (decision 2026-08-17): welded to world at
    # the pose written into mjData.mocap_pos / mocap_quat each control
    # tick, with both arms as its direct jointed children. The arms
    # therefore carry real gravity and are carried kinematically by the
    # Mount, which is what the wearable rig does. It replaces a freejoint
    # whose qpos/qvel were rewritten every substep: that left the
    # freejoint ACCELERATION free, so the whole rig fell freely inside
    # each substep and the arms were weightless.
    #
    # A mocap body is static (no joint, infinite effective mass), so it
    # carries no inertial element: the Mount's mass and inertia are not
    # part of the dynamics and no placeholder values are invented here.
    mount = ET.SubElement(
        worldbody, "body", {"name": "mount", "mocap": "true"}
    )

    # Visual torso proxy, carried by the Mount so the viewer shows what the
    # arms are bolted to. Not collidable, not measured — see the note above.
    ET.SubElement(mount, "geom", {
        "name": "torso_proxy_visual", "type": "box",
        "size": fmt_vec(TORSO_HALF_EXTENTS_M),
        "pos": fmt_vec(TORSO_CENTRE_IN_MOUNT_M),
        "rgba": "0.7 0.5 0.4 0.6",
        "contype": "0", "conaffinity": "0", "group": "2",
    })

    build_arm(mount, "right", urdf_joints, gen3)
    build_arm(mount, "left", urdf_joints, gen3)

    # Contact exclusions the mocap Mount makes necessary. MuJoCo filters
    # contacts between a parent body and its child, EXCEPT when the parent
    # is welded to the world — which every <side>_base_link now is,
    # hanging jointless off the world-welded Mount. Without these two
    # excludes the base and shoulder meshes of each arm collide at the
    # home posture (probe 2026-08-17: 8 penetrating contacts, deepest
    # -12.05 mm) and inject forces no real arm feels.
    #
    # What they restore, precisely: the SAME-ARM base/shoulder filtering
    # the freejoint model got for free. They do NOT restore all of that
    # model's filtering, and deliberately so. Welding the bases to world
    # also un-filtered two CROSS-arm pairs — left_base_link <->
    # right_shoulder_link and right_base_link <-> left_shoulder_link —
    # which the freejoint's weld tree had swallowed as a parent/child
    # relation (every base_link shared the Mount's weld id then; each is
    # its own world-weld now). Probe 2026-08-17, all geom margins forced
    # to 50 m so only the filter can reject a pair: 105 collidable body
    # pairs in this model, 103 in a reconstructed freejoint model, the
    # difference being exactly those two. They stay live because they are
    # genuine interference between two separate arms; the freejoint model
    # was hiding real collisions, not modelling them. Consequence for any
    # ncon == 0 acceptance check: a cross-arm base/shoulder approach now
    # registers where the Plan 02 model would have ignored it.
    #
    # Christian's reference simulation does the identical thing for the
    # identical reason (msc_project/sim/scene.xml: mocap torso, these two
    # excludes, same explanation in its comment).
    contact = ET.SubElement(root, "contact")
    for side in ("right", "left"):
        ET.SubElement(
            contact,
            "exclude",
            {
                "body1": f"{side}_{BODY_NAMES[0]}",
                "body2": f"{side}_{BODY_NAMES[1]}",
            },
        )

    actuator = ET.SubElement(root, "actuator")
    for side in ("right", "left"):
        for index, entry in enumerate(gen3["actuators"], start=1):
            attrs = {
                "class": entry["class"],
                "name": f"{side}_joint_{index}",
                "joint": f"{side}_joint_{index}",
            }
            if entry["ctrlrange"] is not None:
                attrs["ctrlrange"] = entry["ctrlrange"]
            ET.SubElement(actuator, "position", attrs)

    # Home keyframe (mechanics source): both arms at the gen3 home posture
    # and the Mount at world identity. qpos is now the 14 hinges alone;
    # the Mount pose lives in the keyframe's mocap fields (mpos/mquat),
    # which mj_resetDataKeyframe restores into mjData.mocap_pos/mocap_quat.
    # ctrl matches qpos, so the servos start commanding exactly the seeded
    # posture — they are gravity-loaded from the first step and droop from
    # it, which is the physical behaviour the freejoint model could not
    # show.
    keyframe = ET.SubElement(root, "keyframe")
    home = " ".join(gen3["home_qpos"])
    ET.SubElement(
        keyframe,
        "key",
        {
            "name": "home",
            "qpos": f"{home} {home}",
            "ctrl": f"{home} {home}",
            "mpos": "0 0 0",
            "mquat": "1 0 0 0",
        },
    )
    return root


def main():
    urdf_path, mjcf_path, meshes = load_sources()
    urdf_joints = parse_urdf_joints(urdf_path)
    gen3 = parse_gen3(mjcf_path)

    root = build_model(urdf_joints, gen3)
    ET.indent(root, space="  ")

    header = (
        "\n  GENERATED FILE — DO NOT EDIT BY HAND.\n"
        "  Generator: Christian_control/simulation/tools/"
        "generate_dual_mjcf.py\n"
        "  Kinematic authority (all placements, ranges, TCP sites): "
        "Christian_control/model/GEN3_dual_mounted.urdf\n"
        "  Mechanics source (inertials, meshes, actuator classes, "
        "keyframe): msc_project gen3.xml (generic plant, not a Kinova "
        "servo model)\n"
        "  Source hashes are pinned in model/model_provenance.yaml, which "
        "also pins this file's SHA-256: regenerate and update the record "
        "instead of editing.\n"
    )
    xml_text = (
        f"<!--{header}-->\n"
        + ET.tostring(root, encoding="unicode")
        + "\n"
    )

    OUTPUT_XML.write_text(xml_text)

    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    for mesh_path, recorded_hash in meshes:
        target = ASSET_DIR / mesh_path.name
        shutil.copyfile(mesh_path, target)
        copied_hash = sha256_of(target)
        if copied_hash != recorded_hash:
            sys.exit(f"copy of {mesh_path.name} does not match its source hash")
        print(f"asset  {target.name}: {copied_hash}")

    print(f"model  {OUTPUT_XML.relative_to(REPO_ROOT)}: {sha256_of(OUTPUT_XML)}")


if __name__ == "__main__":
    main()
