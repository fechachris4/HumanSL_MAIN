#!/usr/bin/env python3
"""Provenance gate for the dual-arm MuJoCo execution twin (Plan 02 Task 1).

Checks Christian_control/simulation/model/model_provenance.yaml against the
files it claims to describe. Cross-checks are made against independent
sources (the production URDF, the gen3.xml compiler block and mesh list,
the vendored mujoco.h version macro) so a hand-typed or stale provenance
entry fails rather than being trusted.

Evidence class: unit test (file hashing and XML parsing only; nothing is
simulated and no robot-facing binary is touched).

Run directly:
    python3 Christian_control/simulation/tests/test_model_provenance.py
"""

import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
PROVENANCE_PATH = (
    REPO_ROOT / "Christian_control" / "simulation" / "model" / "model_provenance.yaml"
)

# The production URDF is the kinematic authority. Pointing provenance at a
# copy would defeat the record, so the exact repository-relative path is
# required, not just any existing file.
EXPECTED_URDF_REL = "Christian_control/basic_control/config/GEN3_dual_mounted.urdf"
EXPECTED_MUJOCO_LIB_REL = "third_party/lib/libmujoco.so.3.10.0"
EXPECTED_MUJOCO_DOC_REL = "third_party/MUJOCO_PROVENANCE.md"

# Production TCP frames are asymmetric BY DESIGN: right carries the
# configured tool, left is the bare flange. They are different physical
# points and must never be compared as equivalents. pinch_site is a
# menagerie gripper site and is rejected as either production TCP.
EXPECTED_GENERATED_XML_REL = (
    "Christian_control/simulation/model/humansl_dual_gen3.xml"
)
EXPECTED_GENERATOR_REL = (
    "Christian_control/simulation/tools/generate_dual_mjcf.py"
)
EXPECTED_ASSET_DIR_REL = "Christian_control/simulation/model/assets"

EXPECTED_RIGHT_TCP = "ConfiguredTool_Link"
EXPECTED_LEFT_TCP = "leftEndEffector_Link"
REQUIRED_FRAMES = {"world", "mount", "right_base", "left_base"}

_failures = []


def check(condition, message):
    if not condition:
        _failures.append(message)


def sha256_of(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_repo_relative(entry_name, path_str):
    """A repository file must be recorded repository-relative: no leading
    '/', no '..', and it must exist under the repository root."""
    check(isinstance(path_str, str) and path_str, f"{entry_name}: path missing")
    if not isinstance(path_str, str) or not path_str:
        return None
    check(not path_str.startswith("/"),
          f"{entry_name}: '{path_str}' must be repository-relative, not absolute")
    check(".." not in Path(path_str).parts,
          f"{entry_name}: '{path_str}' must not contain '..'")
    resolved = REPO_ROOT / path_str
    check(resolved.is_file(), f"{entry_name}: '{path_str}' does not exist under repo root")
    return resolved if resolved.is_file() else None


def resolve_external_absolute(entry_name, path_str):
    """An out-of-repository source (msc_project) must be recorded as an
    absolute path so the record is unambiguous, and must exist."""
    check(isinstance(path_str, str) and path_str.startswith("/"),
          f"{entry_name}: '{path_str}' must be an absolute path (external source)")
    if not (isinstance(path_str, str) and path_str.startswith("/")):
        return None
    resolved = Path(path_str)
    check(resolved.is_file(), f"{entry_name}: '{path_str}' does not exist")
    return resolved if resolved.is_file() else None


def check_hash(entry_name, path, recorded_hash):
    check(isinstance(recorded_hash, str) and len(recorded_hash) == 64,
          f"{entry_name}: sha256 missing or malformed")
    if path is None or not isinstance(recorded_hash, str):
        return
    actual = sha256_of(path)
    check(actual == recorded_hash,
          f"{entry_name}: sha256 mismatch for {path}\n"
          f"    recorded {recorded_hash}\n    actual   {actual}")


def main():
    if not PROVENANCE_PATH.is_file():
        print(f"FAIL: provenance file absent: {PROVENANCE_PATH}")
        return 1

    with open(PROVENANCE_PATH, "r") as handle:
        doc = yaml.safe_load(handle)
    check(isinstance(doc, dict), "provenance file is not a YAML mapping")
    if not isinstance(doc, dict):
        return report()

    # ---- units: explicit, never implied -------------------------------
    units = doc.get("units", {})
    check(units.get("length") == "metres", "units.length must be 'metres'")
    check(units.get("angle") == "radians", "units.angle must be 'radians'")
    check(units.get("time") == "seconds", "units.time must be 'seconds'")

    # ---- vendored MuJoCo ----------------------------------------------
    mujoco = doc.get("mujoco", {})
    check(mujoco.get("version") == "3.10.0", "mujoco.version must be '3.10.0'")
    check(mujoco.get("library") == EXPECTED_MUJOCO_LIB_REL,
          f"mujoco.library must be '{EXPECTED_MUJOCO_LIB_REL}'")
    lib_path = resolve_repo_relative("mujoco.library", mujoco.get("library"))
    check_hash("mujoco.library", lib_path, mujoco.get("sha256"))
    check(mujoco.get("provenance_doc") == EXPECTED_MUJOCO_DOC_REL,
          f"mujoco.provenance_doc must be '{EXPECTED_MUJOCO_DOC_REL}'")
    resolve_repo_relative("mujoco.provenance_doc", mujoco.get("provenance_doc"))

    # Independent cross-check: the vendored header's version macro
    # (major*1000000 + minor*1000 + patch) must agree with the recorded
    # version string, so a silent library swap fails here.
    header = REPO_ROOT / "third_party" / "include" / "mujoco" / "mujoco.h"
    check(header.is_file(), "vendored mujoco.h is missing")
    if header.is_file():
        macro = None
        for line in header.read_text().splitlines():
            if "mjVERSION_HEADER" in line and line.strip().startswith("#define"):
                macro = int(line.split()[-1])
                break
        check(macro is not None, "mjVERSION_HEADER not found in mujoco.h")
        if macro is not None:
            major, minor, patch = macro // 1000000, (macro // 1000) % 1000, macro % 1000
            check(f"{major}.{minor}.{patch}" == mujoco.get("version"),
                  f"mujoco.h says {major}.{minor}.{patch}, "
                  f"provenance says {mujoco.get('version')}")

    # ---- HumanSL URDF: the kinematic authority ------------------------
    urdf_entry = doc.get("source_urdf", {})
    check(urdf_entry.get("path") == EXPECTED_URDF_REL,
          f"source_urdf.path must be '{EXPECTED_URDF_REL}'")
    urdf_path = resolve_repo_relative("source_urdf", urdf_entry.get("path"))
    check_hash("source_urdf", urdf_path, urdf_entry.get("sha256"))
    check(urdf_entry.get("kinematic_authority") is True,
          "source_urdf.kinematic_authority must be true")

    # ---- msc_project MJCF: mechanics source, never kinematic truth ----
    mjcf_entry = doc.get("source_mjcf", {})
    mjcf_path = resolve_external_absolute("source_mjcf", mjcf_entry.get("path"))
    check_hash("source_mjcf", mjcf_path, mjcf_entry.get("sha256"))
    check(mjcf_entry.get("kinematic_authority") is False,
          "source_mjcf.kinematic_authority must be false (mechanics only)")
    check(mjcf_entry.get("compiler_angle") == "radian",
          "source_mjcf.compiler_angle must record 'radian' "
          "(MJCF defaults to degrees; this trap must stay visible)")

    referenced_meshes = set()
    if mjcf_path is not None:
        mjcf_root = ET.parse(mjcf_path).getroot()
        compiler = mjcf_root.find("compiler")
        check(compiler is not None and compiler.get("angle") == "radian",
              "gen3.xml compiler angle attribute is not 'radian' — "
              "provenance and source disagree")
        referenced_meshes = {
            mesh.get("file") for mesh in mjcf_root.iter("mesh") if mesh.get("file")
        }
        check(referenced_meshes, "gen3.xml references no meshes — wrong source file?")

    # ---- meshes: every MJCF-referenced mesh recorded with a hash ------
    meshes = doc.get("source_meshes", [])
    check(isinstance(meshes, list) and meshes, "source_meshes must be a non-empty list")
    recorded_basenames = set()
    if isinstance(meshes, list):
        for index, mesh in enumerate(meshes):
            name = f"source_meshes[{index}]"
            if not isinstance(mesh, dict):
                check(False, f"{name}: entry must be a mapping")
                continue
            mesh_path = resolve_external_absolute(name, mesh.get("path"))
            check_hash(name, mesh_path, mesh.get("sha256"))
            if isinstance(mesh.get("path"), str):
                recorded_basenames.add(Path(mesh["path"]).name)
    if referenced_meshes:
        check(recorded_basenames == referenced_meshes,
              "source_meshes must record exactly the meshes gen3.xml references\n"
              f"    missing {sorted(referenced_meshes - recorded_basenames)}\n"
              f"    extra   {sorted(recorded_basenames - referenced_meshes)}")

    # ---- asset licence -------------------------------------------------
    license_entry = doc.get("source_license", {})
    license_path = resolve_external_absolute("source_license", license_entry.get("path"))
    check_hash("source_license", license_path, license_entry.get("sha256"))
    check(bool(license_entry.get("spdx")), "source_license.spdx must be recorded")

    # ---- generated model (Task 2): pinned artifact, never hand-edited --
    generated = doc.get("generated_model", {})
    check(generated.get("path") == EXPECTED_GENERATED_XML_REL,
          f"generated_model.path must be '{EXPECTED_GENERATED_XML_REL}'")
    generated_path = resolve_repo_relative("generated_model",
                                           generated.get("path"))
    check_hash("generated_model", generated_path, generated.get("sha256"))
    check(generated.get("generator") == EXPECTED_GENERATOR_REL,
          f"generated_model.generator must be '{EXPECTED_GENERATOR_REL}'")
    resolve_repo_relative("generated_model.generator",
                          generated.get("generator"))
    check(generated.get("hand_edits") == "forbidden",
          "generated_model.hand_edits must declare 'forbidden' — every "
          "correction goes through the generator")

    if generated_path is not None:
        generated_root = ET.parse(generated_path).getroot()
        generated_compiler = generated_root.find("compiler")
        check(generated_compiler is not None
              and generated_compiler.get("angle") == "radian",
              "generated model compiler angle must be 'radian' "
              "(MJCF defaults to degrees)")
        site_names = {site.get("name") for site in generated_root.iter("site")}
        check("pinch_site" not in site_names,
              "pinch_site must not exist in the generated model")
        check({"right_tcp", "left_tcp"} <= site_names,
              "generated model must define right_tcp and left_tcp sites")

    # ---- copied assets: byte-identical to the audited sources ----------
    copied = doc.get("copied_assets", {})
    check(copied.get("dir") == EXPECTED_ASSET_DIR_REL,
          f"copied_assets.dir must be '{EXPECTED_ASSET_DIR_REL}'")
    assets_dir = REPO_ROOT / EXPECTED_ASSET_DIR_REL
    if isinstance(meshes, list):
        for mesh in meshes:
            if not isinstance(mesh, dict) or not isinstance(mesh.get("path"), str):
                continue
            basename = Path(mesh["path"]).name
            copy_path = assets_dir / basename
            check(copy_path.is_file(),
                  f"copied asset missing: {copy_path}")
            if copy_path.is_file() and isinstance(mesh.get("sha256"), str):
                actual = sha256_of(copy_path)
                check(actual == mesh["sha256"],
                      f"copied asset {basename} hash {actual} differs from "
                      f"its audited source {mesh['sha256']} — the local "
                      f"copy was edited")

    # ---- frames ---------------------------------------------------------
    frames = doc.get("frames", [])
    check(isinstance(frames, list), "frames must be a list of frame names")
    if isinstance(frames, list):
        missing = REQUIRED_FRAMES - set(frames)
        check(not missing, f"frames missing required names: {sorted(missing)}")

    # ---- production TCPs ------------------------------------------------
    tcp = doc.get("tcp", {})
    right_tcp = tcp.get("right", {}).get("frame")
    left_tcp = tcp.get("left", {}).get("frame")
    check(right_tcp == EXPECTED_RIGHT_TCP,
          f"tcp.right.frame must be '{EXPECTED_RIGHT_TCP}', got '{right_tcp}'")
    check(left_tcp == EXPECTED_LEFT_TCP,
          f"tcp.left.frame must be '{EXPECTED_LEFT_TCP}', got '{left_tcp}'")
    check(right_tcp != left_tcp,
          "right and left TCP frames are different physical points by design")
    tcp_text = yaml.safe_dump(tcp).lower()
    check("pinch_site" not in tcp_text,
          "pinch_site is rejected as a production TCP")

    # TCP frames must exist as links in the URDF (independent of the yaml).
    if urdf_path is not None:
        urdf_root = ET.parse(urdf_path).getroot()
        link_names = {link.get("name") for link in urdf_root.iter("link")}
        check(EXPECTED_RIGHT_TCP in link_names,
              f"URDF has no link '{EXPECTED_RIGHT_TCP}'")
        check(EXPECTED_LEFT_TCP in link_names,
              f"URDF has no link '{EXPECTED_LEFT_TCP}'")

        # ---- Mount transforms cross-checked against the URDF ----------
        # The runtime authority is DualArmKinematics::MountFromBase, which
        # reads these same fixed joints through Pinocchio. The yaml must
        # match the URDF text exactly (zero tolerance): these numbers are
        # copied, never re-derived, so any difference is a transcription
        # error.
        joints = {joint.get("name"): joint for joint in urdf_root.iter("joint")}
        mounts = doc.get("mount_transforms", {})
        for side, joint_name in (("right", "right_base_mount"),
                                 ("left", "left_base_mount")):
            joint = joints.get(joint_name)
            check(joint is not None, f"URDF joint '{joint_name}' not found")
            entry = mounts.get(side, {})
            check(entry.get("urdf_joint") == joint_name,
                  f"mount_transforms.{side}.urdf_joint must be '{joint_name}'")
            if joint is None or not isinstance(entry, dict):
                continue
            origin = joint.find("origin")
            urdf_rpy = [float(v) for v in origin.get("rpy").split()]
            urdf_xyz = [float(v) for v in origin.get("xyz").split()]
            check(entry.get("rpy_rad") == urdf_rpy,
                  f"mount_transforms.{side}.rpy_rad {entry.get('rpy_rad')} "
                  f"!= URDF {urdf_rpy}")
            check(entry.get("xyz_m") == urdf_xyz,
                  f"mount_transforms.{side}.xyz_m {entry.get('xyz_m')} "
                  f"!= URDF {urdf_xyz}")
            check(entry.get("parent") == joint.find("parent").get("link"),
                  f"mount_transforms.{side}.parent != URDF parent link")
            check(entry.get("child") == joint.find("child").get("link"),
                  f"mount_transforms.{side}.child != URDF child link")

    return report()


def report():
    if _failures:
        print(f"FAIL: {len(_failures)} provenance check(s) failed")
        for failure in _failures:
            print(f"  - {failure}")
        return 1
    print("PASS: model provenance is complete and matches its sources")
    return 0


if __name__ == "__main__":
    sys.exit(main())
