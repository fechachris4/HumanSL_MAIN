#!/usr/bin/env python3
"""Turn planning/config/joint_limits.yaml into a constexpr C++ header.

Why this exists: the physical Kinova limits used to live in five places at
once (this yaml, control/Config.h, analytical_ik.h, quik_solveIK.h, the
URDF's <limit> tags) and had already drifted apart in the third digit.  The
yaml is now the one authoritative source; every C++ consumer reads the header
this script writes, so a disagreement is impossible rather than merely
discouraged.

Generated at build time into the build tree, exactly like dh_params_tool.yaml
is generated from the URDF.  Never committed, never hand-edited.  Editing the
yaml regenerates it on the next build.

Usage: generate_joint_limits.py <input.yaml> <output.h>
"""

import math
import sys

import yaml

JOINTS = 7
CONTINUOUS_SENTINEL = 0.0  # what Config.h has always used for "no limit"


def load(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


def positions(doc):
    """(lower_deg, upper_deg, bounded_mask) in Kortex actuator order.

    A continuous joint (1/3/5/7) carries `continuous: true` and no numeric
    limit.  It gets the zero sentinel and a zero mask, which is the same
    representation every existing consumer already expects; it is NOT given a
    fabricated angle, because that would make an unbounded joint look bounded.
    """
    lower, upper, mask = [], [], []
    for i in range(1, JOINTS + 1):
        entry = doc["position_limits"][f"actuator_{i}"]
        if entry.get("continuous", False):
            lower.append(CONTINUOUS_SENTINEL)
            upper.append(CONTINUOUS_SENTINEL)
            mask.append(0)
            continue
        lower.append(float(entry["lower_limit"]))
        upper.append(float(entry["upper_limit"]))
        mask.append(1)
    return lower, upper, mask


def velocities(doc):
    """Physical hard limits, deg/s.  The yaml stores them in rad/s."""
    out = []
    for i in range(1, JOINTS + 1):
        entry = doc["velocity_limits"][f"actuator_{i}"]
        rad_s = float(entry["upper_limit"])
        out.append(math.degrees(rad_s))
    return out


def array(name, values, fmt="{:.6f}"):
    body = ", ".join(fmt.format(v) for v in values)
    return f"    inline constexpr std::array<double, 7> {name} = {{{body}}};\n"


def mask_array(name, values):
    body = ", ".join(str(v) for v in values)
    return f"    inline constexpr std::array<double, 7> {name} = {{{body}}};\n"


def main(argv):
    if len(argv) != 3:
        sys.stderr.write(__doc__)
        return 2
    doc = load(argv[1])
    margins = doc["margins"]

    pos_planner = float(margins["position_planner_margin_deg"])
    pos_controller = float(margins["position_controller_margin_deg"])
    vel_planner = float(margins["velocity_planner_fraction"])
    vel_controller = float(margins["velocity_controller_fraction"])

    # The ordering the whole design rests on.  Checked here as well as in the
    # generated static_asserts so a bad yaml fails the build at the earliest
    # possible point, with a message that names the file.
    if not pos_planner > pos_controller > 0.0:
        raise SystemExit(
            f"{argv[1]}: margins must satisfy "
            f"position_planner_margin_deg ({pos_planner}) > "
            f"position_controller_margin_deg ({pos_controller}) > 0"
        )
    if not 0.0 < vel_planner < vel_controller < 1.0:
        raise SystemExit(
            f"{argv[1]}: margins must satisfy 0 < "
            f"velocity_planner_fraction ({vel_planner}) < "
            f"velocity_controller_fraction ({vel_controller}) < 1"
        )

    lower, upper, mask = positions(doc)
    vel = velocities(doc)

    def soft(values, margin):
        # Shrink a bounded limit toward zero by `margin`; leave the continuous
        # sentinel alone so downstream masks keep working unchanged.
        return [
            v - math.copysign(margin, v) if m else CONTINUOUS_SENTINEL
            for v, m in zip(values, mask)
        ]

    out = []
    out.append("// GENERATED FILE - DO NOT EDIT.\n")
    out.append(f"// Written by {__file__.split('/')[-1]} from\n")
    out.append(f"// {argv[1]}\n")
    out.append("// Edit that yaml instead; this header is rebuilt from it.\n")
    out.append("#pragma once\n\n#include <array>\n\n")
    out.append("namespace config::limits\n{\n")

    out.append("    // Physical Kinova limits. The boundary, never an operating\n")
    out.append("    // figure. Continuous joints (1/3/5/7) carry the 0 sentinel.\n")
    out.append(array("kPhysicalLowerDeg", lower))
    out.append(array("kPhysicalUpperDeg", upper))
    out.append(mask_array("kBoundedMask", mask))
    out.append(array("kPhysicalVelocityDegS", vel))
    out.append("\n")

    out.append("    // The one margin mechanism. Planner stricter than\n")
    out.append("    // controller, controller stricter than hardware.\n")
    out.append(f"    inline constexpr double kPositionPlannerMarginDeg = {pos_planner};\n")
    out.append(f"    inline constexpr double kPositionControllerMarginDeg = {pos_controller};\n")
    out.append(f"    inline constexpr double kVelocityPlannerFraction = {vel_planner};\n")
    out.append(f"    inline constexpr double kVelocityControllerFraction = {vel_controller};\n")
    out.append("\n")

    out.append("    // Derived operating limits. No min() chain, no second margin.\n")
    out.append(array("kPlannerLowerDeg", soft(lower, pos_planner)))
    out.append(array("kPlannerUpperDeg", soft(upper, pos_planner)))
    out.append(array("kControllerLowerDeg", soft(lower, pos_controller)))
    out.append(array("kControllerUpperDeg", soft(upper, pos_controller)))
    out.append(array("kPlannerVelocityDegS", [v * vel_planner for v in vel]))
    out.append(array("kControllerVelocityDegS", [v * vel_controller for v in vel]))
    out.append("\n")

    out.append("    static_assert(kPositionPlannerMarginDeg >\n")
    out.append("                  kPositionControllerMarginDeg,\n")
    out.append("                  \"planner must stop before the controller does\");\n")
    out.append("    static_assert(kPositionControllerMarginDeg > 0.0,\n")
    out.append("                  \"controller must stop before the hardware limit\");\n")
    out.append("    static_assert(kVelocityPlannerFraction <\n")
    out.append("                  kVelocityControllerFraction,\n")
    out.append("                  \"planner must be slower than the controller allows\");\n")
    out.append("    static_assert(kVelocityControllerFraction < 1.0,\n")
    out.append("                  \"controller must stay inside the hard speed limit\");\n")
    out.append("}\n")

    with open(argv[2], "w") as handle:
        handle.write("".join(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
