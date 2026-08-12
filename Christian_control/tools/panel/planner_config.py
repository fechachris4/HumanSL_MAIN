"""The planner's tuning file: what the panel may write, and how.

Mirrors config_file.py for planner.yaml, with one honest difference the UI
must show: nothing here is compiled. A saved value applies at the NEXT
solve or session — no rebuild.

The whitelist covers every key in the file, deliberately: planner.yaml
already refuses unknown and missing keys with a hard error naming the key,
so the bridge remains the final authority on semantics. The panel's
validation (type + stated range) exists to fail earlier with a plain
sentence, never to replace the bridge's.
"""

import shutil
from pathlib import Path

from . import paths, yaml_text

# dotted key -> (type, rule, one-line meaning). Types: double|int|bool|vec3.
PLANNER_KNOBS: dict[str, tuple[str, str, str]] = {
    "motion.nominal_speed_mps": ("double", "positive",
        "Metres per second the plan is paced at — the speed knob"),
    "motion.min_duration_s": ("double", "nonnegative",
        "Floor on trajectory duration, s, so short moves are not abrupt"),
    "motion.waypoints": ("int", "min2",
        "Optimizer support states between start and goal"),
    "obstacles.epsilon_dist_m": ("double", "positive",
        "Metres from an obstacle at which its cost switches on"),
    "obstacles.collision_sigma": ("double", "positive",
        "Obstacle weight (gtsam sigma: SMALLER avoids harder)"),
    "smoothness.qc_scale": ("double", "positive",
        "GP prior scale: larger wanders freer off a straight line"),
    "goal.position_sigma_xyz": ("vec3", "positive",
        "Goal position stiffness per axis (sigma: smaller stiffer)"),
    "goal.rotation_sigma_rpy": ("vec3", "positive",
        "Goal orientation stiffness (sigma: smaller stiffer)"),
    "solver.max_iterations": ("int", "min1",
        "Levenberg-Marquardt iteration ceiling — convergence, not motion"),
    "path_following.position_prior_sigma_m": ("double", "positive",
        "Weight on each traced waypoint's position"),
    "path_following.rotation_prior_sigma_rad": ("double", "positive",
        "Weight on each traced waypoint's orientation"),
    "path_following.maximum_planning_error_m": ("double", "positive",
        "THE GATE: a plan straying further is not cleared for hardware"),
    "path_following.maximum_orientation_error_rad": ("double", "positive",
        "Orientation half of the gate"),
    "path_following.validation_dt_s": ("double", "positive",
        "Dense validation step, s (0.002 = the controller's own 500 Hz)"),
    "path_following.approach_velocity_fraction": ("double", "fraction",
        "Approach pacing as a fraction of joint velocity limits"),
    "path_following.approach_min_duration_s": ("double", "nonnegative",
        "Floor on the approach phase, s"),
    "path_following.approach_waypoints": ("int", "min1",
        "Support states in the approach phase"),
    "path_following.max_chord_error_m": ("double", "positive",
        "Circle sampling: max chord-to-arc error, m"),
    "seeding.ik_seed": ("int", "any",
        "IK restart seed — change to explore, keep to reproduce"),
    "seeding.randomised": ("bool", "any",
        "Draw a fresh (reported) seed at startup for robustness testing"),
}

_RULES = {
    "positive": (lambda v: v > 0, "must be greater than zero"),
    "nonnegative": (lambda v: v >= 0, "must be zero or more"),
    "fraction": (lambda v: 0 < v <= 1, "must be above 0 and at most 1"),
    "min1": (lambda v: v >= 1, "must be at least 1"),
    "min2": (lambda v: v >= 2, "must be at least 2"),
    "any": (lambda v: True, ""),
}


def read_planner_knobs(path: Path | None = None) -> dict[str, dict[str, object]]:
    """Every whitelisted knob with type, rule, meaning and current value.
    A key the file no longer holds comes back value=None rather than being
    dropped, so the panel shows the file changed shape."""
    target = path or paths.PLANNER_YAML
    text = target.read_text() if target.is_file() else ""
    out: dict[str, dict[str, object]] = {}
    for name, (ktype, rule, doc) in PLANNER_KNOBS.items():
        value = yaml_text.read_value(text, tuple(name.split(".")))
        out[name] = {"type": ktype, "rule": rule, "doc": doc, "value": value}
    return out


def _coerce(ktype: str, rule: str, value: object) -> tuple[bool, object]:
    """Validate a submitted value; return (ok, coerced-or-reason)."""
    check, why = _RULES[rule]
    if ktype == "bool":
        if isinstance(value, bool):
            return True, value
        if str(value).strip() in ("true", "false"):
            return True, str(value).strip() == "true"
        return False, "takes true or false"
    if ktype == "vec3":
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            return False, "takes three numbers"
        try:
            floats = [float(v) for v in value]
        except (TypeError, ValueError):
            return False, "every element must be a number"
        if not all(check(v) for v in floats):
            return False, f"every element {why}"
        return True, floats
    try:
        number = int(str(value).strip()) if ktype == "int" else float(str(value).strip())
    except ValueError:
        return False, "not an integer" if ktype == "int" else "not a number"
    if not check(number):
        return False, why
    return True, number


def write_planner_knob(name: str, value: object,
                       path: Path | None = None) -> tuple[bool, object]:
    """Rewrite one whitelisted planner.yaml value in place.
    A rejected value leaves the file untouched; the first successful write
    copies the file to planner.yaml.panel.bak."""
    if name not in PLANNER_KNOBS:
        return False, "unknown knob (not on the whitelist)"
    ktype, rule, _ = PLANNER_KNOBS[name]
    ok, coerced = _coerce(ktype, rule, value)
    if not ok:
        return False, f"{name}: {coerced}"
    target = path or paths.PLANNER_YAML
    if not target.is_file():
        return False, f"{target} does not exist"
    text = target.read_text()
    replaced = yaml_text.replace_value(
        text, tuple(name.split(".")), yaml_text.render(coerced))
    if replaced is None:
        return False, f"{name} not found in {target.name}"
    backup = paths.panel_backup(target)
    if not backup.exists():
        shutil.copy2(target, backup)
    target.write_text(replaced)
    return True, coerced
