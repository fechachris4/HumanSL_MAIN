"""The compiled settings in Config.h: what the panel may write, and what it may only read.

Four separate jobs live here because they carry four different permissions.

KNOBS is the write whitelist, and the whitelist IS the permission model: a
constant that is not named here cannot be read or written through the browser.
The guard overrides (GUARD_OVERRIDES) are on the whitelist and writable, by
Christian's explicit choice on 2026-08-12, reversing an earlier decision to
exclude them — see the "Superseded decisions" section of
docs/intent/story.md for the full record. Each is marked `dangerous` in
read_knobs() output so the UI can flag it distinctly; that flag is now the
only thing standing between a guard override and an ordinary gain, so treat
it as load-bearing, not decorative.

VECTOR_KNOBS is the write whitelist for per-joint arrays (JointVector
constants). Only the constant that actually holds the values is ever listed
here — never a re-exported alias — so a write always lands on the source
the controller reads, not a name that happens to equal it today.

read_thresholds returns the limits the run screen draws its bands from. The
panel never invents a round number for a band: every band comes from the
compiled constant that actually causes the consequence, so the screen cannot
disagree with the binary, and changing a knob moves the band.

read_joint_limits returns the per-joint ranges. Only joints 2, 4 and 6 are
bounded on a Gen3; the other four are continuous and Config.h stores a zero
sentinel for them. This module returns None rather than that zero, because a
zero would let the UI draw a degenerate 0-to-0 range that looks like a real
limit — and a limit drawn once tends to be believed later.
"""

import math
import re
import shutil
from pathlib import Path

from . import paths

# name -> (type, one-line meaning). Ported unchanged from tools/control_panel.py.
KNOBS: dict[str, tuple[str, str]] = {
    "kKpCartesian": ("double", "1/s on the Cartesian position error"),
    "kKpRotation": ("double", "1/s on the rotation-log error"),
    "kKdPosition": ("double", "Gain on the linear-velocity error (damping/feedforward)"),
    "kKdRotation": ("double", "Gain on the angular-velocity error"),
    "kDlsLambda": ("double", "DLS damping; larger = slower but safer near singularities"),
    "kOrientationEnabled": ("bool", "Track orientation as well as position"),
    "kVelocityTermEnabled": ("bool", "Kd term (twist error) on/off"),
    "kNullSpaceEnabled": ("bool", "Null-space joint-limit avoidance on/off"),
    "kLimitAvoidZoneDeg": ("double", "Zone before a software limit where avoidance wakes up"),
    "kLimitAvoidGain": ("double", "1/s push per degree of zone intrusion"),
    "kArrivalToleranceM": ("double", "Arrival position tolerance (m)"),
    "kArrivalOrientationToleranceRad": ("double", "Arrival orientation tolerance (rad)"),
    "kStopOnFault": ("bool", "DANGER: stop automatically on any live base or actuator fault"),
    "kAllowUnverifiedActuators": ("bool", "DANGER: accept takeover with an actuator whose limits could not be read back"),
    "kSkipStartupGates": ("bool", "DANGER: skip both startup gates; J2/J4/J6 limits revert to a degenerate 0/0 band"),
    "kDisableFollowingErrorStop": ("bool", "DANGER: disable the Cartesian following-error stop"),
}

# Each of these trades away an automatic stop. They are on the whitelist —
# writable — by Christian's explicit choice; this tuple now exists so the UI
# can flag them, not so write_knob can refuse them. See the KNOBS DANGER
# docs above and docs/intent/story.md for why each guard exists.
GUARD_OVERRIDES: tuple[str, ...] = (
    "kStopOnFault",
    "kAllowUnverifiedActuators",
    "kSkipStartupGates",
    "kDisableFollowingErrorStop",
)

# name -> (element type, one-line meaning). Each entry must name the constant
# that Config.h actually stores, never a re-exported alias (kQdotLimitDegS is
# `= kModelVelocityLimitsDegS`, not a separate value) — a write to an alias's
# name would silently change nothing the controller reads.
# Empty since 2026-08-20: kModelVelocityLimitsDegS used to be here, but it is
# now a derived alias (kVelocityControllerFraction of the physical hard limits
# in planning/config/joint_limits.yaml) rather than a literal array. Writing it
# would land on an alias, which this whitelist's own rule forbids. Speed is
# edited in that yaml instead, through the planner-config screen.
VECTOR_KNOBS: dict[str, tuple[str, str]] = {}

# Matches the value between `= {` and `};`, across lines, for one named
# JointVector. Anchored the same way KNOB_RE is: it can only ever land on the
# declaration's own braces, never spill into a neighbouring constant.
VECTOR_KNOB_RE = r"(inline\s+constexpr\s+JointVector\s+{name}\s*=\s*\{{)([^}}]*)(\}}\s*;)"

# The write regex is anchored to the scalar declaration so a rewrite can only
# ever land on `inline constexpr <double|bool|int> kName = <value>;`. It cannot
# match a JointVector, a comment or a static_assert.
KNOB_RE = r"(inline\s+constexpr\s+(?:double|bool|int)\s+{name}\s*=\s*)([^;]+)(;)"

# Read-only bands for the run screen: name -> (type, unit, the consequence of
# crossing it). The consequence text is what the bar is labelled with, so a
# threshold that stops nothing must not read like a stop.
THRESHOLDS: dict[str, tuple[str, str, str]] = {
    "kArrivalToleranceM": ("double", "m", "arrival"),
    "kHandoverToleranceM": ("double", "m", "handover gate — a new plan is rejected beyond this start mismatch"),
    "kArrivalOrientationToleranceRad": ("double", "rad", "arrival only — nothing stops the arm on orientation error"),
    "kFollowingErrorLimitDeg": ("double", "deg", "stop — commanded minus measured, any joint"),
    "kControlDtS": ("double", "s", "control period"),
}

_JOINT_COUNT = 7


def _config_text(path: Path | None) -> str:
    return (path or paths.CONFIG_H).read_text()


def parse_scalar(text: str, name: str) -> str | None:
    """Return the raw right-hand side of `inline constexpr <type> name = ...;`.

    Raw, not converted, because the caller knows which type it asked for and
    a failed conversion should be visible as a missing value rather than a
    coerced one.
    """
    pattern = (r"inline\s+constexpr\s+(?:double|bool|int|std::size_t)\s+"
               + re.escape(name) + r"\s*=\s*([^;]+);")
    match = re.search(pattern, text)
    return match.group(1).strip() if match else None


def parse_joint_vector(text: str, name: str, alias_depth: int = 1) -> list[float] | None:
    """Parse `inline constexpr JointVector name = { ... };` into seven floats.

    The braces may span several lines, so the value is captured up to the
    semicolon. One level of alias is resolved because Config.h defines
    kQdotLimitDegS as kModelVelocityLimitsDegS rather than repeating the
    numbers; anything deeper, or any entry that is not a plain number (the
    ternaries in kJointSoftwareLimitDeg, for instance), returns None so the
    caller computes the value itself instead of half-parsing C++.
    """
    pattern = (r"inline\s+constexpr\s+JointVector\s+" + re.escape(name)
               + r"\s*=\s*([^;]+);")
    match = re.search(pattern, text)
    if not match:
        return None
    body = re.sub(r"//[^\n]*", "", match.group(1)).strip()
    if not body.startswith("{"):
        if alias_depth > 0 and re.fullmatch(r"k\w+", body):
            return parse_joint_vector(text, body, alias_depth - 1)
        return None
    values: list[float] = []
    for token in body.strip("{} \t\r\n").split(","):
        token = token.strip()
        if not token:
            continue
        try:
            values.append(float(token))
        except ValueError:
            return None
    return values if len(values) == _JOINT_COUNT else None


def read_knobs(path: Path | None = None) -> dict[str, dict[str, object]]:
    """Every whitelisted knob with its type, its meaning and its current value.

    A knob the regex cannot find comes back with value None rather than being
    dropped, so the panel shows that Config.h changed shape instead of quietly
    showing fewer knobs than exist.
    """
    text = _config_text(path)
    out: dict[str, dict[str, object]] = {}
    for name, (ktype, doc) in KNOBS.items():
        match = re.search(KNOB_RE.format(name=re.escape(name)), text)
        out[name] = {
            "type": ktype,
            "doc": doc,
            "value": match.group(2).strip() if match else None,
            "dangerous": name in GUARD_OVERRIDES,
        }
    return out


def _render(ktype: str, value: object) -> tuple[bool, str]:
    """Validate a submitted value and render it as C++ source, or say why not."""
    # The browser posts JSON, so a bool arrives as a real bool. Check bool
    # before the string forms, since isinstance(True, int) is also true.
    if isinstance(value, bool):
        text = "true" if value else "false"
    else:
        text = str(value).strip()
    if ktype == "bool":
        if text not in ("true", "false"):
            return False, "bool knobs take true or false"
        return True, text
    if ktype == "int":
        try:
            return True, str(int(text))
        except ValueError:
            return False, "not an integer"
    try:
        return True, repr(float(text))
    except ValueError:
        return False, "not a number"


def write_knob(name: str, value: object, path: Path | None = None) -> tuple[bool, str]:
    """Rewrite one whitelisted constant in Config.h; return (ok, value or reason).

    A rejected value leaves the file untouched, and the first successful write
    of a session copies the file to Config.h.panel.bak so the settings the
    panel started from can always be recovered. Guard overrides are on the
    whitelist and writable like any other knob — see the module docstring for
    why — so nothing here treats them differently from a gain.
    """
    if name not in KNOBS:
        return False, "unknown knob (not on the whitelist)"
    ok, rendered = _render(KNOBS[name][0], value)
    if not ok:
        return False, rendered

    config_path = path or paths.CONFIG_H
    text = config_path.read_text()
    pattern = KNOB_RE.format(name=re.escape(name))
    if not re.search(pattern, text):
        return False, f"{name} not found in {config_path.name}"
    backup = (paths.CONFIG_H_BACKUP if config_path == paths.CONFIG_H
              else config_path.with_suffix(".h.panel.bak"))
    if not backup.exists():
        shutil.copy2(config_path, backup)
    config_path.write_text(re.sub(pattern, rf"\g<1>{rendered}\g<3>", text, count=1))
    return True, rendered


def read_vector_knobs(path: Path | None = None) -> dict[str, dict[str, object]]:
    """Every whitelisted per-joint array, with its type, meaning and 7 values.

    Like read_knobs, a name the regex cannot find comes back with value None
    rather than being dropped.
    """
    text = _config_text(path)
    out: dict[str, dict[str, object]] = {}
    for name, (ktype, doc) in VECTOR_KNOBS.items():
        out[name] = {
            "type": ktype,
            "doc": doc,
            "value": parse_joint_vector(text, name, alias_depth=0),
        }
    return out


def write_vector_knob(
    name: str, values: object, path: Path | None = None
) -> tuple[bool, str | list[float]]:
    """Rewrite one whitelisted JointVector in Config.h; return (ok, values or reason).

    Type-checked only, same standard as write_knob: exactly 7 numbers, each
    convertible to float. No range is enforced here — Christian's explicit
    choice, 2026-08-12 — so a value that is dynamically unsafe is still
    accepted; compilation and the controller's own runtime checks are the
    backstop, as they already are for every other knob.
    """
    if name not in VECTOR_KNOBS:
        return False, "unknown vector knob (not on the whitelist)"
    if not isinstance(values, (list, tuple)) or len(values) != _JOINT_COUNT:
        return False, f"expected {_JOINT_COUNT} numbers, one per joint"
    try:
        floats = [float(v) for v in values]
    except (TypeError, ValueError):
        return False, "every joint value must be a number"

    config_path = path or paths.CONFIG_H
    text = config_path.read_text()
    pattern = VECTOR_KNOB_RE.format(name=re.escape(name))
    if not re.search(pattern, text, re.DOTALL):
        return False, f"{name} not found in {config_path.name}"
    backup = (paths.CONFIG_H_BACKUP if config_path == paths.CONFIG_H
              else config_path.with_suffix(".h.panel.bak"))
    if not backup.exists():
        shutil.copy2(config_path, backup)
    rendered = ", ".join(repr(v) for v in floats)
    config_path.write_text(
        re.sub(pattern, rf"\g<1>{rendered}\g<3>", text, count=1, flags=re.DOTALL))
    return True, floats


def read_thresholds(path: Path | None = None) -> dict[str, dict[str, object]]:
    """The compiled limits the run screen draws its bands from, read-only."""
    text = _config_text(path)
    out: dict[str, dict[str, object]] = {}
    for name, (ktype, unit, consequence) in THRESHOLDS.items():
        raw = parse_scalar(text, name)
        value: object = None
        if raw is not None:
            try:
                value = int(raw) if ktype == "int" else float(raw)
            except ValueError:
                value = None
        out[name] = {"value": value, "unit": unit, "consequence": consequence}
    return out


def _physical_limits_and_margins():
    """(lower, upper, mask, controller_margin_deg, controller_velocity_deg_s).

    Read straight from joint_limits.yaml, the same file the generated header
    is built from, so the panel and the binary cannot disagree. On any failure
    every joint comes back unbounded, which is the safe direction to be wrong
    in: the panel draws nothing rather than drawing a guess.
    """
    try:
        import yaml

        with open(paths.JOINT_LIMITS_YAML) as handle:
            doc = yaml.safe_load(handle)
        margins = doc["margins"]
        margin = float(margins["position_controller_margin_deg"])
        fraction = float(margins["velocity_controller_fraction"])
        lower, upper, mask, velocity = [], [], [], []
        for i in range(1, _JOINT_COUNT + 1):
            entry = doc["position_limits"][f"actuator_{i}"]
            if entry.get("continuous", False):
                lower.append(0.0)
                upper.append(0.0)
                mask.append(0)
            else:
                lower.append(float(entry["lower_limit"]))
                upper.append(float(entry["upper_limit"]))
                mask.append(1)
            rad_s = float(doc["velocity_limits"][f"actuator_{i}"]["upper_limit"])
            velocity.append(math.degrees(rad_s) * fraction)
        return lower, upper, mask, margin, velocity
    except Exception:
        return None, None, None, None, None


def read_joint_limits(path: Path | None = None) -> dict[str, object]:
    """Per-joint ranges, with the continuous joints reported as unbounded.

    Since 2026-08-20 the physical limits and the margins are NOT in Config.h.
    They live in planning/config/joint_limits.yaml, which is the single
    authoritative table, and Config.h holds derived aliases that this module
    cannot text-parse. So the physical range, the bounded mask, the controller
    margin and the firmware warning (which IS the physical limit) all come
    from that yaml; only the firmware error threshold, still hand-authored,
    comes from Config.h.

    software_limit_deg is the physical upper magnitude less the controller
    margin, with no min() against the firmware warning: that cap is gone from
    Config.h and repeating it here would show the panel a number the binary no
    longer uses. Every array is seven long and carries None wherever a joint
    has no such value, so a caller that ignores bounded_mask still cannot draw
    a limit that does not exist.
    """
    text = _config_text(path)
    error = parse_joint_vector(text, "kJointLimitErrorDeg")

    lower, upper, mask, margin, velocity = _physical_limits_and_margins()
    # The firmware WARNING is the physical limit itself since 2026-08-20, so
    # it is no longer a literal in Config.h to parse. Only the ERROR threshold
    # is still hand-authored there.
    warn = upper

    bounded = [bool(v) for v in mask] if mask else [False] * _JOINT_COUNT
    out: dict[str, object] = {
        "bounded_mask": bounded,
        "margin_deg": margin,
        "lower_deg": [],
        "upper_deg": [],
        "warn_deg": [],
        "error_deg": [],
        "software_limit_deg": [],
        "velocity_limit_deg_s": velocity if velocity else [None] * _JOINT_COUNT,
    }
    for joint in range(_JOINT_COUNT):
        if not bounded[joint]:
            for key in ("lower_deg", "upper_deg", "warn_deg", "error_deg",
                        "software_limit_deg"):
                out[key].append(None)
            continue
        low = lower[joint] if lower else None
        high = upper[joint] if upper else None
        warn_deg = warn[joint] if warn else None
        error_deg = error[joint] if error else None
        software = None
        if high is not None and margin is not None:
            software = high - margin
        out["lower_deg"].append(low)
        out["upper_deg"].append(high)
        out["warn_deg"].append(warn_deg)
        out["error_deg"].append(error_deg)
        out["software_limit_deg"].append(software)
    return out
