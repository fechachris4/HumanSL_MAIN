# Planner Config and Goal Forms Panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `planner.yaml` and `joint_limits.yaml` editable from the browser panel, and replace the raw-YAML goal editor with structured per-arm forms (raw view kept, collapsed).

**Architecture:** A shared comment-aware YAML-subset helper (`yaml_text.py`) gives every panel file the same in-place edit rule the `Config.h` knobs already follow: replace single values in the file's own text, never regenerate, one-time `.panel.bak` backup. A new `planner_config.py` mirrors `config_file.py` for the two planner files; `plan.py` gains a structured per-arm goal writer; `server.py` gains three endpoints; the CONFIG and TARGETS tabs gain the UI. Spec: `Christian_control/docs/superpowers/specs/2026-08-12-planner-config-panel-design.md`.

**Tech Stack:** Python 3.12 stdlib only (no PyYAML), `unittest`, vanilla JS/HTML/CSS in `tools/panel/static/`.

## Global Constraints

- The panel never regenerates a YAML file — single values are replaced in place; all comments survive except an inline comment on the exact line whose value changed (it described the old value).
- A refused write leaves the file byte-for-byte unchanged and returns the reason as a sentence.
- First panel write to a file copies it to `<name>.panel.bak`, once.
- Every joint-limit field is `dangerous: True` in read output and danger-styled in the UI.
- The bridge's strict validation remains the final authority; the panel checks types and stated ranges only.
- No changes to what the bridge, controller, or session script read or validate.
- All tests hardware-free. Run from the repo root: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .` — 237 pass before this work; every task keeps the count green.
- Worktree note: the suite needs three symlinks to untracked fixtures (already created in this worktree): `runs/`, `Christian_control/planner_bridge/build`, `Christian_control/basic_control/build`, each pointing into `/home/christian/Desktop/HumanSL_MAIN/`. Never commit them (`git status` must not show them; they are untracked, just leave them).
- Do not run any robot-facing binary. Do not push.
- Commit after each task, on this worktree branch.

---

### Task 0: Stop pinning the tunable velocity limit in tests

Two tests assert the velocity limit equals 45 deg/s. That number is a knob the panel exists to change — Christian's browser edit to 50 today broke both tests in the main checkout. The tests' real meaning is "the alias reads back identical to the source constant", so assert that.

**Files:**
- Modify: `Christian_control/tools/panel/tests/test_config_file.py:139-142` and `:314-315`

**Interfaces:**
- Consumes: `config_file.parse_joint_vector(text, name, alias_depth)`, `config_file.read_vector_knobs()`, existing `JointLimits` fixture `self.limits`.
- Produces: nothing new — test-only change.

- [ ] **Step 1: Rewrite the two assertions to compare against the source constant**

Replace `test_the_real_constant_reads_back` (line 139):

```python
    def test_the_real_constant_reads_back(self):
        # Compare against the constant as written in Config.h, not a pinned
        # number: the velocity limit is a knob the panel is designed to edit,
        # so a legitimate edit must not break this test. What it guards is
        # that read_vector_knobs returns exactly what the file holds.
        source = config_file.parse_joint_vector(
            paths.CONFIG_H.read_text(), "kModelVelocityLimitsDegS",
            alias_depth=0)
        self.assertIsNotNone(source)
        knobs = config_file.read_vector_knobs()
        self.assertEqual(knobs["kModelVelocityLimitsDegS"]["value"], source)
```

Replace `test_velocity_limit_comes_through_the_alias` (line 314):

```python
    def test_velocity_limit_comes_through_the_alias(self):
        # The alias kQdotLimitDegS must resolve to the same seven numbers as
        # the source constant — whatever they currently are.
        source = config_file.parse_joint_vector(
            paths.CONFIG_H.read_text(), "kModelVelocityLimitsDegS",
            alias_depth=0)
        self.assertEqual(self.limits["velocity_limit_deg_s"], source)
```

- [ ] **Step 2: Run the suite**

Run: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .`
Expected: `Ran 237 tests ... OK (skipped=1)`

- [ ] **Step 3: Commit**

```bash
git add Christian_control/tools/panel/tests/test_config_file.py
git commit -m "tests: velocity-limit tests compare alias to source, never a pinned number"
```

---

### Task 1: Shared YAML-subset reader/editor (`yaml_text.py`)

**Files:**
- Create: `Christian_control/tools/panel/yaml_text.py`
- Create: `Christian_control/tools/panel/tests/test_yaml_text.py`
- Modify: `Christian_control/tools/panel/paths.py` (add `JOINT_LIMITS_YAML`, `panel_backup()`)
- Modify: `Christian_control/tools/panel/plan.py:80-109` (`_strip_comment`/`_scalar` delegate to the new module) and `:221-228` (`_backup_of` delegates to `paths.panel_backup`)

**Interfaces:**
- Consumes: nothing outside stdlib.
- Produces (used by Tasks 2, 3, 5):
  - `yaml_text.strip_comment(line: str) -> str`
  - `yaml_text.scalar(raw: str) -> Any` (float | str | list)
  - `yaml_text.render(value) -> str` (bool→`true/false`, list→`[ a, b, c ]`, float→`repr`, int/str→`str`)
  - `yaml_text.locate(lines: list[str], path: tuple[str, ...]) -> int | None`
  - `yaml_text.read_value(text: str, path: tuple[str, ...]) -> Any | None`
  - `yaml_text.replace_value(text: str, path, rendered: str) -> str | None` (None = key absent)
  - `yaml_text.insert_key(text: str, parent: tuple[str, ...], key: str, rendered: str) -> str | None`
  - `yaml_text.remove_key(text: str, path) -> str | None`
  - `paths.JOINT_LIMITS_YAML` (== `PLANNER_BRIDGE / "config" / "joint_limits.yaml"`)
  - `paths.panel_backup(target: Path) -> Path` (== `target.with_name(target.name + ".panel.bak")`)

- [ ] **Step 1: Write the failing tests**

Create `tests/test_yaml_text.py`. Test on a miniature file that has every feature the three real files use — nesting, inline lists, inline comments, comment lines, blank lines:

```python
"""Tests for yaml_text, the panel's comment-preserving YAML-subset editor.

Every edit test asserts two things: the edit landed, and every unrelated
byte survived. Comment preservation is the module's whole reason to exist.
"""

import unittest

from Christian_control.tools.panel import yaml_text

DOC = """\
# top comment
session_arms: left

motion:
  # how fast
  nominal_speed_mps: 0.05
  waypoints: 10

left:
  frame: mount
  path:
    type: circle
    centre: [ 0.31, 0.386, 0.5213 ]
  # trailing note about left
velocity_limits:
  actuator_1:
    lower_limit: -0.8727  # -50.0 deg/s
    unit: "rad/s"
"""


class Locate(unittest.TestCase):
    def test_finds_nested_keys(self):
        lines = DOC.split("\n")
        self.assertEqual(
            yaml_text.locate(lines, ("motion", "nominal_speed_mps")), 5)
        self.assertEqual(
            yaml_text.locate(lines, ("velocity_limits", "actuator_1",
                                     "lower_limit")), 16)

    def test_search_is_bounded_by_the_parent_block(self):
        # 'frame' exists only under left; asking for it under motion is None.
        lines = DOC.split("\n")
        self.assertIsNone(yaml_text.locate(lines, ("motion", "frame")))

    def test_absent_key_is_none(self):
        self.assertIsNone(yaml_text.locate(DOC.split("\n"), ("nowhere",)))


class ReadValue(unittest.TestCase):
    def test_scalars_lists_and_strings(self):
        self.assertEqual(
            yaml_text.read_value(DOC, ("motion", "nominal_speed_mps")), 0.05)
        self.assertEqual(
            yaml_text.read_value(DOC, ("left", "path", "centre")),
            [0.31, 0.386, 0.5213])
        self.assertEqual(yaml_text.read_value(DOC, ("left", "frame")), "mount")


class ReplaceValue(unittest.TestCase):
    def test_replaces_only_the_value_and_keeps_every_comment(self):
        out = yaml_text.replace_value(
            DOC, ("motion", "nominal_speed_mps"), "0.1")
        self.assertIn("nominal_speed_mps: 0.1", out)
        self.assertIn("# how fast", out)
        self.assertIn("# top comment", out)
        # Only that one line changed.
        diff = [(a, b) for a, b in zip(DOC.split("\n"), out.split("\n"))
                if a != b]
        self.assertEqual(len(diff), 1)

    def test_drops_the_stale_inline_comment_on_the_changed_line(self):
        out = yaml_text.replace_value(
            DOC, ("velocity_limits", "actuator_1", "lower_limit"), "-1.0")
        self.assertIn("lower_limit: -1.0", out)
        self.assertNotIn("-50.0 deg/s", out)
        self.assertIn('unit: "rad/s"', out)   # neighbour untouched

    def test_absent_key_returns_none_and_never_invents_one(self):
        self.assertIsNone(yaml_text.replace_value(DOC, ("motion", "zeta"), "1"))


class InsertKey(unittest.TestCase):
    def test_inserts_at_the_end_of_the_parent_block_with_its_indent(self):
        out = yaml_text.insert_key(DOC, ("motion",), "min_duration_s", "4.0")
        lines = out.split("\n")
        i = yaml_text.locate(lines, ("motion", "min_duration_s"))
        self.assertIsNotNone(i)
        self.assertTrue(lines[i].startswith("  min_duration_s:"))
        # It landed inside motion, before the left block begins.
        self.assertLess(i, yaml_text.locate(lines, ("left",)))

    def test_missing_parent_returns_none(self):
        self.assertIsNone(yaml_text.insert_key(DOC, ("ghost",), "k", "1"))


class RemoveKey(unittest.TestCase):
    def test_removes_a_nested_block_and_its_children_only(self):
        out = yaml_text.remove_key(DOC, ("left", "path"))
        self.assertNotIn("type: circle", out)
        self.assertNotIn("centre:", out)
        self.assertIn("frame: mount", out)
        self.assertIn("# trailing note about left", out)

    def test_removes_a_single_scalar_line(self):
        out = yaml_text.remove_key(DOC, ("left", "frame"))
        self.assertNotIn("frame: mount", out)
        self.assertIn("type: circle", out)

    def test_absent_key_returns_none(self):
        self.assertIsNone(yaml_text.remove_key(DOC, ("left", "box")))


class Render(unittest.TestCase):
    def test_each_type_renders_as_the_files_write_it(self):
        self.assertEqual(yaml_text.render(True), "true")
        self.assertEqual(yaml_text.render(False), "false")
        self.assertEqual(yaml_text.render(0.5), "0.5")
        self.assertEqual(yaml_text.render(10), "10")
        self.assertEqual(yaml_text.render([1.0, 2.0, 3.0]), "[ 1.0, 2.0, 3.0 ]")
        self.assertEqual(yaml_text.render("mount"), "mount")
```

(Line numbers in `Locate` count from 0 over `DOC.split("\n")` — verify them against the literal when writing the test; adjust if the docstring layout shifts them.)

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest Christian_control.tools.panel.tests.test_yaml_text`
Expected: FAIL — `No module named 'Christian_control.tools.panel.yaml_text'`

- [ ] **Step 3: Implement `yaml_text.py`**

```python
"""In-place reading and editing of the panel's YAML files.

goal.yaml, planner.yaml and joint_limits.yaml are hand-written, heavily
commented files, and the comments are documentation Christian relies on. So
the panel never regenerates a file from parsed structure — it edits single
values in the file's own text, and everything it did not touch survives
byte-for-byte. This module is the one place that rule is implemented.

The subset understood here is exactly what those three files use: two-space
indents, `key: value` lines, inline `[a, b, c]` lists, `#` comments. Key
names are assumed unique within the block being searched (true of all three
files). No PyYAML — the panel is stdlib-only.

One deliberate loss: an inline comment on the exact line whose value the
panel changed is dropped, because it described the old value (joint_limits
annotates radians with degrees) and keeping it would make the file lie.
Comments on their own lines always survive.
"""

import re
from typing import Any

_KEY_RE = re.compile(r"^(\s*)([A-Za-z_]\w*):(.*)$")
_VALUE_LINE_RE = re.compile(r"^(\s*[A-Za-z_]\w*:\s*)([^#]*?)(\s*#.*)?$")


def strip_comment(line: str) -> str:
    """Remove a YAML comment. '#' only opens one at line start or after
    whitespace, so a '#' inside a quoted value is left alone."""
    if line.lstrip().startswith("#"):
        return ""
    for marker in (" #", "\t#"):
        cut = line.find(marker)
        if cut >= 0:
            line = line[:cut]
    return line


def scalar(raw: str) -> Any:
    """A value as the panel wants it: numbers as floats, [..] as lists,
    anything else as the string it is."""
    text = raw.strip()
    if text.startswith("[") and text.endswith("]"):
        return [scalar(part) for part in text[1:-1].split(",") if part.strip()]
    try:
        return float(text)
    except ValueError:
        return text


def render(value: Any) -> str:
    """A Python value as these files write it."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return "[ " + ", ".join(render(v) for v in value) + " ]"
    if isinstance(value, float):
        return repr(value)
    return str(value)


def _key_match(line: str) -> re.Match | None:
    stripped = strip_comment(line)
    return _KEY_RE.match(stripped) if stripped.strip() else None


def _block_end(lines: list[str], start: int, indent: int) -> int:
    """Index of the first key line at `indent` or shallower — the line that
    closes the block whose children are deeper than `indent`."""
    for i in range(start, len(lines)):
        m = _key_match(lines[i])
        if m and len(m.group(1)) <= indent:
            return i
    return len(lines)


def locate(lines: list[str], path: tuple[str, ...]) -> int | None:
    """Line index of the key `path` names, or None. Each step searches only
    inside the previous key's block."""
    start, parent_indent = 0, -1
    index: int | None = None
    for name in path:
        end = (len(lines) if parent_indent < 0
               else _block_end(lines, start, parent_indent))
        index = None
        for i in range(start, end):
            m = _key_match(lines[i])
            if m and len(m.group(1)) > parent_indent and m.group(2) == name:
                index, parent_indent, start = i, len(m.group(1)), i + 1
                break
        if index is None:
            return None
    return index


def read_value(text: str, path: tuple[str, ...]) -> Any | None:
    lines = text.split("\n")
    i = locate(lines, path)
    if i is None:
        return None
    m = _KEY_RE.match(strip_comment(lines[i]))
    value = m.group(3).strip()
    return scalar(value) if value else None


def replace_value(text: str, path: tuple[str, ...],
                  rendered: str) -> str | None:
    lines = text.split("\n")
    i = locate(lines, path)
    if i is None:
        return None
    m = _VALUE_LINE_RE.match(lines[i])
    lines[i] = m.group(1) + rendered
    return "\n".join(lines)


def insert_key(text: str, parent: tuple[str, ...], key: str,
               rendered: str) -> str | None:
    """Append `key: rendered` at the end of `parent`'s block (top level when
    parent is empty), before any trailing blank or comment lines."""
    lines = text.split("\n")
    if parent:
        p = locate(lines, parent)
        if p is None:
            return None
        parent_indent = len(_key_match(lines[p]).group(1))
        end = _block_end(lines, p + 1, parent_indent)
        indent = parent_indent + 2
    else:
        end, indent = len(lines), 0
    while end > 0 and _key_match(lines[end - 1]) is None:
        end -= 1
    lines.insert(end, (" " * indent + f"{key}: {rendered}").rstrip())
    return "\n".join(lines)


def remove_key(text: str, path: tuple[str, ...]) -> str | None:
    """Delete the key's line and every deeper-indented line under it. Comment
    lines after its last child stay — the panel cannot know whose prose they
    are, and keeping a comment is the safe direction."""
    lines = text.split("\n")
    i = locate(lines, path)
    if i is None:
        return None
    indent = len(_key_match(lines[i]).group(1))
    last = i
    for j in range(i + 1, len(lines)):
        m = _key_match(lines[j])
        if m is None:
            continue
        if len(m.group(1)) <= indent:
            break
        last = j
    del lines[i:last + 1]
    return "\n".join(lines)
```

- [ ] **Step 4: Add the two `paths.py` entries**

After `PLANNER_YAML` (line 27):

```python
JOINT_LIMITS_YAML = PLANNER_BRIDGE / "config" / "joint_limits.yaml"
```

At the bottom, beside `target_pipe`:

```python
def panel_backup(target: Path) -> Path:
    """file.yaml -> file.yaml.panel.bak, beside the original. Written once,
    before the panel's first edit, so it holds the pre-panel state."""
    return target.with_name(target.name + ".panel.bak")
```

- [ ] **Step 5: Point `plan.py` at the shared module**

In `plan.py`: delete the bodies of `_strip_comment` and `_scalar` (lines 80–109) and replace with imports — at the top add `from . import yaml_text`, then:

```python
_strip_comment = yaml_text.strip_comment
_scalar = yaml_text.scalar
```

Replace `_backup_of` (lines 221–228) body with:

```python
def _backup_of(target: Path) -> Path:
    return paths.panel_backup(target)
```

- [ ] **Step 6: Run the full suite**

Run: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .`
Expected: OK — the old 237 plus the new yaml_text tests.

- [ ] **Step 7: Commit**

```bash
git add Christian_control/tools/panel/yaml_text.py \
        Christian_control/tools/panel/tests/test_yaml_text.py \
        Christian_control/tools/panel/paths.py \
        Christian_control/tools/panel/plan.py
git commit -m "panel: one shared comment-preserving editor for the YAML files"
```

---

### Task 2: `planner_config.py` — the planner.yaml knobs

**Files:**
- Create: `Christian_control/tools/panel/planner_config.py`
- Create: `Christian_control/tools/panel/tests/test_planner_config.py`

**Interfaces:**
- Consumes: `yaml_text` and `paths` from Task 1.
- Produces (used by Tasks 4, 6):
  - `planner_config.PLANNER_KNOBS: dict[str, tuple[str, str, str]]` — dotted name → (type, rule, meaning); types `double|int|bool|vec3`, rules `positive|nonnegative|fraction|min1|min2|any`
  - `planner_config.read_planner_knobs(path: Path | None = None) -> dict[str, dict]` — name → `{"type", "rule", "doc", "value"}` (value None when missing)
  - `planner_config.write_planner_knob(name: str, value: object, path: Path | None = None) -> tuple[bool, object]` — (ok, coerced value or reason sentence)

- [ ] **Step 1: Write the failing tests**

```python
"""Tests for planner_config's planner.yaml knobs.

Same rule as test_config_file: the real planner.yaml is read but never
written; write tests work on a copy in a temporary directory.
"""

import shutil
import tempfile
import unittest
from pathlib import Path

from Christian_control.tools.panel import paths, planner_config


class Whitelist(unittest.TestCase):
    def test_every_knob_is_found_in_the_real_file(self):
        knobs = planner_config.read_planner_knobs()
        self.assertEqual(len(knobs), 21)
        missing = [n for n, k in knobs.items() if k["value"] is None]
        self.assertEqual(missing, [], "knob(s) no longer match planner.yaml")

    def test_known_values_read_back(self):
        knobs = planner_config.read_planner_knobs()
        self.assertEqual(knobs["motion.nominal_speed_mps"]["value"], 0.05)
        self.assertEqual(knobs["motion.waypoints"]["value"], 10.0)
        self.assertEqual(knobs["seeding.randomised"]["value"], "false")
        self.assertEqual(
            knobs["goal.position_sigma_xyz"]["value"], [0.01, 0.1, 0.01])

    def test_every_real_key_is_whitelisted(self):
        # planner.yaml refuses unknown keys, so the whitelist and the file
        # must cover each other exactly: a key in the file but not here would
        # be invisible in the browser, which is how "I changed it and nothing
        # happened" starts.
        text = paths.PLANNER_YAML.read_text()
        from Christian_control.tools.panel import yaml_text
        for dotted in planner_config.PLANNER_KNOBS:
            self.assertIsNotNone(
                yaml_text.read_value(text, tuple(dotted.split("."))), dotted)


class WriteAgainstACopy(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_planner_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.yaml = self.tmp / "planner.yaml"
        shutil.copy2(paths.PLANNER_YAML, self.yaml)
        self.original = self.yaml.read_text()
        self.backup = paths.panel_backup(self.yaml)

    def assert_unchanged(self):
        self.assertEqual(self.yaml.read_text(), self.original)
        self.assertFalse(self.backup.exists())

    def test_round_trip_keeps_every_comment(self):
        ok, value = planner_config.write_planner_knob(
            "motion.nominal_speed_mps", "0.1", self.yaml)
        self.assertTrue(ok)
        self.assertEqual(value, 0.1)
        text = self.yaml.read_text()
        self.assertEqual(
            planner_config.read_planner_knobs(self.yaml)[
                "motion.nominal_speed_mps"]["value"], 0.1)
        # Comment lines are untouched: same count, same content.
        original_comments = [l for l in self.original.split("\n")
                             if l.lstrip().startswith("#")]
        new_comments = [l for l in text.split("\n")
                        if l.lstrip().startswith("#")]
        self.assertEqual(new_comments, original_comments)

    def test_vector_write(self):
        ok, value = planner_config.write_planner_knob(
            "goal.position_sigma_xyz", [0.02, 0.02, 0.02], self.yaml)
        self.assertTrue(ok)
        self.assertEqual(
            planner_config.read_planner_knobs(self.yaml)[
                "goal.position_sigma_xyz"]["value"], [0.02, 0.02, 0.02])

    def test_bool_and_int(self):
        ok, _ = planner_config.write_planner_knob(
            "seeding.randomised", True, self.yaml)
        self.assertTrue(ok)
        ok, _ = planner_config.write_planner_knob(
            "solver.max_iterations", "500", self.yaml)
        self.assertTrue(ok)
        knobs = planner_config.read_planner_knobs(self.yaml)
        self.assertEqual(knobs["seeding.randomised"]["value"], "true")
        self.assertEqual(knobs["solver.max_iterations"]["value"], 500.0)

    def test_backup_written_once_holding_the_original(self):
        planner_config.write_planner_knob(
            "motion.nominal_speed_mps", "0.1", self.yaml)
        planner_config.write_planner_knob(
            "motion.nominal_speed_mps", "0.2", self.yaml)
        self.assertEqual(self.backup.read_text(), self.original)

    def test_unknown_key_refused(self):
        ok, why = planner_config.write_planner_knob("motion.zeta", "1", self.yaml)
        self.assertFalse(ok)
        self.assertIn("whitelist", why)
        self.assert_unchanged()

    def test_rule_violations_refused_with_a_sentence(self):
        for name, bad in [("motion.nominal_speed_mps", "0"),
                          ("motion.nominal_speed_mps", "-1"),
                          ("motion.waypoints", "1"),
                          ("path_following.approach_velocity_fraction", "1.5"),
                          ("solver.max_iterations", "0"),
                          ("motion.nominal_speed_mps", "fast"),
                          ("goal.position_sigma_xyz", [0.01, 0.01]),
                          ("goal.position_sigma_xyz", [0.01, -0.01, 0.01])]:
            ok, why = planner_config.write_planner_knob(name, bad, self.yaml)
            self.assertFalse(ok, f"{name}={bad!r} should be refused")
            self.assertTrue(why, name)
        self.assert_unchanged()
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest Christian_control.tools.panel.tests.test_planner_config`
Expected: FAIL — no module `planner_config`.

- [ ] **Step 3: Implement**

```python
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
```

Note: `read_value` returns floats for numbers and the strings `"true"`/`"false"` for bools — the tests above pin that so the UI knows what arrives.

- [ ] **Step 4: Run the new tests, then the full suite**

Run: `python3 -m unittest Christian_control.tools.panel.tests.test_planner_config`
Expected: PASS.
Run: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .`
Expected: OK.

- [ ] **Step 5: Commit**

```bash
git add Christian_control/tools/panel/planner_config.py \
        Christian_control/tools/panel/tests/test_planner_config.py
git commit -m "panel: planner.yaml tuning becomes editable through a whitelist"
```

---

### Task 3: `planner_config.py` — the joint-limit tables

**Files:**
- Modify: `Christian_control/tools/panel/planner_config.py`
- Modify: `Christian_control/tools/panel/tests/test_planner_config.py`

**Interfaces:**
- Consumes: Task 1's `yaml_text`, `paths.JOINT_LIMITS_YAML`, `paths.panel_backup`.
- Produces (used by Tasks 4, 6):
  - `planner_config.LIMIT_SECTIONS == ("position_limits", "velocity_limits", "acceleration_limits")`
  - `planner_config.ACTUATORS == ("actuator_1", ..., "actuator_7")`
  - `planner_config.read_joint_limits_file(path=None) -> dict` — `{section: {actuator: {"lower_limit": float|None, "upper_limit": float|None, "unit": str|None, "dangerous": True}}}`
  - `planner_config.write_joint_limit(section, actuator, bound, value, path=None) -> tuple[bool, object]`

- [ ] **Step 1: Write the failing tests (append to `test_planner_config.py`)**

```python
class JointLimitsRead(unittest.TestCase):
    def test_all_sections_actuators_and_bounds_are_present(self):
        table = planner_config.read_joint_limits_file()
        for section in planner_config.LIMIT_SECTIONS:
            for actuator in planner_config.ACTUATORS:
                entry = table[section][actuator]
                self.assertIsNotNone(entry["lower_limit"], (section, actuator))
                self.assertIsNotNone(entry["upper_limit"], (section, actuator))
                self.assertTrue(entry["dangerous"], (section, actuator))

    def test_known_values(self):
        table = planner_config.read_joint_limits_file()
        self.assertAlmostEqual(
            table["position_limits"]["actuator_2"]["upper_limit"], 2.2515)
        self.assertAlmostEqual(
            table["velocity_limits"]["actuator_1"]["lower_limit"], -0.8727)
        self.assertEqual(
            table["position_limits"]["actuator_2"]["unit"], '"radians"')


class JointLimitsWrite(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_jl_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.yaml = self.tmp / "joint_limits.yaml"
        shutil.copy2(paths.JOINT_LIMITS_YAML, self.yaml)
        self.original = self.yaml.read_text()

    def test_round_trip_touches_one_section_only(self):
        ok, value = planner_config.write_joint_limit(
            "velocity_limits", "actuator_3", "upper_limit", "1.0", self.yaml)
        self.assertTrue(ok)
        self.assertEqual(value, 1.0)
        table = planner_config.read_joint_limits_file(self.yaml)
        self.assertEqual(
            table["velocity_limits"]["actuator_3"]["upper_limit"], 1.0)
        # The same actuator's POSITION limit is untouched — the sections
        # repeat actuator names, so this is the aliasing bug to guard.
        self.assertAlmostEqual(
            table["position_limits"]["actuator_3"]["lower_limit"], -1e20)

    def test_lower_must_stay_below_upper(self):
        ok, why = planner_config.write_joint_limit(
            "velocity_limits", "actuator_3", "lower_limit", "2.0", self.yaml)
        self.assertFalse(ok)
        self.assertIn("upper", why)
        self.assertEqual(self.yaml.read_text(), self.original)

    def test_bad_names_and_values_refused(self):
        for args in [("speed_limits", "actuator_1", "lower_limit", "1"),
                     ("velocity_limits", "actuator_9", "lower_limit", "1"),
                     ("velocity_limits", "actuator_1", "unit", "x"),
                     ("velocity_limits", "actuator_1", "lower_limit", "slow")]:
            ok, _ = planner_config.write_joint_limit(*args, self.yaml)
            self.assertFalse(ok, args)
        self.assertEqual(self.yaml.read_text(), self.original)
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest Christian_control.tools.panel.tests.test_planner_config`
Expected: FAIL — no attribute `LIMIT_SECTIONS`.

- [ ] **Step 3: Implement (append to `planner_config.py`)**

```python
# Every joint-limit field is dangerous by construction: these are Kinova's
# official table values and they feed the planner's dynamics validation, so
# a wrong number weakens a real check. Editable by Christian's explicit
# choice (2026-08-12); the flag is what the UI styles the warning from.
LIMIT_SECTIONS = ("position_limits", "velocity_limits", "acceleration_limits")
ACTUATORS = tuple(f"actuator_{i}" for i in range(1, 8))
_BOUNDS = ("lower_limit", "upper_limit")


def read_joint_limits_file(path: Path | None = None) -> dict[str, dict]:
    target = path or paths.JOINT_LIMITS_YAML
    text = target.read_text() if target.is_file() else ""
    out: dict[str, dict] = {}
    for section in LIMIT_SECTIONS:
        out[section] = {}
        for actuator in ACTUATORS:
            entry: dict[str, object] = {"dangerous": True}
            for field in (*_BOUNDS, "unit"):
                entry[field] = yaml_text.read_value(
                    text, (section, actuator, field))
            out[section][actuator] = entry
    return out


def write_joint_limit(section: str, actuator: str, bound: str, value: object,
                      path: Path | None = None) -> tuple[bool, object]:
    """Rewrite one bound in joint_limits.yaml, keeping lower < upper."""
    if section not in LIMIT_SECTIONS:
        return False, f"unknown section {section!r}"
    if actuator not in ACTUATORS:
        return False, f"unknown actuator {actuator!r}"
    if bound not in _BOUNDS:
        return False, "only lower_limit and upper_limit are writable"
    try:
        number = float(str(value).strip())
    except ValueError:
        return False, "not a number"
    target = path or paths.JOINT_LIMITS_YAML
    if not target.is_file():
        return False, f"{target} does not exist"
    text = target.read_text()
    other_name = "upper_limit" if bound == "lower_limit" else "lower_limit"
    other = yaml_text.read_value(text, (section, actuator, other_name))
    if isinstance(other, float):
        lower, upper = ((number, other) if bound == "lower_limit"
                        else (other, number))
        if lower >= upper:
            return False, (f"{section}/{actuator}: lower_limit must stay "
                           f"below upper_limit ({lower} >= {upper})")
    replaced = yaml_text.replace_value(
        text, (section, actuator, bound), yaml_text.render(number))
    if replaced is None:
        return False, f"{section}/{actuator}/{bound} not found in {target.name}"
    backup = paths.panel_backup(target)
    if not backup.exists():
        shutil.copy2(target, backup)
    target.write_text(replaced)
    return True, number
```

- [ ] **Step 4: Run the module tests, then the full suite**

Run: `python3 -m unittest Christian_control.tools.panel.tests.test_planner_config`
Expected: PASS. Then full discovery: OK.

- [ ] **Step 5: Commit**

```bash
git add Christian_control/tools/panel/planner_config.py \
        Christian_control/tools/panel/tests/test_planner_config.py
git commit -m "panel: joint_limits.yaml editable, every field flagged dangerous"
```

---

### Task 4: Server endpoints for the planner files

**Files:**
- Modify: `Christian_control/tools/panel/server.py` (import + one GET route ~line 220, one POST route ~line 280)
- Modify: `Christian_control/tools/panel/tests/test_static.py` (endpoint presence)

**Interfaces:**
- Consumes: Task 2/3's `planner_config` functions.
- Produces (used by Tasks 6, 7's JS):
  - `GET /api/planner-config` → `{"planner": read_planner_knobs(), "joint_limits": read_joint_limits_file()}`
  - `POST /api/planner-config/set` with body `{"file": "planner", "name", "value"}` or `{"file": "joint_limits", "section", "actuator", "bound", "value"}` → `{"ok": true, "value": ...}` or `{"error": "..."}` (HTTP 400)

- [ ] **Step 1: Add the routes**

Import `planner_config` beside the other module imports at the top of `server.py`. In `do_GET`, after the `/api/config` branch:

```python
            elif route == "/api/planner-config":
                self._json({
                    "planner": planner_config.read_planner_knobs(),
                    "joint_limits": planner_config.read_joint_limits_file(),
                })
```

In `do_POST`, after the `/api/config/set` branch:

```python
            elif route == "/api/planner-config/set":
                req = self._body()
                if req.get("file") == "joint_limits":
                    ok, message = planner_config.write_joint_limit(
                        str(req.get("section", "")),
                        str(req.get("actuator", "")),
                        str(req.get("bound", "")),
                        req.get("value"))
                else:
                    ok, message = planner_config.write_planner_knob(
                        str(req.get("name", "")), req.get("value"))
                self._json({"ok": True, "value": message} if ok
                           else {"error": message}, 200 if ok else 400)
```

- [ ] **Step 2: Pin the endpoints in `test_static.py`**

Append:

```python
class PlannerEndpoints(unittest.TestCase):
    def test_planner_config_endpoints_exist_in_server(self):
        self.assertIn('"/api/planner-config"', SERVER)
        self.assertIn('"/api/planner-config/set"', SERVER)
```

- [ ] **Step 3: Run the full suite**

Run: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .`
Expected: OK.

- [ ] **Step 4: Commit**

```bash
git add Christian_control/tools/panel/server.py \
        Christian_control/tools/panel/tests/test_static.py
git commit -m "panel: /api/planner-config read and set endpoints"
```

---

### Task 5: Structured per-arm goal writes

**Files:**
- Modify: `Christian_control/tools/panel/plan.py` (new `write_goal_fields` after `write_goal`, ~line 248)
- Modify: `Christian_control/tools/panel/server.py` (POST `/api/goal/fields`)
- Modify: `Christian_control/tools/panel/tests/test_plan.py`
- Modify: `Christian_control/tools/panel/tests/test_static.py` (endpoint presence)

**Interfaces:**
- Consumes: `yaml_text`, existing `validate_goal`, `_backup_of`, `paths.GOAL_YAML`, `paths.ARMS`.
- Produces (used by Task 7's JS):
  - `plan.write_goal_fields(arm: str, fields: dict, path: Path | None = None) -> tuple[bool, str]`
  - `POST /api/goal/fields` body `{"arm": "right"|"left", "fields": {...}}` → `{"ok": true}` or `{"error": "..."}` (400)

`fields` keys, all optional except `mode`:
- `"mode"`: `"point"` or `"circle"` — required, decides which of `goal`/`path` survives
- `"frame"`: `"mount" | "right_base" | "left_base"`
- `"goal"`: `[x, y, z]` (point mode)
- `"orientation_rpy_deg"`: `[r, p, y]` or `None` (None = inherit at start: the key is removed)
- `"box"`: `{"center": [3], "half_extent": [3]}` or `None` (None = remove)
- `"path"`: `{"type": "circle", "centre": [3], "radius_m", "normal": [3], "duration_s", "orientation": "fixed"|"radial", "orientation_rpy_deg": [3]}` (circle mode)

- [ ] **Step 1: Write the failing tests (append to `test_plan.py`, following its existing tmp-copy pattern)**

```python
GOAL_DOC = """\
session_arms: left

right:
  # right's comment survives every edit
  frame: mount
  goal: [ 0.5, 0.0, 0.4 ]

left:
  frame: mount
  path:
    type: circle
    centre: [ 0.31, 0.386, 0.5213 ]
    radius_m: 0.1
    normal: [ 1, 0, 0 ]
    duration_s: 12.0
    orientation: fixed
    orientation_rpy_deg: [ 90, 0, 90 ]
"""


class WriteGoalFields(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="panel_goal_"))
        self.addCleanup(shutil.rmtree, self.tmp)
        self.yaml = self.tmp / "goal.yaml"
        self.yaml.write_text(GOAL_DOC)

    def read(self):
        return plan.parse_goal(self.yaml.read_text())

    def test_point_edit_changes_values_in_place(self):
        ok, why = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.4, 0.1, 0.3],
                      "frame": "mount"}, self.yaml)
        self.assertTrue(ok, why)
        self.assertEqual(self.read()["arms"]["right"]["goal"], [0.4, 0.1, 0.3])
        self.assertIn("# right's comment survives every edit",
                      self.yaml.read_text())

    def test_orientation_none_means_inherit_and_removes_the_key(self):
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "orientation_rpy_deg": [10.0, 20.0, 30.0]}, self.yaml)
        self.assertTrue(ok)
        self.assertEqual(self.read()["arms"]["right"]["orientation_rpy_deg"],
                         [10.0, 20.0, 30.0])
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "orientation_rpy_deg": None}, self.yaml)
        self.assertTrue(ok)
        self.assertNotIn("orientation_rpy_deg", self.read()["arms"]["right"])

    def test_switching_left_to_point_removes_the_path_block(self):
        ok, why = plan.write_goal_fields(
            "left", {"mode": "point", "goal": [0.3, 0.4, 0.5]}, self.yaml)
        self.assertTrue(ok, why)
        left = self.read()["arms"]["left"]
        self.assertEqual(left["goal"], [0.3, 0.4, 0.5])
        self.assertNotIn("path", left)

    def test_switching_right_to_circle_removes_goal_and_builds_path(self):
        circle = {"type": "circle", "centre": [0.3, 0.3, 0.5],
                  "radius_m": 0.05, "normal": [0.0, 0.0, 1.0],
                  "duration_s": 10.0, "orientation": "fixed",
                  "orientation_rpy_deg": [90.0, 0.0, 90.0]}
        ok, why = plan.write_goal_fields(
            "right", {"mode": "circle", "path": circle}, self.yaml)
        self.assertTrue(ok, why)
        right = self.read()["arms"]["right"]
        self.assertNotIn("goal", right)
        self.assertEqual(right["path"]["centre"], [0.3, 0.3, 0.5])
        self.assertEqual(right["path"]["orientation"], "fixed")

    def test_box_add_and_remove(self):
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "box": {"center": [0.5, 0.0, 0.3],
                              "half_extent": [0.05, 0.05, 0.05]}}, self.yaml)
        self.assertTrue(ok)
        self.assertEqual(self.read()["arms"]["right"]["box"]["center"],
                         [0.5, 0.0, 0.3])
        ok, _ = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, 0.0, 0.4],
                      "box": None}, self.yaml)
        self.assertTrue(ok)
        self.assertNotIn("box", self.read()["arms"]["right"])

    def test_a_bad_edit_leaves_the_file_byte_identical(self):
        before = self.yaml.read_text()
        ok, why = plan.write_goal_fields(
            "right", {"mode": "point", "goal": [0.5, "x", 0.4]}, self.yaml)
        self.assertFalse(ok)
        self.assertTrue(why)
        self.assertEqual(self.yaml.read_text(), before)
        ok, _ = plan.write_goal_fields("middle", {"mode": "point"}, self.yaml)
        self.assertFalse(ok)
        self.assertEqual(self.yaml.read_text(), before)
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest Christian_control.tools.panel.tests.test_plan`
Expected: FAIL — no attribute `write_goal_fields`.

- [ ] **Step 3: Implement `write_goal_fields` in `plan.py`**

```python
# The order path keys are (re)built in, matching the file's own layout.
_PATH_KEYS = ("type", "centre", "radius_m", "normal", "duration_s",
              "orientation", "orientation_rpy_deg")
_BOX_KEYS = ("center", "half_extent")


def _set_key(text: str, path_t: tuple[str, ...], value: Any) -> str | None:
    """Replace the key's value, or insert the key at its parent's block end.
    Insertion is how a commented-out example key (invisible to the parser)
    becomes real."""
    rendered = yaml_text.render(value)
    replaced = yaml_text.replace_value(text, path_t, rendered)
    if replaced is not None:
        return replaced
    return yaml_text.insert_key(text, path_t[:-1], path_t[-1], rendered)


def _set_block(text: str, parent: tuple[str, ...], key: str, keys: tuple,
               values: dict) -> str | None:
    """Ensure `parent.key:` exists, then set each of `keys` present in
    `values` under it, in order."""
    if yaml_text.locate(text.split("\n"), parent + (key,)) is None:
        text = yaml_text.insert_key(text, parent, key, "")
        if text is None:
            return None
    for k in keys:
        if k in values and values[k] is not None:
            text = _set_key(text, parent + (key, k), values[k])
            if text is None:
                return None
    return text


def write_goal_fields(arm: str, fields: dict,
                      path: Path | None = None) -> tuple[bool, str]:
    """Edit one arm's block of the goal file in place from structured fields.

    Applies the edits to the text, then runs the SAME validate_goal gate the
    raw editor goes through — one gate, not two opinions — and only a text
    that passes replaces the file. mode decides which of goal/path survives,
    so the bridge's "mutually exclusive" refusal is unreachable from here.
    """
    if arm not in paths.ARMS:
        return False, f"unknown arm {arm!r}"
    mode = fields.get("mode")
    if mode not in ("point", "circle"):
        return False, "mode must be point or circle"
    target = path or paths.GOAL_YAML
    if not target.is_file():
        return False, f"{target} does not exist"
    text = target.read_text()
    if yaml_text.locate(text.split("\n"), (arm,)) is None:
        return False, f"{target.name} has no {arm}: block"

    def step(new_text: str | None, what: str) -> str:
        if new_text is None:
            raise ValueError(f"could not {what}")
        return new_text

    try:
        if mode == "point":
            if yaml_text.locate(text.split("\n"), (arm, "path")) is not None:
                text = step(yaml_text.remove_key(text, (arm, "path")),
                            "remove the path block")
            if fields.get("goal") is not None:
                text = step(_set_key(text, (arm, "goal"), fields["goal"]),
                            "set the goal")
        else:
            if yaml_text.locate(text.split("\n"), (arm, "goal")) is not None:
                text = step(yaml_text.remove_key(text, (arm, "goal")),
                            "remove the goal key")
            path_fields = fields.get("path") or {}
            text = step(_set_block(text, (arm,), "path", _PATH_KEYS,
                                   path_fields), "build the path block")

        if "frame" in fields and fields["frame"] is not None:
            text = step(_set_key(text, (arm, "frame"), fields["frame"]),
                        "set the frame")

        if "orientation_rpy_deg" in fields:
            rpy = fields["orientation_rpy_deg"]
            if rpy is None:
                if yaml_text.locate(text.split("\n"),
                                    (arm, "orientation_rpy_deg")) is not None:
                    text = step(yaml_text.remove_key(
                        text, (arm, "orientation_rpy_deg")),
                        "remove the orientation")
            else:
                text = step(_set_key(text, (arm, "orientation_rpy_deg"), rpy),
                            "set the orientation")

        if "box" in fields:
            box = fields["box"]
            if box is None:
                if yaml_text.locate(text.split("\n"), (arm, "box")) is not None:
                    text = step(yaml_text.remove_key(text, (arm, "box")),
                                "remove the box")
            else:
                text = step(_set_block(text, (arm,), "box", _BOX_KEYS, box),
                            "build the box block")
    except ValueError as exc:
        return False, str(exc)

    problems = validate_goal(text)
    if problems:
        return False, "; ".join(problems)
    backup = _backup_of(target)
    if target.is_file() and not backup.exists():
        shutil.copy2(target, backup)
    target.write_text(text)
    return True, str(target)
```

One subtlety the point-mode test will catch: numbers posted as JSON ints
(`[0, 0, 1]` for a normal) must round-trip — `yaml_text.render` writes ints
as ints and `scalar` reads them back as floats, which `validate_goal`'s
`_three_finite` requires. If a test fails on int elements, coerce list
elements to float in `_set_key` before rendering.

- [ ] **Step 4: Add the POST route in `server.py`** (after `/api/goal`)

```python
            elif route == "/api/goal/fields":
                req = self._body()
                ok, message = plan.write_goal_fields(
                    str(req.get("arm", "")), req.get("fields") or {})
                self._json({"ok": True} if ok else {"error": message},
                           200 if ok else 400)
```

And in `test_static.py`'s `PlannerEndpoints` class add:

```python
    def test_goal_fields_endpoint_exists_in_server(self):
        self.assertIn('"/api/goal/fields"', SERVER)
```

- [ ] **Step 5: Run the module tests, then the full suite**

Run: `python3 -m unittest Christian_control.tools.panel.tests.test_plan`
Expected: PASS. Then full discovery: OK.

- [ ] **Step 6: Commit**

```bash
git add Christian_control/tools/panel/plan.py \
        Christian_control/tools/panel/server.py \
        Christian_control/tools/panel/tests/test_plan.py \
        Christian_control/tools/panel/tests/test_static.py
git commit -m "panel: structured per-arm goal edits through the raw editor's gate"
```

---

### Task 6: CONFIG tab UI — planner tuning and joint limits

**Files:**
- Modify: `Christian_control/tools/panel/static/index.html` (two cards after the "Velocity limits" card, before "Thresholds")
- Modify: `Christian_control/tools/panel/static/panel.js` (fetch + two render functions; hook into the existing config load path at `loadConfig()`, panel.js:960-965)
- Modify: `Christian_control/tools/panel/static/panel.css` (only if a new class is needed — reuse `knob-table`, `knob-danger`, `vector-knob-row` styles first)
- Modify: `Christian_control/tools/panel/tests/test_static.py` (ids)

**Interfaces:**
- Consumes: `GET /api/planner-config`, `POST /api/planner-config/set` (Task 4), existing helpers `$`, `getJSON`, `postJSON`, `setNotice` in panel.js.
- Produces: elements `#planner-knob-table`, `#planner-config-error`, `#joint-limit-edit-table`.

- [ ] **Step 1: Add the cards to `index.html`**

```html
    <section class="card">
      <div class="card-head">
        <h2>Planner tuning</h2>
        <span class="card-note">editable · written to planner.yaml · applies at the next solve — no rebuild</span>
      </div>
      <div id="planner-knob-table" class="knob-table"></div>
      <p class="card-foot" id="planner-config-error"></p>
    </section>

    <section class="card">
      <div class="card-head">
        <h2>Planner joint limits</h2>
        <span class="card-note">editable · written to joint_limits.yaml · Kinova table values feeding the planner's dynamics checks</span>
      </div>
      <div id="joint-limit-edit-table" class="knob-table"></div>
      <p class="card-foot">
        Every field here is a safety-relevant table value — the planner's
        joint-limit and dynamics validation reads them. Radians and rad/s;
        the panel checks only that lower stays below upper.
      </p>
    </section>
```

- [ ] **Step 2: Fetch and render in `panel.js`**

In `loadConfig()` (the function at panel.js:960-965 that awaits `/api/config`), also fetch the planner config, then call the two renderers:

```js
    state.plannerConfig = await getJSON('/api/planner-config');
    renderPlannerKnobs();
    renderJointLimitEditor();
```

Add the renderers, modelled line-for-line on the existing knob row builder (panel.js:975-1010) — same classes, same commit-on-change pattern:

```js
// Planner tuning rows. Grouped by the yaml section (the part before the
// dot) so the table reads like the file. vec3 knobs get three inputs
// committed together, like the vector knob row.
function renderPlannerKnobs() {
  const host = $('planner-knob-table');
  host.innerHTML = '';
  const knobs = state.plannerConfig?.planner || {};
  let section = null;
  for (const [name, entry] of Object.entries(knobs)) {
    const head = name.split('.')[0];
    if (head !== section) {
      section = head;
      const h = document.createElement('div');
      h.className = 'knob-section';
      h.textContent = section;
      host.appendChild(h);
    }
    const row = document.createElement('div');
    row.className = 'knob-row';
    const label = document.createElement('span');
    label.className = 'knob-name';
    label.textContent = name.split('.').slice(1).join('.');
    row.appendChild(label);
    const commit = async (value) => {
      const result = await postJSON('/api/planner-config/set',
        { file: 'planner', name, value });
      setNotice('planner-config-error',
        result.error ? `${name}: ${result.error}` : '', 'is-stop');
      if (!result.error) { state.plannerConfig.planner[name].value = result.value; }
      renderPlannerKnobs();
    };
    if (entry.type === 'vec3') {
      const inputs = [];
      for (let i = 0; i < 3; i++) {
        const input = document.createElement('input');
        input.className = 'knob-input num';
        input.value = Array.isArray(entry.value) ? entry.value[i] : '';
        inputs.push(input);
        row.appendChild(input);
      }
      inputs.forEach(inp => inp.addEventListener('change',
        () => commit(inputs.map(i => i.value.trim()))));
    } else if (entry.type === 'bool') {
      const input = document.createElement('input');
      input.type = 'checkbox';
      input.checked = entry.value === 'true' || entry.value === true;
      input.addEventListener('change', () => commit(input.checked));
      row.appendChild(input);
    } else {
      const input = document.createElement('input');
      input.className = 'knob-input num';
      input.value = entry.value ?? '';
      input.addEventListener('change', () => commit(input.value.trim()));
      row.appendChild(input);
    }
    const note = document.createElement('span');
    note.className = 'knob-doc';
    note.textContent = entry.doc;
    row.appendChild(note);
    host.appendChild(row);
  }
}

// Joint-limit rows: one per actuator per section, every input danger-styled
// because every value is dangerous — the server says so per field and the
// styling must never disagree with the data.
function renderJointLimitEditor() {
  const host = $('joint-limit-edit-table');
  host.innerHTML = '';
  const table = state.plannerConfig?.joint_limits || {};
  for (const [section, actuators] of Object.entries(table)) {
    const h = document.createElement('div');
    h.className = 'knob-section knob-danger';
    h.textContent = section;
    host.appendChild(h);
    for (const [actuator, entry] of Object.entries(actuators)) {
      const row = document.createElement('div');
      row.className = 'knob-row';
      const label = document.createElement('span');
      label.className = 'knob-name knob-danger';
      label.textContent = `${actuator} ${entry.unit || ''}`;
      row.appendChild(label);
      for (const bound of ['lower_limit', 'upper_limit']) {
        const input = document.createElement('input');
        input.className = 'knob-input num knob-danger';
        input.value = entry[bound] ?? '';
        input.addEventListener('change', async () => {
          const result = await postJSON('/api/planner-config/set',
            { file: 'joint_limits', section, actuator, bound,
              value: input.value.trim() });
          setNotice('planner-config-error',
            result.error ? `${section}/${actuator}: ${result.error}` : '',
            'is-stop');
          state.plannerConfig = await getJSON('/api/planner-config');
          renderJointLimitEditor();
        });
        row.appendChild(input);
      }
      host.appendChild(row);
    }
  }
}
```

If `knob-section` does not exist in `panel.css`, add it beside the knob styles:

```css
.knob-section { font-weight: 600; margin-top: 0.6em; text-transform: uppercase; font-size: 0.85em; color: var(--ink-dim); }
```

- [ ] **Step 3: Pin the ids in `test_static.py`**

The existing id-consistency tests (`html_ids`/`js_ids`) cover `$()` lookups automatically. Add explicit presence tests to the `PlannerEndpoints` class:

```python
    def test_planner_config_ui_hooks_exist(self):
        ids = html_ids(HTML)
        for element in ("planner-knob-table", "planner-config-error",
                        "joint-limit-edit-table"):
            self.assertIn(element, ids, element)
```

- [ ] **Step 4: Run the full suite**

Run: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .`
Expected: OK.

- [ ] **Step 5: Commit**

```bash
git add Christian_control/tools/panel/static/index.html \
        Christian_control/tools/panel/static/panel.js \
        Christian_control/tools/panel/static/panel.css \
        Christian_control/tools/panel/tests/test_static.py
git commit -m "panel: planner tuning and joint limits editable from CONFIG"
```

---

### Task 7: TARGETS tab UI — per-arm goal cards, raw view collapsed

**Files:**
- Modify: `Christian_control/tools/panel/static/index.html` (TARGETS section, ~line 221-230: goal cards host before the textarea; textarea wrapped in `<details>`)
- Modify: `Christian_control/tools/panel/static/panel.js` (card builder + save; repopulate on every goal load)
- Modify: `Christian_control/tools/panel/static/panel.css` (card layout)
- Modify: `Christian_control/tools/panel/tests/test_static.py` (ids)

**Interfaces:**
- Consumes: `GET /api/goal` (existing — `{"text", "parsed": {"session_arms", "arms": {...}}}`), `POST /api/goal/fields` (Task 5), the existing goal load function that fills `#goal-text` (panel.js ~1270, the one that sets `state.goal` and calls `setNotice('goal-status', ...)`).
- Produces: `#goal-cards` container with one card per arm; `#goal-raw-details` wrapping the existing `#goal-text` + SAVE GOAL button.

- [ ] **Step 1: Rework the TARGETS markup**

Replace the textarea block (index.html ~line 228) with:

```html
      <div id="goal-cards" class="goal-cards"></div>
      <details id="goal-raw-details">
        <summary>advanced — raw goal.yaml</summary>
        <textarea id="goal-text" class="code-area" spellcheck="false" rows="20"></textarea>
      </details>
```

Keep the existing SAVE GOAL / SOLVE AND PREVIEW button row exactly where it is; SAVE GOAL keeps saving the raw textarea. (Move the SAVE GOAL button inside the `<details>` only if it sits visually orphaned — implementer's call; keep ids unchanged either way.)

- [ ] **Step 2: Build the cards in `panel.js`**

Add a builder and a save routine. Every control is created per arm with a helper so ids stay unique (`goal-frame-right`, `goal-mode-right`, ...). The card repopulates from `state.goal.parsed.arms[arm]` every time the goal is (re)loaded — after a raw save, after a fields save, at startup — so the form always shows what is on disk.

```js
const GOAL_FRAMES = ['mount', 'right_base', 'left_base'];

function goalField(card, arm, name) {
  return card.querySelector(`[data-field="${name}"]`);
}

function numberInput(field, value) {
  const input = document.createElement('input');
  input.className = 'knob-input num';
  input.dataset.field = field;
  input.value = value ?? '';
  return input;
}

function vecInputs(host, field, values) {
  const three = [];
  for (let i = 0; i < 3; i++) {
    const input = numberInput(`${field}-${i}`, Array.isArray(values) ? values[i] : '');
    three.push(input);
    host.appendChild(input);
  }
  return three;
}

// One card per arm, rebuilt from the parsed goal whenever it loads. The
// wall of YAML is gone from the front page; this is its replacement.
function renderGoalCards() {
  const host = $('goal-cards');
  host.innerHTML = '';
  const arms = state.goal?.parsed?.arms || {};
  for (const arm of ['right', 'left']) {
    const block = arms[arm] || {};
    const isCircle = 'path' in block;
    const card = document.createElement('section');
    card.className = 'card goal-card';
    card.dataset.arm = arm;

    const head = document.createElement('div');
    head.className = 'card-head';
    head.innerHTML = `<h2>${arm.toUpperCase()} ARM</h2>`;
    card.appendChild(head);

    // frame
    const frameRow = document.createElement('div');
    frameRow.className = 'goal-row';
    frameRow.append('frame ');
    const frame = document.createElement('select');
    frame.dataset.field = 'frame';
    for (const f of GOAL_FRAMES) {
      const opt = document.createElement('option');
      opt.value = f; opt.textContent = f;
      opt.selected = (block.frame || 'mount') === f;
      frame.appendChild(opt);
    }
    frameRow.appendChild(frame);
    card.appendChild(frameRow);

    // mode radio
    const modeRow = document.createElement('div');
    modeRow.className = 'goal-row';
    for (const mode of ['point', 'circle']) {
      const label = document.createElement('label');
      const radio = document.createElement('input');
      radio.type = 'radio';
      radio.name = `goal-mode-${arm}`;
      radio.value = mode;
      radio.checked = isCircle ? mode === 'circle' : mode === 'point';
      radio.addEventListener('change', () => {
        card.querySelector('[data-pane="point"]').hidden = mode !== 'point';
        card.querySelector('[data-pane="circle"]').hidden = mode !== 'circle';
      });
      label.append(radio, ` ${mode === 'point' ? 'point goal' : 'traced circle'}`);
      modeRow.appendChild(label);
    }
    card.appendChild(modeRow);

    // point pane
    const point = document.createElement('div');
    point.dataset.pane = 'point';
    point.hidden = isCircle;
    const goalRow = document.createElement('div');
    goalRow.className = 'goal-row';
    goalRow.append('goal xyz, m ');
    vecInputs(goalRow, 'goal', block.goal);
    point.appendChild(goalRow);
    card.appendChild(point);

    // circle pane
    const circle = document.createElement('div');
    circle.dataset.pane = 'circle';
    circle.hidden = !isCircle;
    const p = block.path || {};
    const centreRow = document.createElement('div');
    centreRow.className = 'goal-row';
    centreRow.append('centre xyz, m ');
    vecInputs(centreRow, 'centre', p.centre);
    circle.appendChild(centreRow);
    const geomRow = document.createElement('div');
    geomRow.className = 'goal-row';
    geomRow.append('radius, m ');
    geomRow.appendChild(numberInput('radius_m', p.radius_m));
    geomRow.append(' duration, s ');
    geomRow.appendChild(numberInput('duration_s', p.duration_s));
    circle.appendChild(geomRow);
    const normalRow = document.createElement('div');
    normalRow.className = 'goal-row';
    normalRow.append('normal ');
    vecInputs(normalRow, 'normal', p.normal);
    circle.appendChild(normalRow);
    const circOrientRow = document.createElement('div');
    circOrientRow.className = 'goal-row';
    circOrientRow.append('orientation ');
    const circOrient = document.createElement('select');
    circOrient.dataset.field = 'path-orientation';
    for (const mode of ['fixed', 'radial']) {
      const opt = document.createElement('option');
      opt.value = mode; opt.textContent = mode;
      opt.selected = (p.orientation || 'fixed') === mode;
      circOrient.appendChild(opt);
    }
    circOrientRow.appendChild(circOrient);
    circOrientRow.append(' rpy, deg ');
    vecInputs(circOrientRow, 'path-rpy', p.orientation_rpy_deg);
    circle.appendChild(circOrientRow);
    card.appendChild(circle);

    // arm-level orientation: inherit or state
    const orientRow = document.createElement('div');
    orientRow.className = 'goal-row';
    const inherit = document.createElement('input');
    inherit.type = 'checkbox';
    inherit.dataset.field = 'inherit';
    inherit.checked = !('orientation_rpy_deg' in block);
    const inheritLabel = document.createElement('label');
    inheritLabel.append(inherit, ' inherit orientation at start');
    orientRow.appendChild(inheritLabel);
    orientRow.append(' rpy, deg ');
    const rpy = vecInputs(orientRow, 'rpy', block.orientation_rpy_deg);
    const warn = document.createElement('p');
    warn.className = 'goal-warn';
    warn.textContent = 'Inheriting makes the goal depend on where the arm '
      + 'was parked — the same goal can plan from one start and fail from '
      + 'another. Stating an orientation makes it mean the same thing every run.';
    const syncInherit = () => {
      rpy.forEach(i => { i.disabled = inherit.checked; });
      warn.hidden = !inherit.checked;
    };
    inherit.addEventListener('change', syncInherit);
    card.appendChild(orientRow);
    card.appendChild(warn);

    // obstacle box
    const boxRow = document.createElement('div');
    boxRow.className = 'goal-row';
    const boxOn = document.createElement('input');
    boxOn.type = 'checkbox';
    boxOn.dataset.field = 'box-on';
    boxOn.checked = 'box' in block;
    const boxLabel = document.createElement('label');
    boxLabel.append(boxOn, ` obstacle box (axis-aligned, ${arm}_base frame)`);
    boxRow.appendChild(boxLabel);
    card.appendChild(boxRow);
    const boxPane = document.createElement('div');
    boxPane.dataset.pane = 'box';
    boxPane.hidden = !boxOn.checked;
    const boxCentreRow = document.createElement('div');
    boxCentreRow.className = 'goal-row';
    boxCentreRow.append('center ');
    vecInputs(boxCentreRow, 'box-center', block.box?.center);
    boxPane.appendChild(boxCentreRow);
    const boxExtentRow = document.createElement('div');
    boxExtentRow.className = 'goal-row';
    boxExtentRow.append('half extent ');
    vecInputs(boxExtentRow, 'box-half', block.box?.half_extent);
    boxPane.appendChild(boxExtentRow);
    boxOn.addEventListener('change', () => { boxPane.hidden = !boxOn.checked; });
    card.appendChild(boxPane);

    // save
    const saveRow = document.createElement('div');
    saveRow.className = 'goal-row';
    const save = document.createElement('button');
    save.type = 'button';
    save.className = 'action';
    save.textContent = `SAVE ${arm.toUpperCase()} GOAL`;
    save.addEventListener('click', () => saveGoalCard(card, arm));
    saveRow.appendChild(save);
    card.appendChild(saveRow);

    syncInherit();
    host.appendChild(card);
  }
}

function vecValue(card, field) {
  return [0, 1, 2].map(i =>
    goalField(card, null, `${field}-${i}`).value.trim());
}

async function saveGoalCard(card, arm) {
  const mode = card.querySelector(`input[name="goal-mode-${arm}"]:checked`).value;
  const fields = { mode, frame: goalField(card, arm, 'frame').value };
  if (mode === 'point') {
    fields.goal = vecValue(card, 'goal');
  } else {
    fields.path = {
      type: 'circle',
      centre: vecValue(card, 'centre'),
      radius_m: goalField(card, arm, 'radius_m').value.trim(),
      normal: vecValue(card, 'normal'),
      duration_s: goalField(card, arm, 'duration_s').value.trim(),
      orientation: goalField(card, arm, 'path-orientation').value,
      orientation_rpy_deg: vecValue(card, 'path-rpy'),
    };
  }
  fields.orientation_rpy_deg =
    goalField(card, arm, 'inherit').checked ? null : vecValue(card, 'rpy');
  fields.box = goalField(card, arm, 'box-on').checked
    ? { center: vecValue(card, 'box-center'),
        half_extent: vecValue(card, 'box-half') }
    : null;
  const result = await postJSON('/api/goal/fields', { arm, fields });
  if (result.error) {
    setNotice('goal-status', `${arm}: ${result.error}`, 'is-stop');
    return;
  }
  setNotice('goal-status', `${arm} goal saved. Solve to see what it produces.`, null);
  await loadGoal();   // the existing /api/goal fetch — repopulates text AND cards
}
```

Numeric fields post strings; `write_goal_fields`' validation and `_three_finite` handle conversion server-side — if the server-side `scalar` leaves a non-numeric string, `validate_goal` refuses with a sentence, which is the wanted behaviour. Convert with `parseFloat` client-side ONLY if the round-trip test in Step 4 shows strings failing on valid input.

Then find the existing goal load function (the one filling `#goal-text`, panel.js ~1270) and add `renderGoalCards();` after it stores `state.goal`. Name it `loadGoal` if it is currently anonymous, so `saveGoalCard` can call it.

- [ ] **Step 3: Style the cards in `panel.css`**

```css
.goal-cards { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
.goal-card .goal-row { display: flex; align-items: center; gap: 6px; margin: 6px 0; flex-wrap: wrap; }
.goal-warn { color: var(--warn); font-size: 0.85em; }
#goal-raw-details { margin-top: 10px; }
#goal-raw-details summary { cursor: pointer; color: var(--ink-dim); }
@media (max-width: 900px) { .goal-cards { grid-template-columns: 1fr; } }
```

- [ ] **Step 4: Pin ids and re-run the suite**

In `test_static.py`, add to `PlannerEndpoints`:

```python
    def test_goal_cards_ui_hooks_exist(self):
        ids = html_ids(HTML)
        self.assertIn("goal-cards", ids)
        self.assertIn("goal-raw-details", ids)
        self.assertIn("goal-text", ids)   # the raw editor survives, collapsed
```

Run: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .`
Expected: OK.

- [ ] **Step 5: Commit**

```bash
git add Christian_control/tools/panel/static/index.html \
        Christian_control/tools/panel/static/panel.js \
        Christian_control/tools/panel/static/panel.css \
        Christian_control/tools/panel/tests/test_static.py
git commit -m "panel: per-arm goal cards; raw goal.yaml collapses to advanced"
```

---

### Task 8: Whole-feature verification and the intent record

**Files:**
- Modify: `docs/intent/story.md` (repo root — new approved-goal entry)

**Interfaces:**
- Consumes: everything above.
- Produces: the finished branch.

- [ ] **Step 1: Full suite, fresh eyes**

Run: `python3 -m unittest discover -s Christian_control/tools/panel/tests -t .`
Expected: OK, count ≥ 237 + all new tests, `skipped=1`.

- [ ] **Step 2: Validation against behaviour the change did not create**

The changed files are read by consumers this work never touched. Prove the
edits are consumable by the real readers, not just by our own round-trips:

```bash
# planner.yaml after a panel-style edit still parses for the bridge's
# strict reader? The bridge binary itself is robot-adjacent, so use the
# session test, which exercises the config plumbing hardware-free:
bash Christian_control/planner_bridge/tests/test_run_session.sh
```

Expected: PASSED. Also hand-check once: make one knob edit through
`planner_config.write_planner_knob` against a temp copy and `diff` it with
the original — the diff must be exactly one line.

- [ ] **Step 3: Update the intent story**

Add under Approved goals in `docs/intent/story.md` (cite the raw-log entry of Christian's 16:19 request and the design-review confirmations of this session):

```markdown
- The browser panel is the one place run configuration is edited: planner
  tuning (`planner.yaml`) and the planner's joint-limit tables
  (`joint_limits.yaml`, danger-flagged by explicit choice) become editable,
  and each arm's goal is edited through structured forms rather than raw
  YAML, with the raw file kept as a collapsed advanced view. The why
  Christian gave: tuning the planner from the browser was impossible and
  the goal editor was "too much" — the whole per-run surface belongs in
  one place, for both arms. Panel edits stay value-replacements in the
  file's own text, so hand editing and comments survive. (Prompt:
  raw-prompt-log 2026-08-12 ~16:19; design approved the same session —
  spec at docs/superpowers/specs/2026-08-12-planner-config-panel-design.md.)
```

- [ ] **Step 4: Commit the story separately**

```bash
git add docs/intent/story.md
git commit -m "intent: the panel as the one place run configuration is edited"
```

- [ ] **Step 5: Report**

Report: files added/modified, test count before/after, the one-line-diff
evidence from Step 2, and that the branch awaits Christian's review and
merge — no push to master, no hardware touched.
