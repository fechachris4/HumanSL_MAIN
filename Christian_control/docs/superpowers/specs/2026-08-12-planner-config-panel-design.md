# Planner configuration and goal forms in the control panel

Date: 2026-08-12
Status: design approved by Christian 2026-08-12 (interactive review).

## Goal

In Christian's words: planner tuning should be editable from the browser,
and the goals should be changeable in a better way than the current wall of
YAML — for the left arm and the right arm both.

Today the panel edits the controller's compiled knobs (`Config.h`) and the
goal file as raw text, but `planner.yaml` and `joint_limits.yaml` cannot be
edited from the browser at all, and the TARGETS tab is a bare textarea
showing the whole commented file.

Decisions made at design review:

- Scope: `planner.yaml` and `joint_limits.yaml` both become editable
  (Christian's explicit choice; joint limits stay danger-flagged).
- Goal editing: structured per-arm forms, with the raw-text editor kept
  beneath them, collapsed, as the advanced fallback.
- Placement: planner tuning lives in the CONFIG tab in its own section, so
  CONFIG is the one place every knob lives.

## The one write rule

The panel never regenerates a YAML file. It replaces single values in
place, in the file's own text, exactly as the `Config.h` knob editor does.
Every comment in the file — the measured orientations, the traps, the
explanations — survives both panel edits and hand edits. Before the panel
first touches a file it copies it to `<name>.panel.bak`, once, so the
backup holds the pre-panel state.

A failed write (unknown key, bad type, out of range, missing file) leaves
the file byte-for-byte unchanged and returns the reason as a sentence.

## Components

### Shared YAML-subset reader

`plan.py`'s comment-aware line parsing (`_strip_comment`, `_scalar`, the
indent walk) moves to a small shared helper module so `goal.yaml`,
`planner.yaml` and `joint_limits.yaml` are all read by the identical rules.
One abstraction, two real users (`plan.py`, `planner_config.py`) — allowed
by the house rule. No PyYAML; the panel stays stdlib-only.

### `planner_config.py` (new, mirrors `config_file.py`)

- `PLANNER_KNOBS`: the write whitelist for `planner.yaml`, keyed by dotted
  path (`motion.nominal_speed_mps`, `solver.max_iterations`,
  `seeding.ik_seed`, …) — every key in the file, since the bridge already
  refuses unknown or missing keys. Each entry carries the type
  (double/int/bool/vec3), a stated range where one exists, and a one-line
  meaning lifted from the file's comments.
- `read_planner_knobs()` returns current values plus the file text.
- `write_planner_knob(dotted_key, value)` validates, then replaces the
  value in place under its section heading.
- `read_joint_limits_file()` / `write_joint_limit(section, actuator,
  bound, value)` do the same for `joint_limits.yaml`, addressing a value
  by its section (`position_limits`, `velocity_limits`,
  `acceleration_limits`), its actuator, and which bound
  (`lower_limit`/`upper_limit`, or the single value where the section has
  one). Every joint-limit field is
  marked `dangerous` in read output: these numbers feed the planner's
  dynamics validation, so a wrong value weakens a real check. The UI shows
  them the way it shows the guard overrides.

The panel's validation is first-line only; the bridge's strict validation
(hard error naming the key) remains the final authority. The UI labels the
section "applies at the next solve — no rebuild", the honest difference
from the compiled knobs above it.

### Structured goal writes (`plan.py` additions)

`write_goal_fields(arm, fields)` edits one arm's block of `goal.yaml` in
place:

- Replace the value of an existing key (`frame`, `goal`,
  `orientation_rpy_deg`, `path.*`, `box.*`).
- Insert a missing key at the end of the arm's block with the block's
  indent.
- Remove a key (and a nested block's lines) when the form drops it —
  switching point↔circle removes the other key's lines, so the bridge's
  "goal and path are mutually exclusive" rule can never be tripped from
  the forms.
- Unchecking the obstacle box removes the `box:` block; unchecking a
  stated orientation removes `orientation_rpy_deg`.

The existing whole-text `write_goal(text)` stays untouched — it is what
the raw editor saves through, with the same validation as today.

### Server wiring (`server.py`)

- `GET /api/planner-config` — planner knobs, joint limits, paths, errors.
- `POST /api/planner-config/set` — `{file, key, value}` for either file.
- `POST /api/goal/fields` — `{arm, fields}` structured goal save.
- Existing endpoints untouched; `GET /api/goal` already returns text plus
  parsed structure, which is what the forms initialize from.

### UI

CONFIG tab, new section under the compiled knobs:

- "PLANNER — applies at the next solve, no rebuild": knob rows grouped by
  the YAML sections (motion, obstacles, smoothness, goal, solver,
  path_following, seeding), same row style as the compiled knobs.
- "JOINT LIMITS" table, one row per actuator, danger-styled like the guard
  overrides.

TARGETS tab, replacing the bare textarea as the primary editor:

- Two arm cards (RIGHT, LEFT): frame dropdown; a point-goal / circle-path
  radio; XYZ fields for a point, centre/radius/normal/duration and
  fixed/radial orientation for a circle; orientation RPY fields with an
  explicit "inherit at start" checkbox — when checked the card shows the
  known warning that an inherited orientation makes the goal depend on
  where the arm was parked; an optional obstacle box behind a checkbox
  (axis-aligned, in that arm's own base frame, as the file documents).
- SAVE writes only the fields that changed, through
  `/api/goal/fields`.
- The raw editor moves beneath, collapsed under "advanced (raw
  goal.yaml)", still saving whole-text through the existing path.
- After any save — form or raw — the panel re-reads the file and
  repopulates both views, so they always show what is really on disk.

## Error handling

- A file that fails to parse is reported; the forms disable and the raw
  view stays usable.
- A refused write names the key and the reason; nothing is half-written.
- The panel never invents a default: an absent optional key renders as
  absent (unchecked), never as a guessed value.

## Testing (hardware-free, house pattern)

- Round-trip tests proving comments survive every kind of edit (replace,
  insert, remove, mode switch) on realistic file texts.
- Refusal tests proving a bad write leaves the file byte-identical.
- `planner.yaml` and `joint_limits.yaml` knob read/write tests, including
  the dangerous flag on every joint-limit field.
- Server dispatch tests for the three new endpoints.
- The existing 237 tests keep passing unchanged.

## Deliberately not in scope

- No change to what the bridge or session script read or validate.
- No editing of `WorldSdf.h`, sphere radii, or anything compiled — those
  stay where `planner.yaml`'s own header points.
- No second opinion on planning judgements: reachability, clearance and
  orientation sense remain the bridge's call.
