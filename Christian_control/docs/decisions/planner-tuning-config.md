# Planner tuning lives in a file; safety policy stays compiled

Date: 2026-08-06

Status: adopted. `planning/config/planner.yaml` is read on every
bridge run.

## Decision

The planner's tuning — plan pacing and every GPMP2 factor-graph weight —
moves out of scattered C++ literals into one strictly-validated YAML file.
Nothing about the controller changes, and nothing safety-bearing becomes
runtime-settable.

## Why this does not reopen `compiled-config.md`

`compiled-config.md` removed the controller's CLI/TOML front end because "a
second configuration surface was somewhere for the compiled defaults and the
effective values to disagree", and because "the compiled constants are the
only thing between the operator and the arm."

That argument is about the thing that commands the arm. It does not transfer
to `planner_bridge`, for three specific reasons:

1. **The bridge commands nothing.** It writes a trajectory to a pipe.
   `ValidateJointPath` and the `Targets.cpp` ingest gates stand between it
   and motion, and those read compiled `Config.h` constants that stay
   compiled.
2. **The bridge already had a runtime config file.** `config/goal.yaml` is
   this same surface. This widens something that exists rather than opening
   a new one.
3. **The removed TOML surface duplicated controller gains and safety
   thresholds.** Planner tuning has no compiled counterpart on the
   controller side, so the "defaults and effective values disagree" failure
   mode has nothing to disagree with.

The four boundaries from `runtime-config.md` are inherited verbatim, because
they are already the answer to "how does a config file bite me later":

- **No working-directory discovery.** The default path is resolved from the
  executable (`config/planner.yaml`, beside `goal.yaml`); `--planner-config
  PATH` overrides it. Program behaviour never depends on where it was
  started from.
- **No safety keys.** Everything in `control/Config.h` stays
  compiled. Naming one of those keys in this file is an unknown-key error,
  not an override — the same treatment `stop_on_fault` had.
- **Typos are hard errors.** Every key is required, unknown keys are
  refused, and every value is range-checked. Nothing falls back to a silent
  default.
- **Every run is self-describing.** The effective values and the file's
  digest are echoed to the bridge's diagnostics on every run.

## What moved, and what deliberately did not

Moved to `planner.yaml`: `motion.nominal_speed_mps`, `motion.min_duration_s`,
`motion.waypoints`, `obstacles.epsilon_dist_m`, `obstacles.collision_sigma`,
`smoothness.qc_scale`, `goal.position_sigma_xyz`, `goal.rotation_sigma_rpy`,
`solver.max_iterations`.

Deliberately not moved, each with a reason recorded at the top of the file
itself so the answer is where someone would look for it:

- **Joint limits** — `TrajectoryGeneration/config/joint_limits.yaml`.
  Kinova table values; they already have a file.
- **SDF grid extent** — `WorldSdf.h`. Not a preference: gpmp2 reports "no
  obstacle" outside the grid, silently, so the grid must contain the arm's
  reachable envelope. `test_grid_coverage` enforces that and would not see a
  change made in a config file. This is the extent/resolution split — extent
  is a derived safety property, resolution would be a legitimate knob, and
  only the former is settled here.
- **Arm collision-sphere radius** — `GenerateArmModel.cpp`. Sets the same
  envelope `test_grid_coverage` checks, so it cannot move without that test
  moving with it. A candidate for a later change that does both together.
- **`target_dt`** — left compiled. `TrajectoryEmit.cpp` caps the emitted
  block at 1000 points and downsamples, so for any plan longer than about a
  second the 1 kHz densification is discarded. Exposing it as a "smoothness"
  knob would misrepresent what it does.

## The y-axis asymmetry is exposed, not fixed

`goal.position_sigma_xyz` is `[0.01, 0.1, 0.01]` — y is ten times looser
than x and z, inherited from an earlier task. That is now visible instead of
buried in a default argument, which was the point. Making it uniform is a
behaviour change on the motion path and needs its own verification; folding
it into a config refactor is how a refactor quietly becomes a behaviour
change.

## Evidence

- `tests/test_planner_config.cpp` holds the checked-in file to the compiled
  defaults (so file and code cannot drift), and pins the rejection of an
  unknown key, a missing key, an out-of-range value, a wrong-length vector,
  and a missing file.
- `tests/test_plan_solver.cpp` now loads the checked-in `planner.yaml`, so
  a config edit that breaks solving fails in the suite rather than on
  hardware.
- Wiring confirmed end to end against the real solve: changing
  `nominal_speed_mps` from 0.05 to 0.01 moved the planned duration from
  4.12 s to 20.62 s (exactly the 5x expected), and `waypoints` 10 to 30
  moved the solve from 25 ms to 119 ms. A key that did nothing would not
  have shown either.
- The real binary refuses a typo before the solve:
  `error: planner config: motion keys differ; missing = [nominal_speed_mps],
  unknown = [nominal_speed]`, exit 1, nothing written to the pipe.
- `ctest`: planner_bridge 10/10, basic_control 11/11.

## Provenance digest

The echo carries an FNV-1a 64 digest of the file's exact bytes. It is a
change detector, not a cryptographic hash, and is named `fnv1a64` in the
output so it is not mistaken for one. The choice was to avoid adding an
OpenSSL link dependency to a build whose third-party rpath handling is
already delicate; the property needed — "this run used exactly these bytes"
— is fully met.

## Not addressed here

`SolveToPosition` still reports `final_goal_error_m` without an acceptance
threshold, so a plan that ends far from the goal is returned as `ok`. A
`goal.max_error_m` key would be a natural home for that threshold, but
adding one changes what the pipeline accepts and belongs in its own
decision.
