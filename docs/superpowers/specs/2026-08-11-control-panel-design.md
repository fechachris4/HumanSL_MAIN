# Control panel — design

Date: 2026-08-11. Status: design approved in conversation; no code written.

## Purpose

Give Christian one screen from which to configure, plan, run and watch the
SRL controller, so that running the system does not depend on remembering
which file holds which knob, which binary is stale, and which flag the
session script wants. The panel replaces remembering, not judgement.

## The rule

> The browser observes and commands. The controller decides whether motion
> is safe.

Two consequences that the rest of this document keeps returning to.

The panel's stop is a **trigger, not a mechanism**. It enters through
`run_session.sh` and becomes the same `SIGINT` a keyboard sends, arriving at
the controller's existing stop path. The browser never acquires a way to
stop the arm that the terminal does not already have.

The physical emergency stop is outside the diagram entirely. No software
here observes it, so the panel says so permanently rather than showing a
status it cannot know.

```
                    CONTROL PANEL
                         |
          +--------------+--------------+
          |              |              |
       Configure       Plan            Run
          |              |              |
          v              v              v
       Config.h       planner       run_session
                         |              |
                         v              v
                    trajectory      controller
                                        |
                               +--------+--------+
                               |                 |
                            Kinova           telemetry
                               |                 |
                               |                 v
                               |               panel
                               |
                         SAFETY LIVES HERE
```

## Decisions taken

- **Browser panel, served from the Linux workstation.** Every vendored
  library in `third_party/lib` is ELF x86-64 (Kortex, GTSAM/GPMP2,
  Pinocchio, coal), so nothing here builds or runs on macOS. "Cross-platform"
  therefore means the frontend is a client; the robot software stays with the
  robot. A browser needs no install on either operating system.
- **Extend `Christian_control/tools/control_panel.py`** rather than start a
  new application. It is already stdlib-only, already reads and writes the
  whitelisted `Config.h` knobs, already builds and already lists runs.
- **Supervised start and stop from the panel.** Start is localhost-only;
  stop works from anywhere. See "Session control".
- **Compile-time configuration stays.** The rebuild loop becomes visible
  instead of becoming a surprise. A runtime-settable non-safety subset is
  deferred until a specific knob proves it needs to change between runs.
- **No simulation.** There is no simulator in this repository; MuJoCo and
  `PlantBackend` live in `msc_project` with no hardware sibling. The
  hardware/simulation indicator can therefore only ever read HARDWARE, which
  is the safe direction for it to be wrong in.
- **Both arms in one 3D scene, one arm's numbers at a time.** Inter-arm
  proximity is only visible in a shared scene; fourteen joint bars are more
  than anyone reads under pressure. The unselected arm keeps a single
  worst-case line so it can still raise an alarm.
- **Stale telemetry: age always on screen, hard visual break at 2 s.** The
  stop control is unaffected by staleness.
- **The stop is labelled "Stop and release the arm."** See "What does not
  exist".

## Visual direction

Machine readout. A pale instrument ground with everything desaturated, and
**colour reserved entirely for abnormal states** — nothing on a healthy
screen is red, amber or green. This follows high-performance HMI practice
(a dark, colourful display buries the one alarm that matters), suits a
bright lab, and is the opposite of the default robot dashboard.

Two signatures:

- **Fixed-width value slots with tabular figures**, so the panel is
  physically motionless while values change. Movement on screen then means
  something happened, rather than a digit growing wider.
- **Joint bars drawn to true travel**, so a joint near its limit is visibly
  near the end of its real range rather than of a normalised 0-100 bar.

Borrowed from the engineering-drawing register for one job only: in the 3D
scene, the gap between measured and commanded pose is called out as a
dimension line with the number on it.

One variable font is vendored locally (no CDN), carrying a condensed face
for labels and a true tabular monospace for every number. Palette: instrument
ground, ink, hairline, warning amber, stop red, and one desaturated blue used
*only* for "what was asked for", so the commanded pose is the same colour
everywhere on screen.

## Architecture

One new long-running process: the panel server, Python 3 stdlib, no
dependencies. The controller and bridge are untouched and are still launched
by `run_session.sh`.

Binding: loopback always; a LAN interface only when started with `--lan`, so
remote viewing is deliberate rather than a default that exposes a start
button to the lab network.

### Files

`control_panel.py` shrinks to a launcher. The work moves into
`Christian_control/tools/panel/`, one file per job:

| File | Job |
| --- | --- |
| `server.py` | HTTP routing, static files, SSE. Nothing else. |
| `config_file.py` | Whitelisted `Config.h` read/write (exists today). |
| `build.py` | cmake invocation and the freshness comparison. |
| `session.py` | Start, stop, status, re-attach. |
| `telemetry.py` | Tail and parse the per-arm CSV and controller log. |
| `plan.py` | `goal.yaml`, solve, validation report. |
| `runs.py` | Run history and summaries (exists today). |

Browser side: `index.html`, `panel.css`, `panel.js` (state and wiring),
`scene.js` (3D), `readouts.js` (fixed-slot numbers and bars). The
build-generated `dh_params_tool.yaml` is served so forward kinematics runs
in the browser.

### Read path

One way, and structurally incapable of touching control. The controller
writes its CSV as it always has; `telemetry.py` tails it and pushes frames
over Server-Sent Events.

- **Columns are read by name, never by index.** The format history in
  `Hardware.h` is explicit that index-based tooling breaks across format
  changes, and it has.
- **The log is 500 Hz; the panel samples at about 20.** Sampling alone would
  step over a 4 ms spike, so every frame carries both the newest row and, for
  the error and margin quantities, the worst value seen since the previous
  frame. That is how "current and maximum error" is answered honestly.
- **Staleness is computed in the browser from its own clock.** A frozen
  server cannot hide behind stale frames. The server also emits heartbeats
  when the file stops growing, so the page can distinguish "the panel lost
  the controller" from "I lost the panel". Both reach the 2 s break; they say
  different things.

### Write path

Three explicit actions, and nothing else.

1. **Set a config knob** — rewrites one matched line in `Config.h` and turns
   the freshness indicator stale.
2. **Build** — runs cmake and streams its output; freshness recomputed.
3. **Session start/stop** — see below.

## Session control

- **Start is localhost-only.** You must be at the workstation, beside the
  physical e-stop, to begin motion. Stop works from anywhere. Remote stop is
  always safe; remote start never is.
- **The session is launched detached with a pidfile.** If the panel crashes,
  the browser closes, or the network drops, the arm keeps doing what it was
  told rather than being halted abruptly by a cosmetic failure. The panel
  re-attaches through the pidfile and says that it has.
- **Stop signals `run_session.sh`'s process group** and lets its existing
  EXIT trap kill the controller — reusing the path `tests/test_run_session.sh`
  pins. `run_session.sh:159` records why this matters: with
  `> >(tee log)`, `$!` was the wrapper subshell, `kill -INT "$!"` never
  arrived, and the arm kept moving.
- **The typed GO gate moves into the browser.** Combined with localhost-only
  start you are still physically present, but the terminal's Ctrl-C is no
  longer in play. The panel therefore displays `kill -INT <pid>` for the live
  session so a terminal stop is always one paste away, and the stop POST works
  even if the page is stale and its JavaScript is dead.
- **Start refuses on a stale binary**, mirroring `fresh_or_die` rather than
  duplicating its judgement.
- Starting twice is already prevented below the panel by `ProcessLock.cpp`.

## The run screen

### Safety bar (fixed, top)

Arm selector, permanent HARDWARE mark, link indicator carrying the age of the
newest row, and **Stop and release the arm**. Link state comes from the SSE
connection, the row age by the browser clock, and `refresh_ok` together.
"Arm enabled" reads `arm_state`, so it means the arm really is in low-level
servoing. Beneath everything, permanently:

> Physical E-STOP is not monitored. It is your primary stop.

### Banner

State, arm, task, progress — as text, at size. Marked as **derived**, because
no state machine publishes these names; the panel infers them from the
session it launched and from `traj_activated` / `traj_complete`. States:
IDLE, PLANNING, READY, MOVING, HOLDING TARGET, STOPPING, FAULT. STOPPING is
brief by construction: there is no ramp, so it covers only the interval
between the stop request and the loop exiting.

"Current controller" is not inferred: the CSV preamble records the compiled
reference source, so the panel can say *pose-primary trajectory* because the
binary said so.

Progress comes from the plan artifact in the session directory, timed from
the activation edge — so it works whether the panel launched the session or
`run_session.sh` was run from a terminal. Trajectory identity is a panel-side session ID; the
controller does not echo an ID back, so the panel must not claim confirmation
it does not have. Making that real is a wire-format change and a separate
slice.

### Scene

- Both arms from `meas_j1..7` via DH forward kinematics in the browser, solid.
- The commanded arm from `cmd_j1..7`, ghosted in the desaturated blue.
  `cmd - meas` is precisely what the following-error guard tests, so this is
  the honest reading of "actual versus desired".
- Desired tool point from `pd_x..z`; dimension callout between it and the
  measured tool, with the number on the line.
- The plan as a polyline, split at the progress point into travelled and
  remaining.
- Obstacle box and SDF bounds when configured.
- Minimum-clearance point from `PathValidationReport` (`minimum_clearance_m`,
  `minimum_clearance_time_s`), labelled as a property of the **plan** —
  nothing measures clearance live.
- The frame is named on screen: mount frame; world equals mount; no Vicon.

### Error rows

**Every band on every bar is read from the compiled config and named by its
consequence** — arrival, replan advised, stop — and where nothing enforces a
threshold the bar says so instead of inventing a round number. The panel can
then never disagree with the binary, and changing a knob moves the bands.

Real thresholds, from `Config.h`:

| Quantity | Threshold | Meaning |
| --- | --- | --- |
| Position error | `kArrivalToleranceM` = 0.001 m | arrival |
| Position error | `kReplanPositionErrorM` = 0.05 m | replan advised |
| Orientation error | `kArrivalOrientationToleranceRad` = 0.001 | arrival only |
| Following error | `kFollowingErrorLimitDeg` = 3.0 | stop |
| Joint reference error | `kTrajFollowingErrorStopDeg` = 8.0 | stop |
| Splice distance | `kTrajStartToleranceDeg` = 2.0 | plan rejected |
| Limit margin | `kReplanJointMarginDeg` = 5.0 | replan advised |
| Posture error | `kReplanPostureErrorDeg` = 15.0 | replan advised |
| `sigma_min` | `kReplanSigmaMin` = 0.02 | replan advised |

Note: **no threshold stops the arm on orientation error.** The bar must not
imply one.

World-frame end-effector error is shown, labelled with its frame. Today it is
identical to base-frame error because `StaticBaseMotionSource` makes world
equal mount and `base_comp_m` is exactly zero. It becomes meaningful when
Vicon is wired, and labelling it now means the change will be visible.

### Joint bars

Angle, distance to nearest software limit, velocity from `vel_j*`, per-joint
following error from `cmd_j* - meas_j*`, warn and fault bands from
`kJointLimitWarnDeg` and `kJointLimitErrorDeg`.

**Only J2, J4 and J6 are bounded** — the warn array is
`{0, 130, 0, 145, 0, 118, 0}`. J1, J3, J5 and J7 are continuous and get angle
and velocity but no range bar, because drawing a limit they do not have is
the kind of thing one later believes.

No detailed per-joint graphs on the run screen; diagnostics open after
stopping.

## What does not exist, and must not be drawn

- **A controlled stop.** `SIGINT` sets `g_stop` (`Main.cpp:588`), the loop
  throws `TakeoverStop{kUserStop}` (`Runner.cpp:296`), and
  `servoing_guard.Restore()` hands the arm back. There is no velocity ramp;
  commanding ceases. Hence the button's label. A ramped stop is a worthwhile
  controller slice, and is deliberately not part of this design.
- **Physical e-stop state.** Nothing in the codebase reads it; the only
  mention is `Runner.cpp:124` telling Christian that he is the safety system.
  The panel shows a fixed statement plus a live arm-feedback indicator.
- **A centre cylinder or a human/backpack exclusion region.** The planner
  supports one optional axis-aligned box per arm (`goal.yaml:44`), currently
  commented out. Drawing an exclusion zone nothing avoids would be worse than
  drawing nothing.
- **Live clearance.** Clearance is computed at plan time only.
- **A controller-acknowledged trajectory ID.** No such field is on the wire.

## Failure behaviour

| Situation | Signal | What the panel says |
| --- | --- | --- |
| Telemetry ages past 2 s | browser clock | hard visual break; values marked not current |
| File stops growing | heartbeats still arriving | "the panel lost the controller" |
| SSE drops | connection state | "I lost the panel" |
| Controller exits | `PrintStopReport` in `controller.log` | FAULT, named: following error, robot fault, stale feedback, overrun |
| Trajectory rejected | `traj_rejected`, `traj_start_error_deg` | "the plan started 4.1 deg away from joint 3; the guard allows 2" |
| Solve fails | bridge stderr | plan-initialisation quality and goal-orientation warnings, which used to scroll past and be lost |
| Panel dies mid-run | pidfile | the controller keeps running; the panel re-attaches and says so |
| LAN drops on the MacBook | connection state | "you cannot stop from here — use the workstation" |
| Stale binary | mtime comparison | start refuses; offers a build |
| Invalid config value | validation before write | rejected with the reason; the file is unchanged |

Empty states are invitations: no session yet, no plan solved yet, no runs
yet, each with the one action that moves forward.

## Testing, without a robot

- **Replay mode** is the main harness: feed a recorded run from `runs/`
  through the tailer at wall-clock speed and the whole screen comes alive —
  scene, bars, banner, staleness — with no hardware. It doubles as a
  demonstration tool and lets the UI be built on the MacBook.
- The parser is tested against the real CSVs already in `runs/`.
- **Browser forward kinematics is cross-checked against
  `tools/print_dual_arm_fk`** over a set of configurations, so the rendered
  arm cannot drift from the C++.
- Config read/write runs against a copy of `Config.h`, asserting in
  particular that the regex never touches the guard overrides.
- **The session module is tested against a fake controller script that traps
  SIGINT and records it** — the exact failure `tests/test_run_session.sh`
  exists to prevent, and the one place in this design where being wrong
  reaches a moving arm.

Risk tiering: everything except `session.py` is ordinary code. `session.py`
gets one adversarial review before its button is pressed on hardware.

## Slices

1. **Split the panel; status pane.** Move HTML out of the Python string into
   `panel/`; add `/api/status` with build freshness, arm, session liveness,
   pipe existence. Fixes the stale-binary surprise. Touches nothing the robot
   runs.
2. **Config pane.** Widen the knob whitelist; show each knob against what the
   built binary contains; make staleness visible; stream build output.
3. **Targets pane.** Edit and validate `goal.yaml`; solve and preview; never
   send. Offline solving is possible today via `--start-deg J1..J7`
   (`BridgeMain.cpp:437`).
4. **Replay harness and the run screen.** Scene, error rows, joint bars,
   banner, staleness — developed entirely against recorded runs.
5. **Session pane.** Supervised start and stop, to the rules above.
   Adversarially reviewed; first use dry.

Slices 1-4 cannot move the arm.

## Deferred, with the reason

- A ramped stop in the controller (motion-path change; own slice).
- A controller-echoed trajectory ID (wire-format change).
- Runtime-settable non-safety config (waiting for a knob that proves it).
- Live Cartesian targets from the panel (needs the live-reference
  architecture; must not be smuggled in through a UI).
- Vendored three.js and real meshes (upgrade path for the scene).
- MuJoCo as a second `PlantBackend` (separate project).
