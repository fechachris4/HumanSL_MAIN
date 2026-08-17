# Simulation Panel and Final Acceptance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the complete execution twin into the HumanSL control panel, add explicit parity/experiment operation, produce simulation plots and provenance, and run the comprehensive hardware-free acceptance suite once at the end.

**Architecture:** A separate panel simulation session module launches only `humansl_sim` plus `planner_bridge` in one process group and is mutually exclusive with hardware sessions. Configuration is materialized as an immutable per-run manifest. Hardware-parity mode locks production values; experiment mode writes simulation-only overrides. The simulator emits the shared telemetry schema plus a namespaced simulation extension, and the panel generates plots and final acceptance reports from recorded runs.

**Tech Stack:** Python 3 standard library panel server/tests, HTML/CSS/JavaScript panel UI, C++ simulation binaries, CSV/JSON/YAML run manifests, Matplotlib plotting scripts, CMake/CTest/unittest.

## Global Constraints

- Plans 01–04 must be accepted first.
- `docs/engineering/humansl-engineering-contract.md` is binding for every task in this plan (evidence-level labels in §13, measured/estimated/referenced/commanded naming in §4, diagnostics classification in §10).
- Panel simulation and hardware sessions are mutually exclusive.
- `humansl_sim` remains incapable of Kortex linkage regardless of panel state.
- Hardware-parity means production code/configuration parity and displays `offline-validated only` until supervised hardware revalidation.
- Experiment overrides never silently edit `Config.h`, planner production YAML, or hardware configuration.
- Every override, model/scene hash, planner block hash, seed, sensor setting, physics substep, and actuator parameter is recorded.
- Shared telemetry names retain hardware semantics; MuJoCo truth/contact fields use an explicit `sim_` namespace.
- Interactive viewer and plots are the normal workflow. The complete automated acceptance matrix runs at the end, not after every edit.
- Numeric thresholds never depend on live planner solve timing or a mismatched Python plant.
- No hardware executable, commit, push, or robot operation without explicit authorization.

---

### Task 1: Add a separate, mutually exclusive simulation session backend

**Files:**
- Create: `Christian_control/tools/panel/sim_session.py`
- Create: `Christian_control/tools/panel/tests/test_sim_session.py`
- Create: `Christian_control/simulation/scripts/run_sim_session.sh`
- Modify: `Christian_control/tools/panel/session.py`
- Modify: `Christian_control/tools/panel/server.py`

**Interfaces:**
- Produces:

```python
def start_simulation(config_path: str) -> dict: ...
def stop_simulation() -> dict: ...
def simulation_status() -> dict: ...
def pause_simulation() -> dict: ...
def reset_simulation() -> dict: ...
```

- [ ] **Step 1: Write process ownership/exclusion tests**

Using fake scripts, assert simulation start is refused while a hardware session state/PGID is live; hardware start is refused while simulation is live; simulation launches `humansl_sim` and `planner_bridge` only; stop signals the whole process group; panel restart reattaches; dead/stale state is reported and cleared without signalling unrelated PIDs.

- [ ] **Step 2: Run and observe missing simulation module**

Run: `python3 -m unittest discover Christian_control/tools/panel/tests -p 'test_sim_session.py' -v`

- [ ] **Step 3: Implement the simulation session state machine**

Use a separate state file with executable paths, PGID, start time, mode, manifest, log paths, and viewer PID. Validate the built binaries and config before process creation. Never import or call hardware connection code. The panel currently hard-codes the absence of a simulator in two places that this work must update deliberately: `server.py:467` (`"hardware": True,  # there is no simulator in this repository`) and the permanent `HARDWARE` badge in `static/index.html:31-34` whose comment gives the same reason — replace both with the real hardware/simulation session distinction rather than leaving stale assertions.

- [ ] **Step 4: Implement the launcher script**

Create temporary FIFOs under the run directory; launch `planner_bridge` and `humansl_sim` in one process group; forward SIGINT; wait for both; preserve logs/manifests on failure; never invoke `controller`.

- [ ] **Step 5: Run session and existing hardware-session tests**

Run: `python3 -m unittest discover Christian_control/tools/panel/tests -p 'test_*session*.py' -v`

Review checkpoint: inspect every subprocess command and prove no simulation API can select a robot IP or hardware binary.

### Task 2: Implement hardware-parity and experiment run manifests

**Files:**
- Create: `Christian_control/tools/panel/sim_config.py`
- Create: `Christian_control/tools/panel/tests/test_sim_config.py`
- Create: `Christian_control/simulation/config/simulation_defaults.yaml`
- Modify: `Christian_control/simulation/src/SimulationConfig.h`
- Modify: `Christian_control/simulation/src/SimMain.cpp`
- Modify: `Christian_control/tools/panel/server.py`

**Interfaces:**
- Produces one immutable JSON run manifest:

```json
{
  "mode": "hardware_parity",
  "hardware_refactor_validation": "offline-validated only",
  "execution_config": {},
  "simulation": {},
  "vicon": {},
  "mount_motion": {},
  "scene": {},
  "provenance": {}
}
```

- [ ] **Step 1: Write parity-lock and override tests**

Assert parity mode exactly matches `ProductionExecutionConfig()` and rejects controller/filter/limit overrides. Experiment mode begins from production values, permits only whitelisted simulation/controller/filter fields, records original and override values, rejects guard weakening not explicitly included by the spec, and never writes `Config.h` or production planner YAML.

- [ ] **Step 2: Run and observe missing config module**

Run: `python3 Christian_control/tools/panel/tests/test_sim_config.py -v`

- [ ] **Step 3: Implement manifest materialization**

Read production values through the existing panel config parser, resolve canonical model/scene/planner paths, hash every input, and write a new manifest into the run directory using create-new semantics. The simulator validates schema/version/hash before startup.

- [ ] **Step 4: Add explicit offline-validation labelling**

Both modes include the current hardware-refactor validation state. Parity mode is never displayed simply as `validated` before the separate runbook is completed.

- [ ] **Step 5: Run config and existing panel configuration tests**

Run: `python3 -m unittest discover Christian_control/tools/panel/tests -p 'test_*config*.py' -v`

Review checkpoint: diff `Config.h` and production YAML before/after experiment-manifest tests; bytes must be identical.

### Task 3: Add the Simulation panel surface

**Files:**
- Modify: `Christian_control/tools/panel/static/index.html`
- Modify: `Christian_control/tools/panel/static/panel.css`
- Modify: `Christian_control/tools/panel/static/panel.js`
- Modify: `Christian_control/tools/panel/server.py`
- Create: `Christian_control/tools/panel/tests/test_sim_api.py`
- Create: `Christian_control/tools/panel/tests/test_sim_panel_contract.py`

**Interfaces:**
- Adds `/api/sim/status`, `/api/sim/config`, `/api/sim/start`, `/api/sim/pause`, `/api/sim/reset`, `/api/sim/stop`, and `/api/sim/goal`.

- [ ] **Step 1: Write API validation tests**

Cover ideal/realistic Vicon, four scripted motion kinds, amplitudes/frequencies/axes/phases, noise/latency/dropout/seed, physics substeps, free-space/box scene, parity/experiment mode, right/left/paired goals, pause/reset/stop, invalid values, and hardware-session exclusion.

- [ ] **Step 2: Write static UI contract tests**

Require visible mode label, `offline-validated only` warning, viewer state, both-arm state, planned-versus-executed clearance labels, start/pause/reset/stop controls, goal forms, Vicon/scenario controls, obstacle toggle default off, and disabled experiment inputs in parity mode.

- [ ] **Step 3: Implement routes with no duplicated policy**

Routes call `sim_config` and `sim_session`; they do not construct shell strings or reinterpret controller values. Return structured errors with failing field/process and preserve current safe panel headers/path checks.

- [ ] **Step 4: Implement the Simulation view**

Reuse existing visual language and goal forms. Keep hardware RUN controls visually distinct. Reset requires confirmation that active plans will be cleared; start shows exact mode/model/Vicon/scenario summary.

- [ ] **Step 5: Run all panel tests**

Run: `python3 -m unittest discover Christian_control/tools/panel/tests -v`

Review checkpoint: manually use the panel against fake processes first, then a headless simulator; ensure a hardware run button cannot be activated while simulation state is live.

### Task 4: Record shared telemetry plus simulation-only truth

**Files:**
- Create: `Christian_control/simulation/src/SimTelemetry.h`
- Create: `Christian_control/simulation/src/SimTelemetry.cpp`
- Create: `Christian_control/simulation/tests/test_sim_telemetry.cpp`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.cpp`
- Modify: `Christian_control/simulation/src/SimMain.cpp`
- Modify: `Christian_control/basic_control/scripts/runlog.py`
- Modify: `Christian_control/basic_control/tests/test_runlog_compat.py`

**Interfaces:**
- Shared columns retain existing semantic names. Simulation extension columns begin `sim_`, including truth Mount pose/twist, true TCP pose/twist, actuator target, MuJoCo state, executed clearance/contact, physics substeps, and sensor error.

- [ ] **Step 1: Write schema/name/provenance tests**

Assert every shared reference/controller/limit/integration field has the same unit and meaning as hardware; simulation truth never appears under `measured_` or `vicon_`; all sim-only names start `sim_`; run preamble contains manifest hash and model/actuator/substep settings.

- [ ] **Step 2: Run and observe missing telemetry writer**

Run: `cmake --build Christian_control/simulation/build --target test_sim_telemetry -j2`

- [ ] **Step 3: Implement asynchronous/buffered writing outside control**

The control owner publishes fixed-size rows to the existing bounded logging pattern. File formatting/writing runs outside the simulation control step. On buffer overflow, increment and report a loss counter rather than blocking physics/control.

- [ ] **Step 4: Extend `runlog.py` by schema detection**

Keep formats 2–13 readable. Detect simulation extension from preamble/column names; never reinterpret historical columns. Expose shared and sim-only groups separately.

- [ ] **Step 5: Run telemetry and compatibility suites**

Run: `ctest --test-dir Christian_control/simulation/build -R '^sim_telemetry$' --output-on-failure`

Run: `ctest --test-dir Christian_control/basic_control/build -R '^log_schema$' --output-on-failure`

Run: `python3 Christian_control/basic_control/tests/test_runlog_compat.py -v`

Review checkpoint: compare one hardware synthetic row and simulation row field-by-field; shared values have identical definitions, while truth/contact fields remain visibly separate.

### Task 5: Add post-run plots and summaries

**Files:**
- Create: `Christian_control/basic_control/scripts/plot_execution_twin.py`
- Create: `Christian_control/basic_control/tests/test_plot_execution_twin.py`
- Modify: `Christian_control/tools/panel/plots.py`
- Modify: `Christian_control/tools/panel/static/index.html`
- Modify: `Christian_control/tools/panel/static/panel.js`

**Interfaces:**
- Produces per-run figures and JSON summary for world pose/orientation error, reference/measured/truth twist, raw/filtered Mount twist and age, raw/limited joint velocity, integrated command/simulated response, saturation, joint margin, planned/executed clearance, contact, reference/replan events, and timing.

- [ ] **Step 1: Write synthetic-log plot tests**

Generate a short log with 100 Hz sequence steps, raw/filtered twist difference, saturation, trajectory replacement, planned/executed clearance divergence, and one infeasible-hold stop. Assert expected plot files and exact summary values; reject missing units/provenance.

- [ ] **Step 2: Run and observe missing plotter**

Run: `python3 Christian_control/basic_control/tests/test_plot_execution_twin.py -v`

- [ ] **Step 3: Implement plots without hiding sensor stepping**

Plot raw and filtered Mount twist as separate series with Vicon update markers. Plot planner-path clearance and executed clearance with different labels and explanatory caption. Mark sim mode and `offline-validated only` in titles/summary.

- [ ] **Step 4: Register plotter in panel**

Use the existing allowlisted plot runner; do not accept arbitrary script/path input. Show generated images and textual metrics in the selected run view.

- [ ] **Step 5: Run plot and panel suites**

Run: `python3 Christian_control/basic_control/tests/test_plot_execution_twin.py -v`

Run: `python3 -m unittest discover Christian_control/tools/panel/tests -v`

Review checkpoint: inspect one ideal and one realistic run at narrow/desktop panel widths; plots must not imply simulated RMSE predicts hardware performance.

### Task 6: Build and run the final acceptance matrix

**Files:**
- Create: `Christian_control/simulation/acceptance/scenarios.yaml`
- Create: `Christian_control/simulation/acceptance/prepare_thresholds.py`
- Create: `Christian_control/simulation/acceptance/run_acceptance.py`
- Create: `Christian_control/simulation/acceptance/compare_python_trace.py`
- Create: `Christian_control/simulation/tests/test_acceptance_harness.py`
- Create: `Christian_control/simulation/acceptance/README.md`
- Create at run time: a unique directory under `runs/simulation/` containing `acceptance-thresholds.json` and `acceptance-report.json`; `prepare_thresholds.py` creates the directory and prints its exact absolute path.

**Interfaces:**
- `prepare_thresholds.py` runs/reads the independent Python baseline before any C++ result, verifies identical plant/scenario hashes, and freezes exact numeric limits.
- `prepare_thresholds.py` atomically writes `runs/simulation/latest-prepared-run.txt` containing the unique directory it created.
- `run_acceptance.py` accepts `--continue-latest-prepared runs/simulation`, resolves that exact directory, refuses missing/stale/postdated thresholds, and runs deterministic scripted planner outputs.

- [ ] **Step 1: Write harness integrity tests**

Reject thresholds created after C++ results, mismatched model/actuator/substep/initial/reference/sensor/scenario/seed hashes, live-planner numeric cases, missing negative controls, and reports that call planned clearance executed/safe.

- [ ] **Step 2: Define the fixed scenario matrix**

Include both arms under static, translation, rotation, and combined Mount motion; ideal and realistic Vicon; static and optional box scene; mid-run scripted replacement; Vicon dropout/recovery; invalid result; non-finite injection; following error; joint-boundary/infeasible hold; executed contact; frozen-joint baseline; measured-Mount-twist-disabled negative control.

- [ ] **Step 3: Freeze thresholds before C++ acceptance**

For model-identical Python/C++ cases, record Python metrics and explicit comparison limits in `acceptance-thresholds.json`. If the Python model or actuator hash differs, mark that case `qualitative_only` and define no numeric Python-derived threshold. Record all decisions before running C++ scenarios. Be aware the "model-identical" precondition will rarely hold as things stand: msc_project's scene uses different mounting geometry (tilt 0.845708 rad, torso-relative offsets) than HumanSL's authoritative 1.2085 rad / ±0.0567075 m, and its own actuator gains — so either deliberately build comparison cases where the Python scene/actuators are aligned to the twin, or accept that most cases are `qualitative_only` and say so up front rather than discovering it here.

- [ ] **Step 4: Run fresh builds and all hardware-free tests**

Run: `cmake --build Christian_control/basic_control/build -j2 && ctest --test-dir Christian_control/basic_control/build --output-on-failure`

Run: `cmake --build Christian_control/simulation/build -j2 && ctest --test-dir Christian_control/simulation/build --output-on-failure`

Run: `cmake --build Christian_control/planner_bridge/build -j2 && ctest --test-dir Christian_control/planner_bridge/build --output-on-failure`

Run: `python3 -m unittest discover Christian_control/tools/panel/tests -v`

- [ ] **Step 5: Run deterministic acceptance once**

Run: `python3 Christian_control/simulation/acceptance/prepare_thresholds.py --scenarios Christian_control/simulation/acceptance/scenarios.yaml --python-project /home/christian/msc_project --output-root runs/simulation`

Run: `python3 Christian_control/simulation/acceptance/run_acceptance.py --build Christian_control/simulation/build --scenarios Christian_control/simulation/acceptance/scenarios.yaml --continue-latest-prepared runs/simulation`

Expected: every scenario records pass/fail with provenance; repeated deterministic scenarios have identical event/stop traces; qualitative-only cases are never counted as numeric passes.

- [ ] **Step 6: Review evidence rather than rerunning until green**

If a case fails, preserve the first report and diagnose the cause. Any changed model, code, scenario, seed, or threshold creates a new acceptance run directory and provenance chain; never overwrite or backdate thresholds.

- [ ] **Step 7: Validate the hardware revalidation boundary**

Confirm `Christian_control/docs/runbooks/execution-core-hardware-revalidation.md` exists and remains unexecuted. Report the simulator and refactored hardware binary as `offline-validated only`; do not launch `controller`.

Final review checkpoint: inspect code/spec coverage, all test outputs, target linkage, first-failure evidence, plots, run manifests, and acceptance report. State explicitly what simulation proved and what requires supervised hardware validation.
