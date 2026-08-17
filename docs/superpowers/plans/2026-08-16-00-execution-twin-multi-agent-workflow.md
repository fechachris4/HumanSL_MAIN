# Execution Twin Multi-Agent Workflow Plan

> **For the coordinating agent:** REQUIRED SUB-SKILLS: use `superpowers:subagent-driven-development` for task execution and review; use `superpowers:dispatching-parallel-agents` only for independent read-only investigations. Do not dispatch implementation agents in parallel.

**Status:** Proposed orchestration plan. No implementation starts until Christian approves this workflow and the workspace-baseline decision in Gate 0.

**Goal:** Execute the approved HumanSL execution-twin design through multiple focused agents without allowing concurrent edits, hidden scope growth, hardware execution, or evidence created by the new implementation to become its only proof of correctness.

**Architecture:** One coordinating agent owns the dependency graph, safety constraints, progress ledger, integration, and user communication. A fresh implementation agent owns one numbered task at a time. A separate reviewer checks both specification compliance and engineering quality after every task. Independent evidence-gathering agents may run concurrently when they are read-only and operate on disjoint questions. The five existing implementation plans remain the detailed source of task requirements.

**Plan set:**

1. `2026-08-16-01-characterize-and-extract-execution-core.md`
2. `2026-08-16-02-dual-arm-mujoco-twin.md`
3. `2026-08-16-03-realistic-vicon-and-scenarios.md`
4. `2026-08-16-04-full-planner-simulation-handoff.md`
5. `2026-08-16-05-panel-and-acceptance.md`

## Non-negotiable constraints

- Preserve the pose/twist-only boundary. Planned joint posture, `q_ref`, `qdot_ref`, elbow targets, and planner null-space bias do not enter the controller.
- The shared execution core is a Level-2 refactor of the hardware controller. Characterization evidence must exist before extraction and replay must pass afterward.
- No robot-facing executable may be run. Hardware revalidation is a later supervised activity described by a runbook only.
- `humansl_sim` must be structurally incapable of linking Kortex or selecting hardware at runtime.
- GPMP2 remains external to the 500 Hz loop and retains its joint-space internals.
- World pose and twist, including `world_T_mount`, remain explicit in planner and controller frame conversions.
- Realistic Vicon emits pose at 100 Hz; the production estimator derives Mount twist only for advancing samples; the 500 Hz controller uses coherent zero-order-held state.
- Both arms share one simulation tick and paired planner results activate atomically.
- MuJoCo receives the same integrated position requests as hardware, but its actuator dynamics remain a generic plant.
- Interactive visual testing and plots are the normal development workflow. The comprehensive deterministic acceptance matrix runs once at the end.
- Preserve unrelated dirty-tree work. No commit, push, or installation without Christian's explicit authorization.

## Why this uses multiple agents but not concurrent writers

The work spans real-time control, kinematics/model provenance, MuJoCo, Vicon timing, planner IPC, panel process ownership, and scientific evidence. Those areas benefit from fresh specialist context and independent review.

They do not all benefit from concurrent implementation. The current feature branch contains substantial uncommitted work, and the plans intentionally build on one another. Two agents editing shared CMake files, controller interfaces, or the simulation runner at once would create ambiguous ownership and make review evidence unreliable. Therefore:

- Read-only audits may fan out in parallel when their questions and outputs are independent.
- Exactly one agent may modify the production workspace at a time.
- Review agents are read-only.
- The coordinator never makes an unreviewed fix; findings return to the task implementer.
- A later wave does not start until the previous wave's gate is accepted.

## Agent roles

### Coordinating agent

- Reads the approved design, this workflow, the active phase plan, `AGENTS.md`, and `docs/intent/story.md`.
- Verifies the workspace and creates the per-plan progress ledger.
- Produces one self-contained task brief per numbered task.
- Dispatches agents, records their identities, and preserves reports outside conversational memory.
- Resolves cross-task interfaces and runs independent integration verification.
- Stops on a load-bearing conflict, unsafe action, or requirement ambiguity.
- Does not write task fixes itself.

### Read-only investigation agent

- Receives one narrow evidence question and exact paths.
- May inspect source, logs, build metadata, models, or tests.
- Does not edit, build hardware targets for execution, install dependencies, or change repository state.
- Returns evidence with file/symbol/command references and clearly labels unknowns.

### Task implementation agent

- Receives exactly one numbered task brief, earlier interface decisions needed by that task, allowed paths, and global constraints.
- Reads every file before editing it and follows test-first steps from the task plan.
- Uses existing installed dependencies and does not install anything.
- Runs only the task's hardware-free tests.
- Self-reviews the diff and writes a durable task report containing commands and outputs.
- Does not broaden scope, commit, push, or operate hardware.

### Task review agent

- Receives the task brief, implementer report, complete task diff, and binding constraints.
- Gives two explicit verdicts: specification compliance and engineering quality.
- Checks real-time safety, frame/unit/timestamp clarity, boundary preservation, test independence, and unintended behaviour changes where relevant.
- Does not edit files or merely repeat the implementer's tests.

### Final red-team agent

- Reviews the complete integrated diff and the progress ledger using the most capable available model.
- Attempts to show how the implementation could be fundamentally wrong while its new tests still pass.
- Checks the full call path, Kortex-link exclusion, paired-arm atomicity, deterministic evidence provenance, and hardware-revalidation boundary.
- Produces one consolidated finding set for at most one final fix wave.

## Gate 0: Workspace and baseline decision

The current branch is `codex/world-cartesian-controller`, and the execution-twin plans depend on substantial uncommitted controller, planner, Vicon, panel, documentation, and test changes. A new worktree created from the current Git `HEAD` would omit those changes.

Before implementation agents are dispatched, Christian chooses one of these execution baselines:

1. **Checkpoint baseline — recommended.** Authorize one explicit checkpoint commit containing the already-approved prerequisite work and planning documents, then create an isolated execution-twin worktree/branch from it. This gives every task and review an exact Git base without changing the intended code.
2. **Current dirty feature workspace.** Explicitly authorize execution in the present feature workspace. The coordinator enforces one writer at a time and captures per-task patches and SHA-256 snapshots because commit-range review is unavailable.

The coordinator must then:

- record branch, `HEAD`, `git status --short`, dependency versions, source/config/model hashes, and selected baseline mode;
- identify which dirty files are prerequisites versus unrelated user work;
- create a separate progress ledger for each of the five phase plans;
- scan all five plans once for contradictions before Task 01.1;
- stop if any planned edit would overwrite an unidentified user change.

**Gate 0 passes when:** the baseline is explicit, recoverable, and review packages can represent every task's complete change.

## Per-task agent loop

Every numbered task in Plans 01–05 follows the same loop:

1. The coordinator records the task base and generates a brief from the task plan.
2. One fresh implementation agent edits and tests only that task.
3. The coordinator checks the returned report for changed files, test commands, outputs, assumptions, and concerns.
4. One fresh task reviewer inspects the brief, report, and complete task diff.
5. Any Critical/Important finding or confirmed specification gap returns to the same implementer for a scoped fix.
6. A fresh scoped re-review verifies only the findings and fix diff.
7. After at most five fix rounds, a real load-bearing residual blocks the workflow and is presented to Christian. Minor or contestable findings are recorded with explicit rulings, never silently discarded.
8. The coordinator runs the independent gate command(s), updates the ledger, and only then releases the next task.

The implementer and reviewer must be different agents. Self-review is required but never substitutes for independent review.

## Wave 0: Parallel read-only reconnaissance

Three read-only agents run concurrently before Plan 01 changes code:

| Agent | Independent question | Required output |
|---|---|---|
| W0-A: controller evidence | What pre-existing logs, tests, and call paths can characterize current configured execution behaviour without inventing missing Cartesian fields? | Evidence inventory, usable fields by log schema, gaps, and proposed replay inputs. |
| W0-B: build/dependency | What compilers, CMake targets, Eigen, Pinocchio, MuJoCo, GPMP2/GTSAM, GLFW, and test runners are already available? | Version/path matrix and hardware-free build commands; no installation. |
| W0-C: model/frame provenance | Which URDF/MJCF assets, joint order, Mount transforms, base frames, flange/tool/TCP transforms, and units are authoritative? | Frame/model provenance table and conflicts requiring resolution. |

The coordinator reconciles the three reports. They are evidence inputs to Plan 01 Task 1 and Plan 02 Task 1, not permission to skip those tasks.

## Wave 1: Characterize and extract the execution core

Execute Plan 01 Tasks 1–5 sequentially with one implementer and one reviewer per task.

Dependency path:

```text
01.1 freeze evidence
  -> 01.2 immutable configuration
  -> 01.3 explicit execution contract
  -> 01.4 reusable core + hardware runner wiring
  -> 01.5 offline-only status and revalidation runbook
```

Review emphasis:

- 01.1: independent pre-change evidence and source/config hashes.
- 01.2: no live direct configuration reads remain in the cycle path.
- 01.3: pose/twist-only input and unchanged controller/safety ordering.
- 01.4: replay equivalence, real-time restrictions, and unchanged Kortex ownership.
- 01.5: no claim of physical equivalence before supervised hardware revalidation.

**Gate 1 passes when:** pre-extraction traces and post-extraction replay agree within the frozen tolerances; hardware-free tests pass; the hardware target builds but is not run; and the shared core has no Kortex dependency.

Christian reviews Gate 1 because it is the highest-risk hardware-controller refactor boundary.

## Wave 2: Build the exact dual-arm MuJoCo twin

Execute Plan 02 Tasks 1–5 sequentially. W0-B and W0-C reports seed Task 02.1, but the task implementer independently verifies them.

Dependency path:

```text
02.1 provenance
  -> 02.2 exact-frame dual-arm MJCF
  -> 02.3 MuJoCo/Pinocchio parity
  -> 02.4 command/feedback adapter
  -> 02.5 dual-arm coordinator + humansl_sim
```

Review emphasis:

- Exact right/left joint order, axes, limits, `mount_T_base`, and configured TCPs.
- MuJoCo/Pinocchio forward-kinematics and Jacobian parity across nontrivial sampled states.
- Exactly 2 ms control ticks with deterministic physics substeps.
- Integrated position requests reach generic MuJoCo position actuators through a simulation adapter.
- Link and symbol evidence proves `humansl_sim` cannot load Kortex.

**Gate 2 passes when:** exact model/frame parity tests pass, both arms execute one shared-core tick coherently, a headless deterministic run succeeds, and Kortex exclusion is proven independently from source assertions.

## Wave 3: Add realistic Vicon and scenarios

Execute Plan 03 Tasks 1–5 sequentially. Before Tasks 03.1 and 03.2, two read-only agents may independently review the analytic motion equations and production estimator contract in parallel. Their findings go to the implementers; they do not edit.

Dependency path:

```text
03.1 scripted Mount truth ----+
                               -> 03.3 runner integration
03.2 100 Hz Vicon emulator ---+

03.4 shared scene -> 03.5 executed clearance and infeasible hold
03.3 runner integration ------^
```

Although Tasks 03.1 and 03.2 are logically independent, they both touch build/config surfaces; implementation remains one writer at a time.

Review emphasis:

- Analytic SE(3) truth pose/twist consistency.
- Exactly one derivative update per advancing 100 Hz Vicon sample.
- No direct MuJoCo truth velocity in realistic mode.
- Coherent pose/twist/sequence/timestamp/age zero-order hold at 500 Hz.
- Raw and filtered estimator outputs remain visible for plots.
- Planned and executed clearance are clearly separate claims.
- Infeasible holds eventually use the existing joint-boundary full-frame hold and warning, not an invented escape posture.

**Gate 3 passes when:** identical seeds reproduce sensor/scenario traces, ideal and realistic modes are explicitly switchable, timing tests prove 100/500 Hz behaviour, and executed-clearance monitoring follows actual MuJoCo state.

## Wave 4: Connect the full external planner pipeline

Execute Plan 04 Tasks 1–5 sequentially. The request/result contracts are frozen by Task 04.1 before any process or activation implementation begins.

Dependency path:

```text
04.1 versioned paired contracts
  -> 04.2 coherent dual request
  -> 04.3 paired external planner result
  -> 04.4 atomic two-arm activation
  -> 04.5 live and deterministic planner modes
```

Review emphasis:

- GPMP2 remains unchanged internally and outside the control process.
- The controller receives only timed world TCP pose/twist and provenance.
- Both arms derive from one coherent Mount snapshot.
- Both references activate on one common cycle or neither activates.
- A rejected replacement cannot disturb the active valid reference.
- Latest-wins coalescing is bounded and non-blocking for the 500 Hz loop.
- Planned collision/inter-arm clearance is labelled as applying only to GPMP2's internal joint branch.
- Deterministic acceptance injects fixed planner outputs; live asynchronous solve timing is tested only as a contract and interactive behaviour.

**Gate 4 passes when:** paired round-trip, coherent request, failure preservation, all-or-neither activation, coalescing, and full live-process contract tests pass without placing planner/file/pipe work in the control thread.

Christian reviews Gate 4 because it freezes the planner/controller boundary and dual-arm activation semantics.

## Wave 5: Panel, telemetry, plots, and final evidence

Execute Plan 05 Tasks 1–6 sequentially. After Task 05.4 freezes telemetry names, independent read-only agents may review plot completeness and acceptance evidence design in parallel before Tasks 05.5 and 05.6.

Dependency path:

```text
05.1 simulation process ownership
  -> 05.2 parity/experiment manifests
  -> 05.3 panel controls
  -> 05.4 telemetry schema
  -> 05.5 plots
  -> 05.6 final acceptance matrix
```

Review emphasis:

- Hardware and simulation sessions are mutually exclusive.
- Hardware-parity mode locks production values and displays `offline-validated only`.
- Experiment overrides are simulation-only, explicit, and fully recorded.
- Shared telemetry retains hardware semantics; simulator truth uses `sim_` names.
- Plots expose raw and filtered 100 Hz Vicon twist, ZOH stepping, world Cartesian error, command limiting/integration, actuator response, joint margins, replans, clearance/contact, and timing.
- Acceptance thresholds are frozen before reading the C++ acceptance result.
- Python comparison is permitted only for identical plant/scenario hashes.
- First failing evidence is retained and investigated; the suite is not repeatedly tuned until green.

**Gate 5 passes when:** panel visual testing works for both modes, plots are generated from recorded runs, all hardware-free builds/tests pass, deterministic acceptance completes once against pre-frozen thresholds, and the hardware revalidation runbook remains unexecuted.

## Final whole-system review

After Gate 5, dispatch one final red-team reviewer over the complete implementation range and all five ledgers. The reviewer must answer:

1. Can the hardware and simulation paths diverge before the intended adapter seam?
2. Can `humansl_sim` reach Kortex through direct or transitive linkage?
3. Can planned joint posture influence the controller despite the pose/twist-only contract?
4. Can stale or repeated Vicon samples create false velocity updates?
5. Can one arm activate a new plan without the other?
6. Can planner latency or C++ output influence acceptance thresholds?
7. Can a test pass while using inconsistent frames, TCPs, units, hashes, or timestamps?
8. Are planned-path and executed-path clearance claims kept distinct?
9. Does every hardware-parity claim still say `offline-validated only`?

There is at most one consolidated final fix wave, followed by one scoped re-review. Any remaining load-bearing finding blocks completion and is reported rather than explained away.

## Progress and evidence records

Each phase gets its own ignored Subagent-Driven Development workspace and ledger. Durable project evidence belongs in the run directories and documentation required by the five plans; agent chatter does not become architecture.

For every task, record:

- task brief and exact base snapshot;
- implementer identity/model and report;
- files changed;
- test commands and exact outcomes;
- review verdicts and fix rounds;
- hashes/provenance where the task produces scientific evidence;
- any deferred minor with an explicit ruling.

The coordinator may mark a task complete only after its review gate is clean or the formal fix-round breaker has been handled. A plan may be marked complete only after its phase gate passes.

## Human approval checkpoints

- **Before implementation:** choose and authorize the Gate 0 baseline.
- **After Gate 1:** approve the characterized execution-core extraction before building simulation on it.
- **After Gate 2:** visually inspect exact dual-arm model/frame/TCP behaviour.
- **After Gate 4:** approve the final paired planner/controller boundary.
- **After Gate 5:** review plots and acceptance evidence.
- **Before any future robot run:** separately authorize and supervise the hardware revalidation runbook. This workflow never implies that authorization.

## Completion condition

The workflow is complete only when all 26 numbered tasks, all five phase gates, and the final red-team review are complete; the simulation remains Kortex-free; the controller boundary remains world pose/twist only; deterministic evidence and interactive plots are available; and no physical-hardware equivalence claim is made without later supervised revalidation.
