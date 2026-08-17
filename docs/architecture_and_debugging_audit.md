# Architecture and debugging audit

## 2026-08-16 current-architecture addendum

The detailed 2026-08-14 audit below is preserved as historical evidence, but
its controller topology is superseded. In particular, do not use its sections
on the joint-trajectory follower, idle `WorldHold` dispatch, `TRAJ_BEGIN`
boundary, format-11 telemetry, or one-shot session bridge to explain the current
runtime.

The current production path is:

```text
fresh Vicon T_W_M + measured q/qdot
  -> one coherent measured T_W_E, J_W, V_W_E
  -> world pose/twist reference (hold or CART_TRAJ)
  -> one Cartesian PD resolved-rate law
  -> raw qdot
  -> shared velocity/joint-boundary/integration/safety path
  -> Kortex POSITION command
```

Evidence map for the replacement:

| Question | Current authority |
| --- | --- |
| What crosses from GPMP2? | `cartesian_contract/WorldCartesianTrajectory.h`: timed WORLD pose/twist, IDs, provenance; no joints |
| Where are GPMP2 joints converted? | `planner_bridge/src/WorldTrajectoryProjection.cpp`, after solve/validation and outside GPMP2 internals |
| Where does `T_W_M` enter planning? | `PlannerWorker.cpp` request conversion and `PlannerModel.cpp` world-root construction |
| Where does base motion enter control? | `ViconSource.cpp` filtered Mount twist, then `Frames.h::ArmControllerState` rigid-body transport |
| What controller runs? | `basic_control/src/Controller.cpp::DesiredVelocity`, the sole world Cartesian pose/twist law |
| How are references selected? | `CartesianReference.cpp`: startup world hold, validated trajectory interpolation, final hold, dropout states |
| How is replanning triggered? | fixed-size `PlanningRequestSlot` in `Runner.cpp`; FIFO serialization is in `PlanningRequestWriter.cpp` |
| How is planning kept out of 500 Hz? | `PlannerWorker.cpp`: one sequential solve thread per arm plus a latest-wins reader mailbox |
| What reaches hardware? | `Runner.cpp` sends every raw result through the existing clamp, `PositionIntegration`, stop arbitration, and `CyclicSession::Send` |
| What proves the cycle? | format-13 CSV: Vicon provenance, world reference/measurement, trajectory/replan edges, raw/clamped/integrated commands, feedback/fault/timing |

Debug the current path in that same order. First verify Vicon age/validity and
`T_W_M`; then compare measured and reference world pose/twist; then inspect task
and null-space `qdot`; finally distinguish requested, integrated-command, and
measured joints. A planner issue cannot be inferred from Cartesian error alone:
match `cart_trajectory_id` and `cart_planner_vicon_sequence` to the saved
`request_<arm>_<id>.planreq` and `plan_<arm>_<id>.carttraj` artifacts first.

Dropout interpretation is now explicit. Age above 50 ms pauses trajectory time
and decays retained Mount twist. Recovery before 200 ms resumes without
wall-clock catch-up. At 200 ms, cancellation occurs once; fresh recovery captures
the measured world pose, rejects pre-gap provenance, and emits one replan edge.

This addendum is source-verified and hardware-free. It does not prove marker
cluster calibration, collision clearance, Vicon latency under load, physical
world-frame stability, or safe motion around a person. No robot-facing command
was executed for this migration.

## Historical audit: 2026-08-14

The following was a current-state audit on 2026-08-14, on branch `master` at HEAD `02348ecc`
("controller: world hold engages through the idle joint hold") with a dirty
working tree. This historical section describes the system as it existed on
that date; the
2026-08-07 audit (`docs/codebase_cleanup_audit.md`, HEAD `f23719ad`) is
preserved unchanged as historical evidence and is superseded by §5 of this
file. The published "Gen3 Command Path" artifact was updated from this audit
on the same date.

Method: read-only. Four parallel source reads (runtime path, gate/limit
inventory, observability surface, build boundaries + old-audit delta), each
claim cited to file:line, load-bearing citations re-verified by the lead
against the working tree. No binary was executed; no robot, controller, or
Vicon acquisition was run.

**Evidence labels used throughout.** [code] = verified by reading the cited
source line in the current tree. [tree] = file-system or git observation
(mtimes, build dirs, commits). [doc] = a repository document's claim, not
independently re-verified. [hw] = would require a physical run — nothing in
this audit carries hardware proof, and no claim here should be read as one.

**Binary caveat** [tree]: the on-disk `controller` binaries are stale.
`basic_control/cmake-build-debug/controller` is dated 2026-08-11 and
predates the entire world-hold feature; `basic_control/build/` was
relinked 2026-08-13 19:48, before the final `Config.h` edits (19:43 is the
Config mtime but the dirty velocity-limit values are unverified in any
binary). Every "reachable" claim below means *reachable in a rebuild of the
current tree*, and any hardware run must rebuild first.

**Dirty-tree summary** [tree]: uncommitted changes are the panel PLOTS tab
(`tools/panel/plots.py` + server/static wiring), the world-hold plot script
(`basic_control/scripts/plot_world_hold.py`), a `runlog.py` comment fix,
the velocity-limit raise to 76.0/66.5 deg/s in `Config.h` and
`planner_bridge/config/joint_limits.yaml`, the per-joint dynamic-limits
gate fix in `planner_bridge` (with regression test), the
`nominal_speed_mps` rail raise to 2.0, and two untracked docs
(`motion-limits-map.md`, `thesis/far-target-joint-limit-stop.md`).

---

## 1. Current verified architecture

### 1.1 Build boundaries

There is no repository-root build. `CMakeLists.txt` was retired to
`CMakeLists.txt.legacy` by commit `2c6d27ff` (2026-08-11) [tree]. The
active layer is four standalone projects, none of which compiles anything
from the inherited root trees [code: basic_control/CMakeLists.txt:136 uses
its own `src/Dynamics.cpp`; planner_bridge/CMakeLists.txt:33-38 uses its
own `trajectory_generation/`]:

| Project | Entry points | Tests | Deps |
|---|---|---|---|
| `Christian_control/basic_control` | `controller` (the only arm-moving binary) + 11 tools | 14 ctest suites | Kortex, Pinocchio, Eigen, optional Vicon SDK |
| `Christian_control/planner_bridge` | `planner_bridge`, `generate_dh_params`, `probe_path_reachability` | 16 ctest suites | GTSAM/GPMP2, Pinocchio, yaml-cpp, Eigen |
| `Christian_control/vicon` | `connect_vicon`, `record_vicon` | 5 ctest suites | Vicon DataStream SDK, Eigen only — no Kortex |
| `Christian_control/tools/panel` | `python3 Christian_control/tools/control_panel.py` | 286 pytest tests | Python stdlib + matplotlib (PLOTS tab) |

Vicon build mode is `sdk` (the SDK .so exists and `NO_VICON` is unset), so
the controller compiles the real Vicon source, not the stub
[tree: basic_control/CMakeLists.txt:107-119]. `basic_control/CMakeLists.txt:2`
still says "No GTSAM / GPMP2 / Vicon required" — true for the planner libs,
now misleading for Vicon (optional, auto-detected).

Test state [tree]: basic_control and planner_bridge ctest both green
2026-08-13 20:07; panel pytest 2026-08-13 20:37 had one failure in the
dirty tree (`test_diagnose.py::Report::test_flags_a_run_log_that_cannot_seed_a_plan`).
The old audit's "27/27" baseline is superseded by 35 ctest + 286 pytest.

### 1.2 Runtime path, startup to shutdown

`main()` is `basic_control/src/Main.cpp:591` [code]. Per selected arm,
`RunOneArm` (`Main.cpp:334`) runs: process lock (flock, per-arm, before any
hardware contact — `ProcessLock.cpp:16-31`, `Main.cpp:621-629`) → Pinocchio
model load → Kortex connect (TCP :10000 + UDP :10001, `Hardware.cpp:34-88`)
→ `ClearFaults` + 500 ms settle → readiness gate `RobotReadyForTakeover`
(`Safety.cpp:81-125`) → `VerifyKinematicHardLimits` (read-only, not
skippable — `Hardware.cpp:177-216`) → `EnsureJointLimits` (write-verify
firmware bands, skippable only via `kSkipStartupGates` — `Hardware.cpp:90-175`)
→ CSV opened and preamble written → Vicon source thread started
(`Main.cpp:439-444`) → log-writer thread → controller/integrator built →
per-arm FIFO created → trajectory-input thread → `RunControlLoop`
(`Runner.cpp:92`).

Takeover: the `ServoingGuard` constructor switches to LOW_LEVEL servoing
(`Safety.cpp:333-339`), the seeded measured position is streamed unchanged
for 25 cycles (0.05 s; `kTakeoverHoldS`, `Config.h:162`) under the full
stop-precedence check, and only then does control begin, reseeded from the
final hold reply (`Runner.cpp:241-357`) [code].

The 500 Hz loop (`Runner.cpp:373-664`, `kControlDtS = 0.002`,
`Config.h:152-157`) runs, in order: measured-dt clamp → previous exchange's
feedback into state → **one Vicon slot read** (ZOH into
`state.world_p_mountseg/world_R_mountseg/world_fresh`, `Runner.cpp:410-430`)
→ `reference.Get` → `controller.DesiredVelocity` → raw request logged
pre-limits → per-joint velocity clamp → position integration with lead
bound, rate envelope, and joint-boundary check (`Actuation.cpp:48-144`) →
`cyclic.Send` (POSITION command, `Hardware.cpp:266-281` — no velocity or
torque command is ever sent) → log push → stop arbitration
(`ResolveStopPriority`, `StopPriority.h`) → prints in post-Send slack →
`sleep_until` the 2 ms grid [code].

Thread structure [code]: control thread per arm, trajectory-input thread,
CSV writer thread, Vicon acquisition thread, main thread. There is no
supervisor thread; all safety runs inline in the loop. All blocking I/O
(CSV, FIFO, Vicon SDK) is off the control thread.

Shutdown [code: Runner.cpp:713-720, Main.cpp:536-575]: loop exit →
servoing restore (explicit, with destructor retry) → stop report → input
thread join → cross-arm stop propagation → log drain → `# exit_reason`
trailer → RAII disconnect. **There is no ramp-down**: on stop the last
integrated setpoint is simply the last thing sent, and the arm's own
low-level servo brings it to rest. Exit 0 only for a user stop with no
faults observed.

### 1.3 The two control laws and the idle-hold dispatch

Both laws are now reachable in the shipped binary — the central change
since the last audit [code: Controller.cpp:70-92]:

- Active trajectory (`reference.joint && !reference.joint_is_idle_hold`)
  → joint law `q̇ = q̇_ref + Kp·wrap(q_ref − q_meas)`, Kp = 5.0
  (`Controller.h:55-74`, `Config.h:198`), with the 8° following-error stop.
- Idle hold (`joint_is_idle_hold`, set by `Targets.cpp:280-284` and by a
  completed trajectory at `:303-305`) → the Cartesian resolved-velocity
  law (`ReactiveLaw.h:186-210`): pose error, DLS inverse
  `Jᵀ(JJᵀ+λ²I)⁻¹` with λ = 0.1, damped null-space limit avoidance —
  toward either the world-hold target (fresh Vicon) or the re-seated
  base-frame hold pose (hold latched off or never engaged after a prior
  engage).
- No Vicon ever → plain joint hold, pre-Vicon behaviour preserved
  (`Controller.cpp:207-226`).

The explicit `reference.pose` channel remains unwritten by production code
(`JointTrajectorySource` is the only `ReferenceSource`, `Targets.h:92`), so
`Controller.cpp:151-177` is still dead at runtime — but the Cartesian law
itself no longer is. The comment at `Controller.cpp:70-76` records why the
old gate made the world hold unreachable ("found on the first hardware run,
2026-08-13") [code].

### 1.4 Planner vs controller boundary

The planner (`planner_bridge`, offline, never links Kortex) owns: start
state read from the newest run CSV (`StartState.cpp:34-60` — the
controller exposes no state API), IK + GPMP2 solve, validation of the
*reconstructed emitted block* using the controller's own Hermite sampler
(`PathValidationReport.h:6-19`), time parameterisation, sequencing, and
emission of one `TRAJ_BEGIN <count> … TRAJ_END` block (deg, deg/s, s) to
stdout, buffered so no failure path can leave partial output
(`BridgeMain.cpp:817-834`) [code].

The controller owns: independent re-validation at ingest against its own
compiled limits (`Targets.cpp:96-102`), the 2° activation splice guard
(`Targets.cpp:246-277`), Hermite sampling on its own clock, the single
velocity clip, integration and boundary handling, all stops, the servoing
lifecycle — and the world hold, which is invisible to the planner [code].

Handoff is a per-arm FIFO (`/tmp/humansl_bridge_targets_right|_left`,
`Config.h:120,126`); `run_session.sh:214-217` solves to a file first and
`cat`s it into the pipe only on planner exit 0. The velocity table is
deliberately duplicated (`Config.h:146-149` ↔
`planner_bridge/config/joint_limits.yaml`) and currently consistent at
76.0/66.5 deg/s in the dirty tree; `VerifyKinematicHardLimits` is the
runtime backstop [code].

### 1.5 Vicon and the world hold today

Acquisition: one SDK thread per arm (`ViconSource.cpp:39-115`), host
`192.168.128.206:801`, retrying forever without delaying takeover;
duplicate Vicon frame numbers are never republished, so a stale sample
cannot fake freshness (`ViconSource.cpp:89-97`). Publication is a
wait-free triple-buffered slot (`BasePose.h:134-171`); absence is NaN +
sequence 0, never a plausible zero [code].

Per cycle the control thread reads the slot once; the same sample feeds
both the log row and the control law (`Runner.cpp:145-155, 410-430`).
Freshness: `world_fresh = valid && seq>0 && finite(age) && age ≤ 0.05 s`
(`Runner.cpp:425-429`, `kWorldHoldFreshMaxAgeS`, `Config.h:333` — the
comment cites a measured p99 age of 9.7 ms on 2026-08-13 [doc]).

The `WorldHold` state machine (`WorldHold.h`) auto-engages on the first
fresh sample at zero error (anchor = current EE world pose), ramps
authority over 2 s (position lerp + quaternion slerp,
`kWorldHoldRampS`), freezes on staleness, resumes with the same anchor if
the gap ≤ 0.2 s and re-anchors (counted) otherwise, and latches off
one-way for the rest of the run when `ramp·‖e‖` exceeds 0.08 m or 0.5 rad
or the error is non-finite (`WorldHold.h:150-206`, `Config.h:336-346`).
Latch-off and post-trajectory transitions re-seat the fallback hold pose
from the current measurement so no step is ever commanded
(`Controller.cpp:100-103, 200-206`) [code]. Frames:
`world_T_base = world_T_mountseg ∘ mount_T_base` (`Controller.cpp:112-116`,
`Frames.h`); the logged `pd_*` stays in base frame [code].

**Stale comments — code disagrees with its own annotations** [code]. The
world hold went live at HEAD, but these sites still describe the retired
"slice 1, observe and log only" behaviour:

1. `Runner.h:76-81` — "OBSERVE AND LOG ONLY, no control law reads it".
2. `Runner.cpp:136-138` — same claim, contradicted at `:145-147` in the
   same file.
3. `Main.cpp:432-434` and the **operator-facing startup print** at
   `Main.cpp:442-444`: "vicon world-pose source: … (observe/log only)" —
   printed on every run start, now false.
4. `BasePose.h:2-3, 24-25, 32` — "observe and record only".
5. `Hardware.h:387-391` — "OBSERVED AND LOGGED ONLY … in this format"
   (format 10 text; `Hardware.h:274-277` in the same header states the
   current truth).
6. `ViconSource.h:2-3`, `ViconSource.cpp:9-11` — "observe and log only".
7. `State.h:27-29` — "external sensing … arrives as a Reference instead",
   violated by the three Vicon fields at `State.h:43-45`; the diagram at
   `State.h:5-14` still says "(future: Vicon)" and omits the
   `joint_is_idle_hold` dispatch.
8. `Runner.h:20-21` and `State.h:201-205` — claim a pose/joint source
   selection in Main.cpp that no longer exists.
9. `Runner.cpp:238-240` — "exactly 0.5 s" takeover hold; it is 0.05 s
   (10× stale).
10. `Controller.h:5-8` — the file-header dispatch summary omits the
    idle-hold → world-hold diversion (the member comments below it are
    current).
11. `Config.h` (dirty tree) lost the comment stating the divergence latch
    tests `ramp·error`, which `WorldHold.h:169-173` still implements.
12. `TrajectoryExecution/CMakeLists.txt:1-10` — legacy, unreachable, still
    lists deleted `Connect.cpp` (harmless dangling text).

### 1.6 Compact system diagram

```
 OPERATOR                 PLANNER (offline)                CONTROLLER (per arm)                      ARM
 goal.yaml ┐                                              ┌────────────────────────────────┐
 planner.  ├─▶ planner_bridge:                            │ input thread:                  │
 yaml      │   CSV start state ─▶ IK ─▶ GPMP2 ─▶ validate │  parse TRAJ block ─▶ validate  │
 joint_    │   (reconstruction) ─▶ emit TRAJ block        │  (pos/vel limits) ─▶ mailbox   │
 limits ───┘        │ stdout → file → FIFO (exit 0 only) ─▶                                │
                                                          │ 500 Hz thread:                 │
 Vicon Nexus ─▶ SDK thread ─▶ triple-buffered slot ──────▶│  feedback ─▶ Vicon ZOH read    │
 (100 Hz, ZOH,   (dup frames dropped)                     │  ─▶ reference.Get              │
  age-gated 50 ms)                                        │  ─▶ law: active traj → joint   │
                                                          │       idle hold → Cartesian    │
                                                          │       (world hold / base hold) │
                                                          │  ─▶ qdot clip (76/66.5 °/s)    │
                                                          │  ─▶ integrate + lead bound     │
                                                          │     + rate env + j2/4/6 hold   │──▶ Kortex UDP
                                                          │  ─▶ Send POSITION ─▶ log push  │◀── feedback:
                                                          │  ─▶ stop arbitration ─▶ sleep  │    q, q̇, τ, faults
                                                          └──────────────┬─────────────────┘
                                     writer thread (SPSC ring, 100 ms) ◀─┘
                                     runs/YYYY-MM-DD/loop_log_<arm>_*.csv  (log_format 11)
                                       ▲ read back: planner start state · panel RUN/RUNS/PLOTS · plot scripts
```

---

## 2. Gates, limits, and stop conditions

Classification: **S** safety-critical, **E** experimental/tuning,
**D** diagnostic (log/report only), **L** legacy/inert. All entries are
[code] unless marked. "Impedes" = what it can block or falsely trip on.

### 2.1 Startup (refuse the run)

| Gate | Where | Input | Action | Protects | Impedes | Class |
|---|---|---|---|---|---|---|
| Process lock | `ProcessLock.cpp:13-31` | flock on `/tmp/basic_control-<ip>.lock` | refuse before any hardware contact | two writers on one arm | none known (kernel drops on crash) | S |
| `--arm` required | `MainArgs.cpp:24-30` | argv | exit 2 | ambiguous arm selection | none | operational |
| Model validation | `Kinematics.cpp:77-225` | URDF dims, frame names | throw → exit 1 | garbage Jacobians from a wrong model | none | S |
| `RobotReadyForTakeover` | `Safety.cpp:81-125` | fault banks, arm state | refuse | taking over a faulted arm | tolerant of lone latched base JOINT_FAULT by design | S |
| `VerifyKinematicHardLimits` | `Hardware.cpp:177-216` | live per-joint hard speed limits vs `kQdotLimitDegS` | refuse (read-only) | streaming a clip the firmware would fault | fires if firmware reports lower limits than expected; the **only** backstop on the panel's unranged velocity write | S |
| `EnsureJointLimits` | `Hardware.cpp:90-175` | firmware JOINT_LIMIT bands j2/4/6 | correct, re-read, refuse on mismatch | degenerate 0/0 bands (2026-08-04 incident) | j6 config-service flakiness; `kAllowUnverifiedActuators` exists for it | S |
| Takeover hold | `Runner.cpp:241-341` | 25 cycles measured-pose streaming under full stop precedence | throw before motion | initial command jump; unquiet arm | none | S |
| FK finiteness, CSV/FIFO creation | `Main.cpp:463-466, 414-429, 509-520` | FK result; filesystem | refuse | model errors; evidence-less runs | none | D/operational |

### 2.2 Plan and reference validation (refuse the block)

| Gate | Where | Input | Action | Protects | Impedes | Class |
|---|---|---|---|---|---|---|
| Planner config rails | `PlannerConfig.cpp:134-239` | every planner.yaml key, exact-keys + ranges | throw naming key and interval | typos silently defaulting | none — rails are wide (speed rail now [1e-4, 2.0]) | D |
| `ValidateJointPath` | `PathValidation.cpp:27-44` | GPMP2 path vs ±126.9/145/118° | exit 4, nothing emitted | optimiser settling a revolution away | hand-copied mirror of `kJointSoftwareLimitDeg`; only a test keeps them in sync | S |
| `ValidatePlannedPath` verdicts | `ValidatePath.cpp:64-289` | reconstruction fidelity, SDF clearance, joint & dynamic limits, start state | refuse emission | plans the controller would reject or that collide (modelled world only) | `optimiser_converged` is hardcoded true (decorative); out-of-grid SDF reads as non-finite, not "collision" | S |
| Per-joint dynamic limits | `PathValidationReport.h:138-153` (dirty-tree fix) | per-joint max q̇, q̈ | invalid → time-scale retry → refuse | the max-vs-max bug that went live with split 76/66.5 limits | acceleration bound is invented (`vel×2`, `PlanSolver.cpp:238`) — too loose for j1-4, too tight for j5-7 vs the unread Table-43 yaml section | S (new) |
| Time scaling | `PlanSolver.cpp:93-103, 250-290` | report maxima, 3 passes | uniform slow-down, geometry untouched | dynamic-limit refusals | remedy uses `minCoeff` of limits while the verdict is per-joint → over-slows by up to ~14% | E |
| Duration floor | `PlanSolver.cpp:40-41` | `max(min_duration_s, dist/speed)` = max(1.0 s, d/0.25) | paces the plan | — | inert-knob trap: speed changes do nothing below 0.25 m of travel | E |
| GPMP2 soft limit factors | `TrajectoryOptimization.cpp:82-91, 278-287, 547-550` | optimiser costs | nudge only, guarantees nothing | — | `vel_limit_thresh` inconsistent between code paths (0.05 vs 0.1), unexplained | L/E |
| Controller ingest validation | `JointTrajectory.cpp:127-186` via `Targets.cpp:96-102` | every point: grammar, count ≤1000, finiteness, monotonic t, position vs software limits, stated and implied-average q̇ vs 76/66.5 | reject block loudly, keep previous reference | trusting the planner | uses software limits, so a legal-by-hardware plan through j2 at 128° is refused; chord-average check can reject a fine Hermite curve | S |
| Splice guard | `Targets.cpp:246-277` | worst wrapped distance first-point vs measured, tol 2° | reject whole block, print distance | a step command at activation | replan loops must respect it; NaN rejects | S |

### 2.3 Per-cycle limits (shape the command)

| Limit | Where | Input | Action | Protects | Impedes | Class |
|---|---|---|---|---|---|---|
| dt clamp | `Actuation.h:28-34` | measured cycle dt | min(dt, 2× nominal) | stall integrating into a position jump | long cycles under-integrate (reference lags) | S |
| Non-finite hold | `Runner.cpp:452-458` | `qdot_raw` finiteness | zero for the cycle, count | NaN reaching the integrator | none (raw NaN still logged as evidence) | S |
| Velocity clip | `Actuation.cpp:18-33` | raw q̇ vs `kQdotLimitDegS` 76.0/66.5 | clamp per joint | firmware velocity fault | **no saturation stop by design** (removed 2026-07-23) — a pinned clip is silent apart from `sat=` in the status line | S |
| Lead bound | `Actuation.cpp:77-90` | command-vs-measured lead, 1.0° | clip candidate | command running away from the joint | not authoritative — rate envelope wins; 3° following-error stop is the backstop | S |
| Rate envelope | `Actuation.cpp:96-99` | step vs `|q̇_clamped·dt|` | clamp | feedback-discontinuity jumps | none | S |
| Joint software boundary | `Actuation.cpp:102-129` | candidate crossing ±126.9/145/118° **outward** | hold whole frame (transmitted), then stop | crossing before firmware warn thresholds | **the documented false-trigger source**: five 2026-08-05 runs stopped with j6 pinned at −118.0° because the held takeover orientation made a reachable position infeasible (`docs/thesis/far-target-joint-limit-stop.md` [doc]); fix chosen (nearest-achievable projection) but not implemented | S |
| Null-space limit avoidance | `ReactiveLaw.h:140-169` | wrapped position in a 20° zone, gain 2.0, 1 s ramp | push inward through damped projector | reaching the hard boundary at all | projector leak into task space (218 mm stall incident, 2026-08-05 — hence deadband, first-class `null_leak_mps` telemetry) | S |
| DLS damping | `ReactiveLaw.h:111-118`, λ=0.1 | task Jacobian | finite solution at singularities | division blow-up | accuracy loss near singularities; `sigma_min` is logged but gates nothing | S |

### 2.4 World-hold trust boundary

| Mechanism | Where | Input | Action | Protects | Impedes | Class |
|---|---|---|---|---|---|---|
| Segment validity | `SnapshotBuilder.cpp:47-72` | occlusion, quat finiteness/norm (1e-3) | mark invalid, never stop | garbage rotations entering control | none | S |
| Duplicate-frame suppression | `ViconSource.cpp:89-97` | Vicon frame number | don't republish | faked freshness | none | S |
| Absence-is-NaN + ZOH slot | `BasePose.h:59-171` | sequence, sample | NaN when absent; same-sequence reuse | plausible-zero poses; finite-differencing a reused sample (standing hazard, nothing does it today) | none | S |
| Freshness gate | `Runner.cpp:425-429`, 0.05 s | sample age | hold freezes, never stops | acting on stale world data | conservative vs measured 9.7 ms p99 [doc] | S |
| Freeze / re-anchor | `WorldHold.h:102-136`, 0.2 s | freshness transitions | same-anchor resume vs counted re-anchor | ramp-churn on a flickering stream; losing the anchor silently | long blackout loses the absolute point (honestly counted) | S |
| Authority ramp | `WorldHold.h:186-193`, 2 s | time since engage | error scaled by r∈[0,1] | full-gain transient at engage; wrong-sign frame errors become slow drift, not a jerk | slower convergence after engage | S |
| Divergence latch | `WorldHold.h:150-177`, 0.08 m / 0.5 rad on `ramp·e` | unramped anchor error × ramp | one-way latch off to base-frame hold — degrade, not stop | wrong calibration dragging the arm; engage/diverge oscillation | latched for the whole run; authority-scaling was a deliberate 2026-08-13 review change | S |
| Re-seat guards | `Controller.cpp:100-103, 200-206` | current measurement | zero-error fallback hold | commanded steps toward a stale pose | none | S |
| Trajectory precedence | `Controller.cpp:78-82` | active trajectory | hold resets (latch persists) | plan and hold fighting | none | S |

### 2.5 Stops (end the loop) and their precedence

Precedence (`StopPriority.h:43-58`, pinned by `test_supervisor`):
following error → left low-level → live fault (if `kStopOnFault`) →
joint-limit warning → stale feedback. An ignored fault still taints the
exit code (`Runner.cpp:526`, `Main.cpp:571-575`).

| Stop | Trigger | Where | Class |
|---|---|---|---|
| Cartesian following error | any joint \|cmd−meas\| > 3.0° | `Safety.cpp:33-50` | S (disable-able via `kDisableFollowingErrorStop`) |
| Joint-tracking following error | worst wrapped \|q_ref−q_meas\| > 8.0° | `Controller.h:56-75` | S (**not** touched by the disable flag; only a non-positive threshold disables it) |
| Left low-level servoing | arm state ≠ SERVOING_LOW_LEVEL | `Runner.cpp:522` | S, unconditional — survives every override |
| Live fault | any actuator bank ≠ 0, or base bank minus latched JOINT_FAULT | `Safety.cpp:52-58` | S (gated by `kStopOnFault`) |
| Joint-limit warning | consumer of the boundary hold; safe frame transmitted first | `Runner.cpp:541-545` | S |
| Stale cyclic ack | `command_id` unchanged 25 replies (50 ms) | `Freshness.h:15-59` | S |
| Non-finite command | 3 consecutive cycles (held at zero meanwhile) | `Runner.cpp:561-567` | S |
| Cycle overrun | 10 consecutive cycles > 1.5× nominal (unclamped dt) | `Runner.cpp:387-395, 568-574` | S |
| Communication / internal | any exception out of the exchange; four catch tiers | `Runner.cpp:674-707` | S |
| Cross-arm propagation | either arm's non-clean stop stops both | `Main.cpp:546-547` | S |

**There is no software e-stop.** No e-stop input, handler, or interlock
read exists anywhere in `basic_control` [code: grep]. SIGINT/SIGTERM route
through the same graceful stop path; the physical e-stop and the operator's
hand are the layer above this program, and two Config warnings say exactly
that (`Runner.cpp:111-127`).

### 2.6 Guard overrides and the panel's write surface

All four overrides are at safe defaults in the working tree [code:
`Config.h:290, 303, 311, 323`]: `kStopOnFault = true`,
`kAllowUnverifiedActuators = false`, `kSkipStartupGates = false`,
`kDisableFollowingErrorStop = false`. All four are echoed into every run's
CSV preamble (`Main.cpp:117-122`).

All four are **browser-writable** by explicit 2026-08-12 decision
(`tools/panel/config_file.py:5-13, 55-70`), and `write_vector_knob` can
rewrite `kModelVelocityLimitsDegS` with **no range check at all**
(`config_file.py:252-255`) — `VerifyKinematicHardLimits` at startup is the
only automatic backstop. `Config.h:289` still says `kStopOnFault` is
"never runtime-settable"; that sentence predates the panel decision.

`Config.h.panel.bak` and `planner.yaml.panel.bak` [tree] are stale
pre-first-write panel snapshots (the Config one still holds 45 deg/s and
predates the entire world-hold block). They are read by nothing; restoring
one by hand would silently revert the speed decision and delete the
world-hold thresholds.

### 2.7 Things that gate nothing (inert/decorative)

- `kAcceptOrientationTargets` (`Config.h:399`) — the Cartesian pipe
  grammar was retired; zero consumers. L.
- `report.optimiser_converged` — hardcoded `true` (`PlanSolver.cpp:265`). L.
- `joint_limits.yaml` acceleration section (Table 43) — parsed by nothing;
  the enforced acceleration bound is the invented `vel×2`. L.
- `CheckInterArmClearance` (`InterArmDistance.cpp`) — reachable only from
  its test; no binary calls it. E, unwired.
- `sigma_min` — computed, logged, printed; gates nothing. D.
- Velocity-clip saturation — counted and printed, never stops. D by design.

### 2.8 Firmware-side limits and the known gaps [doc: motion-limits-map.md]

Enforced robot-side: configured JOINT_LIMIT bands (written each connection
by §2.1) and the live kinematic hard speed limits 80.0021/70.004 deg/s
(read each startup, backstopping our 76.0/66.5 clip ≈ 95%). Never read
from this robot: the firmware `MAXIMUM_VELOCITY` and `FOLLOWING_ERROR`
thresholds — the faults that actually end fast runs — plus torque tables
and any low-level acceleration figure. The read-only tool to close the
first gap exists and is built (`read_safety_limits`), but no recorded
readout exists in `runs/`. Our 8°/3° stop thresholds were chosen without
knowing the firmware's own following-error number.

---

## 3. Observability and visualization

### 3.1 What each surface actually shows

Panel tabs (`static/index.html:57-62`): RUN, CONFIG, TARGETS, RUNS,
SESSION, PLOTS, plus the safety bar and e-stop footer outside the tabs.
Data character per view [code]:

| Surface | Consumes | Character |
|---|---|---|
| RUN scene (3D) | SSE `meas_j*`/`cmd_j*` + build-generated DH via `/api/dh` | **FK reconstruction** — only the 7 joint angles per chain are measured; links, mounting, root convention come from the URDF-derived tables (`scene.js:5-11, 197-217`) |
| RUN error/joints/scalars | SSE rows at ~20 Hz over the growing CSV | live log; single-cycle edge flags are aggregated so 20 Hz sampling cannot miss them (`telemetry.py:346-377`) |
| RUN "controller says" | `controller.log` tail | live text |
| CONFIG | Config.h / planner.yaml / joint_limits.yaml text + build status | source text, not robot data |
| TARGETS | goal/plan APIs, solver runs | plan = solver output; start state may be a recorded run |
| RUNS | one streaming pass over a chosen CSV (`runs.py:175-314`) | after-the-fact analysis of logged data |
| PLOTS (new, uncommitted) | POST `/api/plots/run` → `subprocess` matplotlib scripts, 90 s timeout (`plots.py:59-144`) | logged data rendered offline |
| Replay (`--replay CSV`) | same SSE pipeline in replay mode (`telemetry.py:723-770`) | logged data re-paced; scene is still FK reconstruction; REPLAY banner + wrong-arm streams deliberately silent |

**Independently measured physical behaviour is absent from the panel**:
no panel file references `vicon_*` or `hold_*` columns [code: grep], and
two RUN-tab captions ("world equals mount · no Vicon", `index.html:89,
110-113`) are stale for format-11 runs. The only world-frame evidence
surface is `plot_world_hold.py`, whose figure 3 recomputes
`world_T_base(t)` from the logged Mount pose + `dual_arm_mounting.yaml` —
deliberately an independent reconstruction, not the controller's own
numbers (`plot_world_hold.py:12-18, 370-480`).

### 3.2 The run CSV (log_format 11, 190 columns)

Authority: `Hardware.cpp:361-485`, prose map `Hardware.h:222-277` [code].
Preamble: `# log_format`, `# vicon_source`, `# arm`, ~35 `# key = value`
Config lines including the guard overrides and all six world-hold knobs (but not the velocity clip — see §4.1);
exit trailer `# exit_reason/# exit_time_s/# exit_cycle/# faults_observed`.
**Parser trap** [code + verified on a real file]: three `# startup_*` lines
are written *after* the header row; parsers must accept `#` anywhere (the
panel and plot scripts do).

The requested → sent → measured contract (`Hardware.h:279-289`):

- (`pd_*`/`p_*` carry FK only on Cartesian-law cycles; joint-law cycles — active trajectories and pre-engage holds — write NaN, and `cycle == 0` rows carry uncomputed zeros [code].)
- **requested** — the controller's own output before any limiter:
  `reqvel_j*` (pre-clip, NaN preserved as stop evidence), `req_j*`
  (integrated candidate pre-lead/rate).
- **sent** — what entered the cyclic message: `cmd_j*` (the `Send`
  argument), `cmdvel_j*` (velocity realised by the applied command).
- **measured** — that same exchange's reply: `meas_j*` (unwrapped onto the
  command's turn), `measraw_j*`, `vel_j*`, `torque_j*`, `fault_j*`.

So `req−cmd` = lead + rate limiting; `cmd−meas` = the tracking error the
following-error guard tests. `taskvel_j*`/`nullvel_j*`/`null_leak_mps`
decompose the Cartesian law; `sigma_min`, `rot_error_rad` diagnose it.

Vicon columns (format 10): `vicon_seq/frame/latency_s/age_s` plus
`vicon_<seg>_{x,y,z,qx,qy,qz,qw,valid}` for all five segments —
**raw Vicon-world segment poses, not calibrated mount poses**
(`Hardware.h:264-266`); absence is all-NaN + seq 0; age can go slightly
negative by construction (`Hardware.h:403-409`). World-hold columns
(format 11): `hold_state` (0 inactive/1 engaged/2 frozen/3 latched-off),
`world_err_m`, `world_err_rot_rad` (unramped latch inputs), `hold_ramp`,
`hold_reanchor_count`.

### 3.3 Timestamps and the alignment rule

All controller stamps are one steady clock, seconds since log start:
`time_s` (cycle start), `dt_s`, `t_send_s`, `t_recv_s` [code:
`Runner.cpp:487-490`]. **A row mixes two exchanges**
(`Hardware.h:306-311`): `p_*`, `pd_*` and every controller input derive
from the feedback received in row i−1 (near its `t_recv_s`); `meas_j*`
etc. come from row i's own reply. Within-row comparisons of `pd_*` vs
`p_*`, or `cmd_j*` vs `meas_j*`, are self-consistent; anything across the
two families must be matched by timestamp, never row index —
`analyze_run.py:134-148` implements exactly this. Vicon: the sample
instant is `time_s − vicon_age_s` on the same clock; `vicon_seq`, not the
row, is the sample identity, and nothing may finite-difference across a
repeated sequence.

### 3.4 Event fields

Per-row: fault banks (`fault_j*`, `base_fault`), `arm_state`,
`refresh_ok`, `lead_limited_j*`, trajectory edges
(`traj_activated/rejected/complete`, single-cycle), `traj_start_error_deg`
on rejection rows, `joint_follow_error_deg` + `joint_follow_stop`,
freshness (`ack_unchanged_j*`, frame ids, `vicon_age_s`, `vicon_*_valid`),
and the hold quartet above. Stop reasons are the `# exit_reason` trailer
plus one "loop stopped …" sentence in `controller.log`
(`Safety.cpp:187-285`); the panel merges both (`runs.py:317-338`).

**Gaps** [code]: velocity-clip and rate-envelope saturation have no
columns (infer from `reqvel` vs `cmdvel` and `req` vs `cmd`); the
joint-limit stop's *joint identity* never reaches the CSV — it lives only
in the console sentence, so keep `controller.log` beside the CSV.

### 3.5 Plot scripts and their limits

`plot_run.py` (requested/sent/measured, six figures), `plot_world_hold.py`
(Vicon health; hold state and error vs ramp-scaled latch thresholds;
EE-in-world reconstruction — independent of the hold's arithmetic though built on the same logged mount data; error vs base speed),
`plot_joint.py`, `plot_move.py`, `analyze_run.py` (integrity report,
timestamp-interpolated tracking PDFs, refuses >1% dropped cycles without
`--force`), `measure_delay.py`. Known limits [code]: every script loads
the whole file (a 3.0 GB, ≈3.3M-row log on 2026-08-13 exceeds RAM /
the panel's 90 s timeout); `cycle == 0` takeover rows carry struct-default
zeros for `p_*` (not real FK — `plot_world_hold.py:394-414` and
`analyze_run.py:151-160` both exclude them); only the three newest logs on
disk are format 11, so hold figures on older runs honestly degrade;
`plot_world_hold.py` falls back to the left arm on an unrecognised
preamble, which silently flips `mount_T_base` — check the arm line.
The panel replay shares the whole-file-read shape (`telemetry.py:196-198,
731-733`); do not replay multi-GB logs.

---

## 4. Worked debugging method (read-only)

### 4.1 Selecting and validating a run

1. List candidates: `ls -lt runs/*/loop_log_*.csv | head`. Selection is by
   **mtime, not name** (with `--arm both`, lexical order returns the wrong
   arm — `runlog.py:15-31`). Note the file size before anything loads it.
2. Read the preamble and trailer without loading the file:
   `grep '^#' <run.csv>` (cheap even on GB files). Confirm: `log_format`,
   `arm`, `vicon_source`, the four guard overrides, and `# exit_reason` —
   noting that the preamble does NOT echo the velocity clip or software
   joint boundaries (WriteConfigLines omits them [code]); which limits a
   run obeyed is evidenced by its session.json binary hash/build time or
   the `cmdvel_j*` saturation plateaus — remembering `#` lines
   also appear after the header row and at EOF.
3. Confirm the header row matches the format:
   `grep -m1 -v '^#' <run.csv> | tr ',' '\n' | wc -l` (190 for format 11)
   and check the columns you need exist (`req_j1`, `t_send_s`,
   `hold_state`).
4. Pair the CSV with its `controller.log` — the joint-limit stop's joint
   number and other stop sentences exist only there.

### 4.2 Plotting and inspecting gates

5. `python3 Christian_control/basic_control/scripts/plot_run.py <run.csv>`
   for the requested/sent/measured triad per joint, or the PLOTS tab
   (loopback panel, no session needed). Read the triad as: `req` vs `cmd`
   = what the limiters did; `cmd` vs `meas` = what the arm did.
6. For world-hold runs (format 11):
   `plot_world_hold.py <run.csv>` — Vicon health first (age vs the 50 ms
   threshold, validity bars), then hold state/error vs the ramp-scaled
   latch thresholds, then the independent EE-in-world figure.
7. Gate events without plotting: trajectory edges
   (on `traj_*` columns or the RUNS tab summary),
   `joint_follow_error_deg` against 8.0, `lead_limited_j*` runs,
   `ack_unchanged_j*` growth, `hold_state` transitions and
   `hold_reanchor_count` steps.
8. Replay for spatial intuition:
   `python3 Christian_control/tools/control_panel.py --replay <run.csv>`
   (add `--replay-speed`). The scene is an FK reconstruction from logged
   angles; the REPLAY banner confirms no arm is involved.

### 4.3 The causal-explanation template (dissertation-ready)

> **Observed symptom.** One sentence of what the run did, cited to rows/
> times: "the loop stopped at t = 41.86 s with `# exit_reason =
> joint_limit_warning`."
> **Evidence.** The specific columns/preamble lines and their values, with
> the timestamp-alignment rule respected; console sentences quoted where
> the CSV is silent (e.g. which joint).
> **Relevant gate or design choice.** The mechanism by name, its
> implementation site, threshold, and *why it exists* (its incident or
> decision record).
> **Conclusion.** The causal chain, distinguishing confirmed observation
> from hypothesis; alternative explanations and what ruled them out.
> **Next experiment.** The smallest read-only or sim check that would
> discriminate the remaining hypotheses — before any code change.

Worked instance (from the repo's own records [doc:
`thesis/far-target-joint-limit-stop.md`]): symptom — five runs ended
`joint_limit_warning`, j6 at exactly −118.0°; evidence — clean fault
banks, monotonic j6 march in `cmd_j6`, `null_leak_mps` nonzero; gate —
the outward software-boundary hold (`Actuation.cpp:102-129`), protecting
firmware limits; conclusion — not a hardware fault: the held takeover
orientation made the position target infeasible and the DLS gradient
walked j6 into its boundary; next experiment — offline IK over relaxed
orientations (done; showed ~45° of tool pitch buys full position reach).

### 4.4 What a CSV or replay can and cannot establish

Can [code-backed]: everything the controller computed, commanded, and was
told — requested/sent/measured per joint, limiter action, gate trips,
stop reason and precedence, Vicon sample identity/age/validity, hold
state and its error *as the controller computed it*, timing (compute
time, UDP round trip, overruns, log drops).

Cannot: (a) true physical pose — `p_*` and the replay scene are FK from
the URDF; a model or mounting error is invisible by construction;
(b) world-frame truth beyond what Vicon reported — `vicon_*` are raw
segment poses with no `mountseg_T_mount` calibration yet, and
`world_err_m` is the controller's own belief, not an external measurement;
(c) anything about torque/current/temperature interlocks or the unread
firmware thresholds (§2.8); (d) why the *firmware* faulted, beyond the
decoded bank bits; (e) behaviour of the current dirty tree — a log proves
the binary that wrote it (check its preamble's config echo against
today's Config.h before attributing today's constants to an old run);
(f) anything on `cycle == 0` rows beyond "the takeover hold streamed the
seed pose". Independent physical proof requires an external measurement —
which is precisely the role the Vicon chain and `plot_world_hold.py`'s
reconstruction are being built to play.

---

## 5. What changed since the previous audit (2026-08-07)

Both headline findings are resolved:

- **Root build**: retired, not repaired — `CMakeLists.txt` →
  `CMakeLists.txt.legacy` (`2c6d27ff`), README banner added. The old
  audit's slice-2 recommendation, executed.
- **Unreachable pose channel**: reconnected via `joint_is_idle_hold`
  (`Targets.cpp:280-284`, `Controller.cpp:78`) — found the hard way on the
  first hardware run, 2026-08-13, exactly as the audit warned. The
  null-space limit avoidance is consequently live during idle/hold; during
  active trajectory execution the joint law still runs without it.

New since then: the `Christian_control/vicon` project (snapshot contract,
recorder, replay, 5 tests), the world hold (log formats 10 and 11), the
control panel grown to 286 tests including the run-config write surface
and the PLOTS tab, both live projects fully decoupled from the inherited
root trees, and the velocity envelope reconciled at 76.0/66.5 deg/s
across controller and planner (the old three-way C7 disagreement is
closed; the alias is now equal-by-construction, `Config.h:241`).

Still true from the old audit: B2 (`TrajectoryExecution/CMakeLists.txt`
dangling sources, now unreachable), B10 (`test_run_session.sh` wired into
nothing), C1/C2/C3 dead symbols (`kAcceptOrientationTargets`,
`kLoopLogPrefix`, `right_base_frame_id()` — slice 1 never executed), B11
(`show_frames.m` stale grammar comment), S8 (raw-vs-clamped dt pair
undocumented), S13 (IK failure message names no residual).

Actively wrong if applied today: the old L2 recommendation (annotate the
pose channel "currently unreachable") — the channel is now live; the
correct stale-doc targets are §1.5's comment ledger and
`docs/architecture.md`'s component table (refreshed 2026-08-14 alongside
this audit).

Open items this audit adds: the twelve stale comment sites (§1.5); the
invented planner acceleration bound vs the unread Table-43 yaml section;
the `minCoeff` time-scaling asymmetry; the panel's zero world-hold/Vicon
telemetry and two stale captions; no `test_plots.py` for the new panel
module; the prefix-string path check in `server.py:164,180` (weaker than
`runs.py`'s `is_relative_to`); the joint-limit stop's joint identity
missing from the CSV; whole-file plot/replay loading vs multi-GB logs;
and the `.panel.bak` restore hazard (§2.6).
