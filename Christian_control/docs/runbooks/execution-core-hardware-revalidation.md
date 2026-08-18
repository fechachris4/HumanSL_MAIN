# Runbook: supervised hardware revalidation of the extracted execution core

**Status:** procedure only — this document is NOT an authorization, and no
run described here has occurred. Every hardware session still requires
Christian's explicit approval for that specific session (project CLAUDE.md).

**Why this runbook exists.** Plan 01 (2026-08-16) reorganized the
`basic_control` command pipeline into the hardware-independent
`ArmExecutionCore` (`src/ExecutionCore.h`) inside the
`humansl_execution_core` library, and rewired the Kortex runner to call it.
The offline evidence — unit tests, the linkage test, and byte-for-byte
replay of the frozen pre-extraction characterization fixture — establishes
software equivalence on recorded inputs. It cannot establish physical
equivalence: low-level arm response, Vicon latency on the live rig, timing
under real exchange jitter, and calibration are outside what replay can
prove (engineering contract §13). The refactored `controller` binary is
therefore **offline-validated only** until one supervised session follows
this procedure and its review passes.

**Evidence label for the session** (contract §13): `supported torso-rig
hardware test` unless a person actually wears the system, and the session
record must state the fixture/mannequin/human-worn condition, the overhead
suspension state (attached; slack, partially load-bearing, or constraining),
which arm(s) ran, and the payload/tool/marker configuration (contract §1).

---

## 1. Build-provenance gates (before anyone touches the arm)

1. The tree that built the binary is the reviewed extraction diff. Record
   `git rev-parse HEAD` and `git status --porcelain`; a dirty tree must be
   explained file-by-file in the session notes before proceeding.
2. The frozen characterization fixture is byte-identical to the reviewed
   one:
   `sha256sum Christian_control/runtime/tests/fixtures/execution_preextract_v1.csv`
   must print
   `d0575dec586906f0a2bff0d72fa83443b04565da2cc835d2b80c6fde61954c6a`.
   A different hash means the replay baseline changed; stop and re-review.
3. The full hardware-free suite passes on the exact tree the binary was
   built from — all 21 registered tests, including `execution_core`
   (contract and allocation-freedom), `execution_characterization` (frozen
   replay), `execution_core_linkage` (no Kortex/Vicon SDK symbols in the
   core archive), and `log_schema`:

   ```bash
   cmake --build Christian_control/runtime/build -j2
   ctest --test-dir Christian_control/runtime/build --output-on-failure
   ```

4. Binary freshness is then enforced mechanically by
   `planning/scripts/run_session.sh` (`fresh_or_die`): it refuses a
   `controller` or `planner_bridge` binary older than any source file, and
   a generated DH YAML older than the URDF. Do not pass `--allow-stale`
   for a revalidation session.
5. The session's `session.json` (written by `run_session.sh` on every exit
   path) records `git_revision`, `git_dirty`, `controller_sha256`,
   `bridge_sha256`, and `urdf_sha256`. Keep it with the run log; it is the
   provenance record for this procedure.

## 2. Configuration snapshot

All control policy is compiled into the binary from `src/Config.h` and
snapshotted once at construction into `ExecutionConfig`
(`ProductionExecutionConfig()`; the identity mapping is pinned by the
`execution_config` test). There is no runtime configuration file for the
controller, so the binary hash in `session.json` IS the control-policy
snapshot. Additionally:

- `run_session.sh` copies `goal.yaml` and `planner.yaml` into the session
  directory; confirm the copied `goal.yaml` contains the conservative
  target defined below, and nothing else.
- Record in the session notes the values this run relies on, read from the
  built revision's `Config.h`: velocity limits `kQdotLimitDegS`
  (76 deg/s joints 1–4, 66.5 deg/s joints 5–7), following-error limit
  `kFollowingErrorLimitDeg` = 3 deg, command-lead limit
  `kCommandLeadLimitDeg` = 1 deg, `kStopOnFault` = true, world freshness
  `kWorldFreshMaxAgeS` = 0.05 s and `kWorldProlongedStaleS` = 0.2 s.

## 3. Workspace and e-stop confirmations (the run_session.sh checklist)

Before typing `GO`:

- Christian is present and personally authorizes this specific session.
- The workspace around both arms is clear of people and obstacles.
- The emergency stop is in reach of the operator's hand.
- The Kinova web dashboard is closed (it blocks `SetServoingMode`).
- The arm(s), fixture condition, and suspension state are recorded.

The typed `GO` is only a script gate, not an authorization.

## 4. Phase A — conservative initial hold

Start the session with `run_session.sh --arm <right|left>` (one arm only
for the first revalidation). The controller starts in a zero-error
Cartesian hold and captures a fixed world hold on the first fresh Vicon
sample. Note that the launcher starts a planner worker and the controller
activates the first published plan, so for this phase set the `goal.yaml`
target AT the current tool pose (within about a centimetre, orientation
unchanged): the first activated trajectory is then effectively a hold.
Observe at least 30 s of stationary hold with the Mount stationary.
Expected behaviour, written before the run:

- no visible tool motion; `req_j*` and `reqvel_j*` near zero;
- no `lead_limited_j*` flags and no velocity saturation during the hold;
- `world_fresh` = 1 for essentially all cycles, `vicon_age_s` ≤ 0.05;
- following error (|`cmd_j*` − `meas_j*`|) well under 3 deg.

Any unexpected motion, oscillation, or audible anomaly during Phase A:
press the e-stop or Enter (clean stop), keep the log, and stop the
procedure — the session review then happens before any retry.

## 5. Phase B — nearby target

Only after Phase A looks correct in the live terminal and its log has
been reviewed: a separate session whose `goal.yaml` target sits a few
centimetres (≤ 0.05 m recommended, orientation unchanged or nearly so)
from the current tool pose. One move, then hold. Do not command broader motion,
dual-arm operation, or a moving-wearer test in the same session as the
first revalidation.

## 6. Stop criteria

Operator stops: Enter (clean stop through teardown) or Ctrl+C; the e-stop
at any time. Automatic stops that remain armed exactly as before the
extraction (StopPriority ladder, then the counter stops):

- following error above 3 deg between command and measured position;
- loss of low-level servoing on either arm;
- any enabled live fault (`kStopOnFault` = true);
- joint-limit warning (whole-frame hold is sent first, then the stop);
- stale actuator acknowledgement for 25 completed replies;
- non-finite controller output for 3 consecutive cycles;
- loop overrun for 10 consecutive cycles.

On any stop the last position setpoint is held and SINGLE_LEVEL servoing
is restored (teardown D1), and the decoded stop report prints (D2).

## 7. Log fields to review afterwards

The run CSV is format 13; read it with `scripts/runlog.py` by column name.
The minimum post-run review set:

- timing: `dt_s` distribution, `jitter_us_j*`, `cycle` continuity;
- world state: `world_fresh`, `vicon_age_s`, `vicon_seq`,
  `vicon_mount_*` pose/twist and `world_mount_twist_valid`;
- tracking: `cart_ref_*_world_*` against `cart_meas_*_world_*` (pose and
  twist), `rot_error_rad`;
- commands: `taskvel_j*`, `nullvel_j*`, `null_leak_mps`, `req_j*`,
  `reqvel_j*`, `cmd_j*`, `cmdvel_j*` versus `meas_j*`;
- guards: `lead_limited_j*`, `ack_unchanged_j*`, `fault_j*`, `base_fault`,
  `arm_state`, `refresh_ok`;
- reference lifecycle: `cart_traj_activated/rejected/complete/cancelled`,
  `cart_replan_requested`, `cart_trajectory_id`,
  `cart_planner_vicon_sequence`.

## 8. Post-run review, before any broader motion

1. Confirm the exit was a clean operator stop (exit code 0, no fault in
   the stop report), or explain the stop from the log.
2. Check every Phase A expectation above against the CSV, not memory.
3. General health sanity check against earlier hardware logs. No
   pre-extraction hardware log of the world-Cartesian control law exists:
   the most recent runs under `runs/` (2026-08-13, log formats 9–11) come
   from the previous joint-trajectory controller, so they have no
   `world_fresh` or `cart_*` columns and their control law, gains and
   limits all differ. They can therefore only serve as a loose sanity
   baseline for quantities whose meaning did not change — `dt_s`
   distribution and jitter, following-error magnitude staying well under
   the 3 deg stop limit, absence of faults, clean stop behaviour — not as
   evidence about the extraction. Any equivalence claim about the
   extraction rests on the offline replay gate alone; this session's log
   becomes the first hardware baseline for the world-Cartesian law.
4. Record the outcome in the session notes with the contract §13 evidence
   label. Only after this review passes may a broader-motion session be
   proposed — and that session again requires its own explicit
   authorization.
