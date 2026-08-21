# Intent story

The interpretation layer over `raw-prompt-log.md`. Rules:

- Every claim cites the log entries (by timestamp) it rests on. An uncited
  claim is suspect.
- Entries under **Interpretations** are hypotheses until Christian confirms
  them; confirmed goals move to **Approved goals**.
- Goals are recorded as outcomes, never methods — intent captured too
  concretely is imagination captured too early.
- Christian reviews *diffs* of this file (git history is the audit trail),
  batched at session boundaries, not the whole document each time.
- Christian's live word outranks everything in this file. Conflicts earn
  at most one plain-English question, then compliance and a recorded
  supersession. This file is memory, never authority.

## Approved goals

Goals Christian has explicitly confirmed. Cite the confirming prompt or
commit.

- The intent record itself: raw prompts as ground truth, cited
  interpretation, active pursuit of the why behind every want, exposure to
  options beyond what he asks for. (Approved in the 2026-08-12 design
  session; principles committed to CLAUDE.md in 3c2a5687.)
- Misreadings must be caught before work starts, not after: when a prompt
  carries new intent, the agent proposes its reading of the why and gets
  confirmation before acting. The visible check is the point — it is
  Christian's evidence that alignment is working. (Confirmed via
  interactive question, 2026-08-12 ~14:50. Cited to the session
  transcript: the prompt predates hook capture in that session.)
- Vicon integration into controller and planner, staged. Christian wants
  live Vicon data (the five Nexus segments, markers as diagnostics, and
  the Vicon-world origin related to the robot's mount frame) feeding both
  the controller and the planner, delivered as small separately-approved
  stages: verify + capture, snapshot/record/replay, calibration, the
  controller's world-frame hold (chicken-head), then plan-time frame
  conversion in the planner. The why he confirmed: the wearer moves, so a
  live world-frame estimate of the mount is the missing input on both
  sides — base compensation is control, plan-frame conversion is
  planning. (Prompt: raw-prompt-log 2026-08-12 15:21. Reading and scope
  confirmed via interactive questions the same session; design approved
  as presented — spec at
  docs/superpowers/specs/2026-08-12-vicon-controller-planner-integration-design.md.)
- The control panel edits velocity limits (`kModelVelocityLimitsDegS`,
  never the `kQdotLimitDegS` alias) and all four guard overrides
  (`kStopOnFault`, `kAllowUnverifiedActuators`, `kSkipStartupGates`,
  `kDisableFollowingErrorStop`) from the browser, same edit-then-rebuild
  flow as the existing knobs, type-checked validation only. Position
  limits and warn/error thresholds stay read-only — out of scope, not
  forgotten. See "Superseded decisions" for the guard-override reversal.
  Implemented 2026-08-12; 237/237 panel tests pass. (Cited to raw-log
  entry ~15:03 and the three confirming AskUserQuestion exchanges that
  followed it.)
- The browser panel is the one place run configuration is edited: planner
  tuning (`planner.yaml`) and the planner's joint-limit tables
  (`joint_limits.yaml`, danger-flagged by explicit choice) become editable,
  and each arm's goal is edited through structured forms rather than raw
  YAML, with the raw file kept as a collapsed advanced view. The why
  Christian gave: tuning the planner from the browser was impossible and
  the goal editor was "too much" — the whole per-run surface belongs in
  one place, for both arms. Panel edits stay value-replacements in the
  file's own text, so hand editing and comments survive. (Prompt: session
  transcript, 2026-08-12 ~16:19 (mid-turn message, predates hook capture
  for that turn); design approved the same session — spec at
  Christian_control/docs/superpowers/specs/2026-08-12-planner-config-panel-design.md.)
- The planner's static obstacle scene has one persistent definition in the
  planner's YAML and is edited and visualised from the existing panel. The
  panel may hold an explicitly unsaved visual draft, but it owns no second
  scene, SDF, collision rule or clearance judgement; `MakeMountSdf()` and
  GPMP2 remain the consuming collision path. The first approved slice uses
  named mount-frame boxes and finite vertical cylinders, lets their position
  and dimensions be changed numerically or with viewer handles, retires the
  per-arm `goal.yaml` box path, and never solves or commands merely because a
  scene was edited or saved. Because the live in-process planner rereads
  `planner.yaml` on later replans, the panel refuses persistence while a
  controller is commanding, while still allowing a browser-local draft. The
  why: Christian wants to understand visually why a plan avoids an obstacle
  and eventually compare obstacles, planned and executed trajectories and
  collision geometry, without letting the visualisation become a second
  source of physical truth. (Prompt and explicit design approval: current
  session, 2026-08-21; accepted design recorded in
  `docs/superpowers/specs/2026-08-21-panel-scene-editor-design.md`.)
- The arm should move as fast as the Kinova's own limits allow, not slower
  because of numbers chosen during bring-up. Christian's stated source is
  his professor: move "as fast as possible according to the physical limits
  of the kinova arm, so only the ones that have been set" — his point being
  that under low-level control the binding limits are the arm's, not ours.
  He is right about the mechanism, and the repository already established
  it (`docs/decisions/qdot-limit-raise.md`, 2026-07-22): per-mode soft
  limits do not govern the low-level stream, and what provably faults is
  the per-actuator firmware safety bank (MAXIMUM_VELOCITY,
  FOLLOWING_ERROR, JOINT_LIMIT_HIGH/LOW). Present state at the time of
  asking: commanded clip 50 deg/s against live hard limits of 80.0021
  (joints 1-4) and 70.004 deg/s (joints 5-7). This records the objective
  only; what to change and when is unsettled — see Open questions.
  The why, as first confirmed (~15:20): to demonstrate in the thesis that
  the hardware was driven to its real capability, with peak speed as the
  objective itself. Broadened by Christian ~15:35 (interactive question,
  after raw-log 15:32:43): the speed serves all three of tracking the
  wearer, characterising the platform, and demo credibility — so the
  earlier "not a means to tracking performance" reading was too narrow,
  though the target remains the Kinova limits and speed everywhere, for
  every goal type, not per-goal tweaks. (The Interpretations entry below
  preserves the earlier refutation; the ~15:35 broadening supersedes its
  scope — tracking is one of three whys, not zero and not the sole one.)
  How this coexists with the world-frame stability goal: by phase — speed
  governs free moves between poses, the hold governs stationary work near
  the wearer, and both pass through the same velocity limits and safety
  path, so neither goal relaxes the other's safety envelope.
- The intent steward's designed mechanisms should actually exist, not
  remain an agreed design. Christian noticed the gap himself — "what
  happened to, yesterday we worked on ... how to improve our workflow ...
  you were supposed to have a hook" — and approved building all three
  (staleness and session-end diff reminders, blind red-team
  reconstruction, prediction ledger) at reminder strength rather than
  blocking, so the record never becomes an authority that can refuse him.
  Built 2026-08-13. (Prompts: raw-prompt-log 2026-08-13 14:56:57,
  14:59:04, and the approving 15:08:39; enforcement and scope chosen via
  two interactive questions between them.)
- The agent's own memory should be actively curated for performance, not
  left to accumulate: "i want you to manage the memory, clean it up a
  little to improve your own performance." (Prompt: raw-prompt-log
  2026-08-13 14:58:02.)
- The panel shows live Vicon world pose by reading what the controller
  already logs, rather than computing or displaying a pose of its own.
  Christian chose this over two cheaper routes that would have shown a
  pose the controller never acted on. This refines the staged Vicon goal
  above; it does not replace it. (Prompts: raw-prompt-log 2026-08-13
  14:51:39, 15:10:25, 15:11:05; choice recorded in
  docs/intent/predictions.md for the same date.)
- The end-effector must stay true in the world while the wearer moves —
  Christian restated the project's master goal in his own words ("this is
  for my SRL arm stability problem being able to keep the end effector
  true in the world") and asked that design choices be understood against
  it, not just recorded. The reading he was given and did not dispute: the
  wearer is the dominant disturbance, not the operator — the arm exists to
  hold a point in the room while the human does something else, and the
  human doing something else is what shakes the base. Consequences that
  follow: measurement age is the ceiling on achievable stabilisation; a
  frame or sign error turns stabilisation into amplification on a worn
  arm, so frames stay explicit and honestly named; corrections must fade
  with estimate age and re-engage smoothly, never step; mount flex between
  marker cluster and arm base is an invisible error floor that stage 0's
  rigidity captures exist to measure. (Prompt: raw-prompt-log 2026-08-13
  15:35:39; reaffirmed 15:43:20.)
- World-frame wiring, first slice scoped 2026-08-13 (refines the staged
  Vicon goal): the controller reads Vicon on its own thread behind a
  stub-able provider, so the standalone build ("No GTSAM / GPMP2 / Vicon
  required") stays true; log columns and reader code are written only
  after the stage 0 lab recording, honouring the 2026-08-12
  data-before-format decision; until stage 2 calibration exists the
  columns carry raw segment poses named for what they are
  (world_T_<segment>, plus an age column), never presented as mount
  poses; and all five Nexus segments are logged, not just the mount —
  recordings are thesis evidence, disk is cheap, lab time is the scarce
  resource. At this stage the world pose is observed and recorded only; it
  enters no control law. (Prompts: raw-prompt-log 2026-08-13 15:24:35,
  15:43:20; the four choices and one prediction miss recorded in
  docs/intent/predictions.md the same date.)
- Every limit that can fault the arm is stated from Kinova's documentation
  from first principles — velocity, acceleration, jerk, anything
  fault-capable — fixed as hard boundaries the controller cannot cross,
  with normal operation about 5% below them; and the map of where every
  current limit comes from is written on paper before anything is changed.
  In his words: "I would like to state those limits clearly and put those
  as like hard limit boundaries that the controller could not go. And then
  I will say anything other than that, perhaps 5% below." Survey delivered
  as Christian_control/docs/motion-limits-map.md; found three gaps no
  document can close (firmware thresholds unread, torque tables
  unextracted, no low-level acceleration figure) and one latent validator
  bug (max-vs-max limit comparison) that goes live when limits split
  80/70. (Prompt: raw-prompt-log 2026-08-13 15:32:43 and the interactive
  answer ~15:35 recorded in docs/intent/predictions.md.)
- A target beyond what the arm can reach cleanly gets the nearest
  achievable pose plus an honest report of the shortfall — never a stop,
  never a silent substitution. Chosen ~16:10 after the far-target
  diagnosis; the offline IK evidence showed "nearest" must be nearest in
  pose space (the failed 2026-08-05 target was fully reachable in
  position if ~45° of tool pitch was yielded), so the projection must be
  able to trade orientation for position and say so. Root cause and
  evidence: Christian_control/docs/thesis/far-target-joint-limit-stop.md.
  (Prompts: raw-prompt-log 2026-08-13 16:04/16:05; choice recorded in
  docs/intent/predictions.md, both predictions hit.)

- The scientific project is designing **and evaluating** a system that
  maintains an end-effector pose in the external world while its
  supporting base moves — Christian's own definition, correcting "the
  gap is the project" as too narrow. Components he named: world-state
  estimation, frame calibration, reference generation, motion
  compensation, latency handling, occlusion and dropout handling,
  safety, quantitative evaluation, later dual-arm and planning. "Simply
  wiring the systems together proves integration. It does not yet prove
  successful stabilisation." Evaluation is a deliverable, not a
  by-product. (Prompt: raw-prompt-log 2026-08-13 17:26.)
- World-on-hardware architecture, decided 2026-08-13 ~17:35 (interactive
  questions with implementation previews; predictions ledger same date):
  **Architecture A** — the 500 Hz loop assembles world-frame inputs
  (T_W_B from ZOH'd Vicon sample × calibration, world-rotated Jacobian,
  world pose error) and feeds the unchanged frame-agnostic law, mirroring
  msc_project controller/frames.py, which he built and trusts ("i done
  it pretty well there so it can be a rough estimate"). The sim is the
  reference architecture. Timing gate for the first hardware run: the
  strict minimum — full snapshot contract (frame number, sequence,
  receive timestamp, reported latency, pose, validity), zero-order hold,
  never differentiate a reused sample, stale ⇒ freeze the world
  correction at the last good value with graded logged freshness, never
  a stop or a step. His line, now a design rule: "A controller that uses
  stale Vicon data without knowing it is stale is more dangerous than
  one that has no Vicon integration." Packaging: two slices — slice 1
  acquisition+logging+panel with no control change and explicit exit
  criteria; slice 2 the pose-only world hold behind its own design gate.
  (Prompt: raw-prompt-log 2026-08-13 17:26; answers same hour.)
  *Status, 2026-08-13 evening:* slice 2 was approved ("go ahead", 19:10:58),
  committed and taken to hardware ("commit slice 2 and lets run it",
  19:42:41). The first run exposed a dispatch gap — production always
  carries a joint reference, so the hold was unreachable — fixed the same
  night as `02348ecc` (world hold engages through the idle joint hold);
  a re-run was authorized after the fix (interactive answer "run it",
  recorded in docs/intent/predictions.md the same date). Three format-11
  run logs from 19:45–19:52 are the evidence on disk.
- The simulation is the reference implementation of world-frame
  control, not an estimate: "You're treating world hold as a new control
  problem, but it isn't." Hardware work maps msc_project's implementation
  (frames.py, reactive_controller.py, runner.py cycle order) onto
  hardware rather than re-deriving; genuinely new logic is limited to
  what the sim never needed — world_T_B from Vicon, the Mount-to-model
  frame relationship, 100/500 Hz handling, validity/staleness/dropout,
  and reference arbitration. Every remaining difference must be called
  out with its reason or eliminated in favour of the sim convention.
  Caught in review of the hold-slice design v1, which had re-based the
  law inputs at the seam; v2 carries the side-by-side mapping. (Prompt:
  raw-prompt-log 2026-08-13 19:02.)
- The world-frame hold's first hardware slice uses feedback only — "Its a
  PD velocity controller but i want to establish world on hardware using
  the vicon. which uses only feedback." Establish the world frame in the
  loop first; the base-twist feedforward V_base,E (his section-6 math,
  2026-08-13 images) stays the target law but is deferred, not dropped.
  This answers the 2026-08-12 spec's open question 3 for the first slice.
  (Prompt: raw-prompt-log 2026-08-13 16:25; interactive answer same turn.)
- Targets are defined relative to the wearer's torso, and the torso is a
  separate tracked segment from the mount plate — control runs through
  the mount, target specification through the torso (`p_d^T, R_d^T`,
  transformed once into world). Six tracked bodies, not five; the segment
  inventory in the staged-Vicon goal above widens accordingly, and the
  mount-rigidity experiment gains a second purpose: measuring
  mount-vs-torso flex. (Interactive answer, 2026-08-13 ~16:30, to a
  direct frame question; his notation table names T throughout.)
- Debugging happens inside the panel: the run-evidence plots are generated
  and viewed from the browser, per run, "so I can debug within the UI"
  (his words). The sequence was: he asked what graphs the project needs
  (19:56:53, 19:59:42), approved building the plotting scripts (20:02:35),
  then asked for them in the UI — twice, nearly verbatim (20:12:25,
  20:41:20) — which reads as emphasis, not accident. Built the same
  evening as the panel's PLOTS tab plus plot_world_hold.py (uncommitted,
  2026-08-13). Consistent with the earlier one-live-chain choice: the
  panel reads what the controller logged, it computes nothing of its own.
  (Prompts: raw-prompt-log 2026-08-13 19:56:53 through 20:41:20.)
- A reusable Vicon recording exists for testing: "can you record vicon
  data for 30 seconds so i can reuse for testing" (20:46:48). Recorded
  goal: a short captured stream that tests can replay offline, so Vicon
  code is exercisable without the lab. My reading of how this coexists
  with the "live stream only, no captures" supersession of 2026-08-13
  ~16:40: that decision removed captures as the *calibration data path*;
  this is a *test fixture*, which the recorder was explicitly kept for
  ("tested, and useful for synthetic tests") — not a reversal. If that
  reading is wrong, the supersession entry needs revisiting instead.
  (Prompt: raw-prompt-log 2026-08-13 20:46:48.)
- The written record of the system describes the current implementation,
  not the historical one. Commissioned 2026-08-14 in detail: current
  verified architecture, a gate/limit/stop inventory, the observability
  surface, and a worked read-only debugging method with a
  dissertation-ready causal template — with three standing rules in his
  own framing: historical audits are preserved as evidence, never
  silently overwritten or reinterpreted; source evidence stays separate
  from assumptions and from physical-robot proof; and nothing
  robot-facing runs for documentation work. Existing published artifacts
  are updated in place rather than duplicated — asked where a new audit
  file should live, he redirected to the artifact he already had ("You
  previously created an artifact with something like this. Can you check
  it?"), then extended the same treatment to the second one ("update the
  architecture map artifact too"). Delivered 2026-08-14:
  docs/architecture_and_debugging_audit.md, a refreshed
  docs/architecture.md, and both artifacts brought to HEAD 02348ecc.
  (Prompts: raw-prompt-log 2026-08-14 13:06:50/13:07:06, 13:57:40;
  redirect recorded in docs/intent/predictions.md 2026-08-14.)
- Working mode, standing: equations before code, always. Every
  control-behaviour change is presented first as the equation that
  changes, in his notation, then as the code diff realising it, with a
  file:line map; a derivation doc in the repo ties the full law to code
  symbols. His stated why: "i am not able to see what and when you create
  changes" without the math — the math is how he audits the work and how
  the thesis gets written. (Prompt: raw-prompt-log 2026-08-13 16:25;
  interactive answer same turn.)
- The final planner/controller architecture has one production world-frame
  Cartesian pose/twist controller. GPMP2 remains internally joint-space and
  unchanged, but the world-aware planner application uses a fresh
  `world_T_mount` snapshot and densely projects its final validated
  `q(t), qdot(t)` through world FK and `J qdot`; only timed world-frame
  end-effector pose/twist, frame, timing, and Vicon provenance cross the
  boundary. No planned joint posture crosses it. The controller follows the
  existing Python simulation law exactly: filtered measured Mount twist is
  transported into measured end-effector world twist, while planned twist
  appears through Cartesian twist error, with no separate base-motion
  feedforward term. Planning remains asynchronous and outside 500 Hz; newer
  requests coalesce, valid old references remain until atomic replacement,
  and brief/prolonged Vicon loss pauses or cancels/re-anchors explicitly.
  The existing Kinova safety/actuation path remains common. Christian's why:
  two production controllers are difficult to explain when the intended
  system behaviour is one world-frame Cartesian objective. Design approved
  2026-08-15; spec at
  docs/superpowers/specs/2026-08-15-world-cartesian-planner-controller-design.md.
  Implemented in the approved working-tree migration on 2026-08-16 and
  verified only with hardware-free builds/tests; physical world tracking,
  dropout tuning, calibration, and person-nearby safety remain unproven.
  (Prompts and interactive approvals: session transcript, 2026-08-15 through
  2026-08-16.)
- HumanSL has one dual-arm simulation execution twin inside `HumanSL_MAIN` so
  planner/controller ideas can be tested interactively before hardware. It
  shares the exact hardware-independent C++ execution core with production:
  world measurement, Cartesian reference handling and law, generic software
  safety, limits, position integration, and telemetry semantics. A separate
  `humansl_sim` target can never connect to Kortex; MuJoCo and simulated Vicon
  replace only the hardware boundaries. Scope includes the full GPMP2-to-world
  Cartesian pipeline, atomic dual-arm activation, mid-run replanning, exact
  HumanSL frames/TCPs, 500 Hz control, switchable ideal or 100 Hz realistic
  Vicon, scripted Mount motion, panel integration, parity/experiment tuning,
  and optional shared obstacles disabled by default. Generic actuator dynamics
  are accepted; Kinova firmware emulation, functional grippers, recorded/manual
  Mount motion, and claims of physical proof are excluded. The existing Python
  simulation remains an independent comparison implementation. Sharing the
  execution core necessarily refactors the hardware executable: pre-extraction
  trace characterization and post-extraction replay equivalence are gates, and
  "hardware parity" means code/configuration parity only until a separately
  authorized physical revalidation. The pose/twist-only boundary remains
  binding after review: planned GPMP2 posture does not cross, so planned
  collision/inter-arm clearance is planner-path evidence while executed
  clearance is monitored in simulation, never claimed as guaranteed.
  Christian's why: he needs to see whether ideas he implements work correctly
  in simulation before taking them to hardware. Design approved interactively
  and review correction confirmed on 2026-08-16; spec at
  docs/superpowers/specs/2026-08-16-humansl-execution-twin-design.md.
  (Prompts and interactive approvals: session transcript, 2026-08-16.)

- The Pinocchio model wrapper is named for what it does. Christian's own
  conclusion (raw-prompt-log 2026-08-17 20:22): the `Dynamics` class was
  never doing dynamics — no mass/Coriolis/gravity call exists anywhere in
  the controller; it existed to load and hold the URDF model for
  kinematics, a `RobotModel` with a misleading name inherited from
  TrajectoryExecution's impedance work. The velocity-level controller
  needs no rigid-body dynamics; a real dynamics component returns only
  if torque/impedance/operational-space control does, and git history of
  Dynamics.cpp keeps the removed methods. He chose the full slice — record
  + build dedup + rename — over the staged options (interactive question,
  2026-08-17 evening session; prediction log same date). Companion fact,
  same session: the build-memory failure traced chiefly to thirteen
  targets each recompiling Dynamics.cpp/Kinematics.cpp instead of linking
  humansl_execution_core once; the dedup was approved in the same answer.
  (Questions asked out of order with prediction logging twice that session
  — recorded as process misses in predictions.md.)

- Requests to the planner get answers, not vetoes, and requested targets
  are visible in space before hardware time. Christian's framing, after
  three supervised sessions were lost to rejected circle plans on
  2026-08-17: "it is too easy for the planner to reject a trajectory
  I've requested, and it's too easy for me not to know where the points
  I've requested are in space." Confirmed the same evening via
  interactive questions: (1) targets can be authored as poses fixed in
  the room — "the now target should be in world pose so now we can get
  a real error" — so wearer motion produces genuine stabilisation
  error rather than a silently relocated goal (the 174 deg mount
  re-hang between 14:10 and 15:59 relocated the mount-declared circle);
  (2) all three capabilities are designed together: pre-flight offline
  validation on goal edit, a spatial view of the requested path against
  reach and current pose, and graded planner output that emits the best
  safety-respecting trajectory with an honest shortfall report while
  safety-critical checks stay hard. Design drafted, not yet approved:
  docs/superpowers/specs/2026-08-17-world-targets-preflight-graded-planning-design.md.
  (Prompts: raw-prompt-log 2026-08-17 20:44:40, 23:36:16; interactive
  answers recorded in docs/intent/predictions.md the same evening.)

- The repository's shape must be evidence of what the system does, not of
  how it came to be. Christian's outcome, in his words: the filesystem
  should reflect "the actual engineering architecture", each top-level
  area should be named from "what the code actually does rather than
  forcing these names", and "a normal execution path should be
  understandable from roughly 3-6 meaningful files". He named the two
  paths that must be visible: tracking to world/base state to controller
  to actuation to hardware, and planning request through
  planner/IK/GPMP2/collision to trajectory to runtime. He also named four
  architectural decisions the shape must preserve rather than re-open:
  one URDF geometric model, Pinocchio as the canonical FK/Jacobian
  implementation, GPMP2 owning optimisation rather than a robot model,
  and external world/Vicon transforms staying distinct from robot
  kinematics. Explicitly bounded as structural only — no change to
  control mathematics, planner behaviour, safety behaviour, numerical
  constants, APIs, or hardware behaviour — and he required the design to
  be shown and falsified before anything moved. Delivered 2026-08-18:
  eight subsystems (model, contracts, control, runtime, tracking,
  planning, simulation, panel), both flows readable end to end, and the
  same test counts before and after (22 controller-side, 22 planning, 6
  tracking, 9 simulation, 289 panel). This entry records the standing
  principle, not the one migration: a directory named after a vendor, an
  interface, or its own history is a defect to be fixed when found. It
  generalises the naming principle Christian reached himself one day
  earlier for the Pinocchio model wrapper (see the RobotModel entry
  above) from a class to a directory. (Prompt: raw-prompt-log 2026-08-18
  19:47:03. Three naming choices confirmed via interactive questions the
  same evening: control/ over execution/, runtime/ over hardware/,
  top-level model/ over control/model/.)

## Interpretations (hypotheses)

"He asked for X, likely because Y" — cited, awaiting confirmation.

- ~~**The why behind "as fast as possible" is probably keeping up with the
  wearer.**~~ **Refuted 2026-08-13** by Christian directly: the reason is
  demonstrating the hardware's full capability in the thesis. Kept rather
  than deleted because the error has a pattern worth remembering — the
  reasoning ran from the *system's* purpose (track a moving wearer) to the
  *request's* purpose, when this is MSc work and a thesis has goals the
  control system does not. Requests that serve the degree rather than the
  running system will keep being misread this way otherwise.
  *Amended ~15:35 same day:* asked directly, Christian selected all three
  whys — tracking the wearer, characterising the platform, and demo
  credibility — so "refuted" was itself too strong: tracking is among the
  reasons, it is just not the only one. The pattern note above stands.
- **The 2026-08-14 audit request is preparation for a tuning day, plus
  thesis defence.** Likely because: the world-hold work of 2026-08-13
  inverted the two headline facts of every existing architecture document
  within twenty-four hours (the "dead" Cartesian channel went live, "no
  world-frame feedback" became a wired Vicon path), a dozen code comments
  and the operator-facing startup print still said the opposite, and the
  next session is log-driven tuning (the Day 301 video, 20:40:42) — so
  stale documentation was about to be actively misleading during hardware
  work. The dissertation-template deliverable points the same way: he must
  defend every diagnosis in the report. Awaiting confirmation. (Prompts:
  raw-prompt-log 2026-08-14 13:06:50; 2026-08-13 20:40:42.)
- **The frustration at 14:56:57 and 14:59:04 was about process that
  produced nothing, not about process as such.** In the same session a
  one-line question ("why are there such bounds") drew a fifteen-agent
  workflow whose answer was already written in a comment Christian had
  authored himself in commit 77991493. Minutes later he approved building
  three new mechanisms without hesitation. The distinguishing feature is
  whether the ceremony produces something he did not already have.
  (Cited to raw-prompt-log 2026-08-13 14:38:07 through 15:08:39.)

## Superseded decisions

Past decisions Christian has explicitly overridden. The old decision and
its reasoning stay recorded — never deleted — so the history of why the
system's guardrails are what they are is never lost, only superseded.

- **Guard-override editability, 2026-08-12.** Prior decision (2026-08-05,
  `compiled-config-guard-overrides` session memory; encoded in
  `tools/panel/config_file.py`'s `GUARD_OVERRIDES` tuple and docstring:
  "safety policy must not be one click away"): `kStopOnFault`,
  `kAllowUnverifiedActuators`, `kSkipStartupGates`,
  `kDisableFollowingErrorStop` are excluded from the panel's write
  whitelist. Superseded: Christian confirmed (via two interactive
  questions, once for the group of three explicitly-labelled "Guard
  overrides," once specifically for `kStopOnFault` after I surfaced that
  its Config.h wording — "never runtime-settable" — and its placement
  outside the labelled Guard-overrides comment block made it a possibly
  harder line) that all four should become editable from the panel. No
  why was volunteered beyond wanting full config control from the UI;
  none was pressed for, per "ask at most one plain-English question, then
  comply." If a future session needs the reasoning behind this
  supersession and it is not recorded elsewhere, ask Christian directly
  rather than assume.
- **Stage 0's two captures, 2026-08-13.** Prior decision (2026-08-12
  design, reaffirmed in the stage-0 runbook): capture a static and a
  wearer-moving recording as stage 0's deliverable, feeding stage 2's
  calibration and the mount-rigidity answer. Superseded 2026-08-13
  ~16:40: back from the lab, Christian chose "live stream only, no
  captures" with the cost stated at the moment of choice (calibration
  and the rigidity check lose their recorded input). Consistent with his
  route decision earlier the same day: one live chain, Vicon →
  controller → log → panel, where the controller's own log is the
  de-facto recording once stage 1.5 lands. Consequences: stage 1's
  "stage-0 recording replays offline" acceptance is void as written;
  stage 2's calibration must sample the live stream (or a stage 1.5
  controller log) when it runs; the recorder/replay code stays (tested,
  and useful for synthetic tests) but is no longer the data path.
  (Prompt: raw-prompt-log 2026-08-13 16:38; interactive answer ~16:41.)
- **Stage 0 before stage 1 code, 2026-08-12.** Prior decision
  (interactive question, 2026-08-12 design session): run the lab
  verification before writing stage 1, so real data informs the snapshot
  format. Superseded the same evening by Christian's approval of the
  combined stage 0+1 plan (raw-prompt-log 2026-08-12 16:20:43 "I've
  reviewed it, and it's pretty good"; 17:12:41 "Go with option 1") — the
  reconciliation being that stage 0's recordings need the recorder to
  exist, so the recorder came first. Reading confirmed by Christian
  2026-08-13 16:07. The residue is honest: the snapshot format predates
  real data, and the guard is stage 1's acceptance test that a stage-0
  recording replays offline — if the lab exposes a format problem, stage 1
  is revised, never the recording bent to fit.
- **World-frame slices on the old boundary, 2026-08-12 rollback.**
  Slices 1–4 of the 2026-08-10 world-frame architecture were reverted
  (d04b2035). The revert removed an implementation on the wrong boundary;
  it did not abandon the world-frame goal, which re-entered eleven days of
  work later as the staged Vicon integration (2026-08-12 spec) and was
  restated as the master goal in Christian's own words on 2026-08-13.
  Anyone reading git history should read that revert as "wrong boundary
  corrected", never "goal dropped" — for the thesis it is the
  found-and-fixed part of the story.
- **Joint trajectory as the planner/controller boundary, 2026-08-12
  rollback through 2026-08-15.** The rollback correctly removed a
  controller-side conversion carrying joint posture, and the repository then
  retained `q_ref(t), qdot_ref(t)` as the production boundary. Superseded by
  Christian's 2026-08-15 approval: GPMP2 may retain joints internally, but the
  planner application now owns dense world FK/Jacobian projection and exposes
  only timed world end-effector pose/twist. The old reason survives — keep
  conversion on the planner side and do not smuggle posture into control —
  while the boundary type changes.
- **Feedback-only world hold with zero measured base twist, 2026-08-13.**
  This was deliberately the first hardware slice for establishing world pose.
  Superseded for the final architecture on 2026-08-15: measured Mount velocity
  is now required to calculate measured end-effector world twist accurately.
  This is not a separate explicit feedforward term; it is the same measured
  twist construction used by Christian's Python simulation. Planned twist
  enters through twist error only.
- **Mechanical steward reminders, 2026-08-13.** Prior decision
  (2026-08-12, Exposure log below): a per-prompt steward reminder injected
  by hook was dismissed with "CLAUDE.md is enough". That dismissal named
  its own re-open condition — "only if sessions visibly forget to
  why-check new intent." The condition was met on 2026-08-13: the session
  opened, took substantive work, and never read `story.md` at all, with
  nothing to catch the omission until Christian noticed it himself.
  Superseded: three reminder hooks now report staleness at session start,
  unreviewed story diffs at session end, and the prediction obligation
  before each question. The dismissal's substance still stands in one
  respect — none of them fires per-prompt, and none blocks. (Prompts:
  raw-prompt-log 2026-08-13 14:56:57 and 14:59:04; approved 15:08:39.)

## Open questions

Unconfirmed whys and ambiguities, ranked by (chance I'm wrong) x (cost if
wrong). Proceeding on anything listed here must be said out loud.

- ~~**`min_duration_s` blocks the confirmed goal**~~ / ~~**Does Christian
  want the commanded clip raised now?**~~ / ~~**Should
  `motion.min_duration_s` change?**~~ **Resolved 2026-08-13** by the
  limits decision already recorded under Approved goals (~15:32/15:35):
  the working tree now carries the clip at 76.0/66.5 deg/s (95% of the
  live hard limits), `min_duration_s` at 1.0 s (down from 4.0), and the
  planner's velocity table raised to match, with the per-joint validator
  fix that the split limits made necessary
  (Christian_control/docs/motion-limits-map.md, "Applied" section, and
  docs/decisions/qdot-limit-raise.md as modified in the working tree).
  Residue worth keeping: all of it is **uncommitted**, and every built
  `controller` binary predates it — so the decision is not on the robot
  until a rebuild, and `qdot-limit-raise.md`'s safety note (small nearby
  targets, e-stop in hand) applies to the first runs after.

- **What does the "Day 301 decreasing test" video test?** Recorded on the
  Vicon PC on 2026-08-13 evening "so I can basically tune it tomorrow"
  (20:40:42, 20:44:26) — but what "it" is (which behaviour the video
  captures, and which parameter gets tuned from it) was never stated.
  Cheap to ask at the start of the tuning session; guessing wrong wastes
  the recording.
- **Was the 30 s Vicon test recording actually captured, and is it the
  blessed fixture?** The request (20:46:48) came at session end; the vicon
  build tree shows output activity at 20:47, but no recording has been
  confirmed as the standing test input.

- **Why the reorganisation, and why now?** The 19:47 prompt states the
  outcome but not the cost that prompted it. My reading, from the same
  day's prompts: the layout had started costing Christian answers about
  his own system. At 14:29:44 he asked "how is redundancy handled in my
  code", and at 14:57:50 "I have 2 forward kinematic equation that can
  cause errors because it does not mean the both equal each other" —
  both are questions the structure should have answered and did not.
  The second is pointed: the repository does hold two forward-kinematics
  implementations, Pinocchio's in the controller and the DH-based one in
  the GPMP2 layer, and nothing in the old directory names said which was
  canonical. Five hours later he asked for a layout that makes exactly
  that answerable, and named "Pinocchio as the canonical FK/Jacobian
  implementation" as a decision to preserve. I think the why is
  therefore: he must be able to explain and defend this architecture in
  his MSc report, and a structure he has to re-derive each time is a
  structure he cannot defend. Proceeding on this reading was not
  necessary for the migration itself, which was fully specified — but it
  is the reading that decides how aggressively future naming defects
  should be chased. A yes or no is enough.

## Examples

Concrete anchors: one case that must work, one that must not, per goal.
These become acceptance criteria in Christian's words.

- **Must not happen:** a supervised hardware session reaching takeover
  hold and then dying on a configuration typo. On 2026-08-13 the left arm
  connected, passed the kinematic hard-limit and joint-limit gates, held
  at its startup pose, and only then did the planner reject
  `nominal_speed_mps: 0.5` — so the run was spent without a plan ever
  being sent. Configuration that can be checked before the arm is touched
  should be. (Observed in raw-prompt-log 2026-08-13 14:38:07.)
- **Must work:** a limit Christian changes has a visible effect, or says
  why it does not. `nominal_speed_mps` currently fails this — at any value
  above 0.25 m/s it changes nothing, because the 4.0 s duration floor
  dominates every reachable move. (Derived from PlanSolver.cpp:40-41
  against planner.yaml; prompted by raw-prompt-log 2026-08-13 14:38:07.)

## Exposure log

Options shown to Christian from beyond his request: adopted or dismissed,
and why. Dismissals are binding — do not re-pitch.

- 2026-08-12: per-prompt steward reminder injected by hook (mechanical
  nudge beside every prompt). Dismissed — "CLAUDE.md is enough". Re-open
  condition named by the dismissal itself: only if sessions visibly
  forget to why-check new intent.
- 2026-08-12 (Vicon design session): three transport options for getting
  Vicon into the controller were shown. Adopted: provider seam + replay
  (live/replay/static implementations behind one interface). Dismissed
  for the control path: a separate sensing process feeding the controller
  (extra latency hop + serialization format) — it survives only as panel
  diagnostics per the 2026-08-11 monitor spec; and a seamless in-process
  SDK link (untestable without the lab). Also adopted: planner scope
  limited to plan-time frame conversion (replanning deferred), and
  minimal recorder+replay infrastructure in stage 1.
- 2026-08-13: four enforcement strengths for the steward mechanisms were
  shown — remind-only, block-until-current, pull-only slash commands, and
  a mixed tier. Adopted: remind, never block. Dismissed: a Stop hook that
  refuses to end a session while the story is stale, on the grounds that a
  record which can refuse Christian has become an authority over him,
  which the story's own rules forbid.
- 2026-08-13: offered scoping the build down to the tier-1 mechanism
  alone (staleness and diff reminders), deferring prediction and red-team
  as the design's own tiering suggests. Dismissed — all three were built.
  Worth remembering as a calibration data point: reluctance to spend
  Christian's time is not the same as reluctance to build him something
  (docs/intent/predictions.md, 2026-08-13).
- 2026-08-13 (world-frame wiring scoping): four routes for Vicon into the
  panel were shown — replay-only display, controller logs the world pose
  it used, the panel opening its own DataStream client, explain-and-defer.
  Adopted: controller logs world pose. Dismissed: the panel's own client,
  because it would display a pose sampled separately from anything control
  acted on, and the two disagree exactly when the base moves — the only
  time it matters. Also shown: mount-segment-only logging (recommended as
  the minimal surface) versus all five segments — all five adopted, on the
  grounds that an unused control is clutter but an unrecorded segment is
  lost thesis evidence.
- 2026-08-15 (single-controller architecture): considered (1) retaining the
  joint-trajectory controller beside Cartesian hold, (2) exporting world
  Cartesian pose/twist from the planner application while GPMP2 remains
  internally joint-space, and (3) changing the planning problem to operate
  directly in world coordinates. Adopted a combination of (2) and a
  world-aware planner application: a fresh `world_T_mount` snapshot makes the
  solve and export world-consistent, without changing GPMP2 internals.
  Dismissed retaining two production controllers because it obscures the one
  Cartesian system objective.
- 2026-08-15 (base motion and redundancy): adopted the Python simulation's
  measured-world-twist construction using filtered Mount translation and
  rotation; dismissed a separate explicit base-twist feedforward term.
  Considered an offline resolved-rate rollout validator to check whether the
  posture actually executed from Cartesian references preserves the planned
  collision clearance. Dismissed from this migration: Christian accepted the
  limitation that a pose/twist-only boundary cannot guarantee GPMP2's 7-DoF
  joint branch, while retaining null-space joint-limit avoidance and honest
  documentation of that limitation.
- 2026-08-16 (execution twin): considered (1) a shared hardware-independent
  execution kernel with explicit Kortex/Vicon and MuJoCo/simulated-Vicon
  adapters, (2) fake Kortex and Vicon devices around the hardware executable,
  and (3) a separate simulation runner sharing only controller components.
  Adopted (1) because it shares the behaviours under test while keeping the
  simulation physically separate from robot I/O. Dismissed (2) because
  protocol/firmware emulation is costly and misleading, and (3) because timing,
  reference, safety, and integration logic would drift.
- 2026-08-16 (simulation scope): adopted full dual-arm GPMP2 integration,
  exact HumanSL frame/TCP parity, switchable ideal and 100 Hz realistic Vicon,
  scripted repeatable Mount motion, asynchronous replanning, full panel
  integration, and explicit hardware-parity/experiment modes. Shared obstacle
  support is included but defaults off. Dismissed for this version: identified
  Kinova actuator dynamics, firmware acknowledgement emulation, functional
  grippers/object manipulation, recorded Vicon replay, and manual Mount motion.
- 2026-08-16 (external spec review): the review exposed that extracting a
  shared core is also a hardware refactor, generic MuJoCo tracking metrics do
  not predict Kinova performance, live asynchronous planner completion cannot
  drive deterministic thresholds, and long world holds can become infeasible.
  Adopted trace-replay gating, explicit pending hardware revalidation,
  physics-substep freedom under a fixed 2 ms control period, deterministic
  planner-output injection for numeric acceptance, and explicit hold-stop and
  Vicon-noise semantics. Considered passing GPMP2 `q(t)` as a low-priority
  null-space posture bias. Christian explicitly kept the pose/twist-only
  boundary; therefore planned clearance is labelled planner-path evidence and
  executed clearance is monitored rather than guaranteed. (Pasted review and
  interactive choice, session transcript 2026-08-16.)
- 2026-08-17 (Pinocchio warnings and the Dynamics split): options shown
  for the `-Wmaybe-uninitialized` noise (extern-template instantiation,
  targeted suppression, both, leave) were all rejected — Christian
  corrected the premise instead: the warning is real, caused by how
  Pinocchio's mimic-joint code is written (`nv()`==0 at run time vs
  `NV`==1 at compile time; unreachable for our mimic-free URDFs). The
  false-positive framing was the agent's verification failure, logged in
  predictions.md. Options for the follow-up (record only / + CMake dedup /
  + full RobotModel split / nothing): adopted the full slice; job-count
  cap in the panel explicitly deferred ("lets wait for now"). Upstream
  bug report to Pinocchio remains open as an option nobody has taken.
- 2026-08-18 (repository reorganisation): three naming choices were put to
  Christian rather than decided for him, each with the argument for the
  alternative — `control/` versus `execution/` (the latter matching the
  existing `humansl_execution_core` target name), `runtime/` versus
  `hardware/` (the latter blunter about which binaries can reach the arm),
  and a top-level `model/` versus folding the URDF into `control/model/`.
  He adopted the recommendation in all three. Shown and dismissed as moves
  in their own right, each for a stated reason, and binding unless he
  re-opens them: merging `RobotModel.*` back into `Kinematics.*` (reverses
  his own 2026-08-17 decision); splitting `Config.h` into robot description
  and controller tuning (correct in principle, and the reason `planning`
  still depends on `control`; deferred as a content change, not because
  the line is unclear — every one of the nine constants planning's
  production code reads is robot description: the four frame names, the
  two nominal joint vectors, the reference-frame switch and its names,
  and `kJointSoftwareLimitDeg`. The real coupling is that the panel edits
  this file by regex against a name whitelist and `WriteConfigLines`
  embeds it in every run's CSV preamble, so a split must not move a
  whitelisted name out from under either. An agent claim on 2026-08-18
  that a panel-writable knob is read by the planner was wrong and is
  corrected here: `kModelVelocityLimitsDegS` is read only by
  `planning/tests/test_path_validation.cpp`);
  merging the three confusably-named planning validators (the defect is
  their names, and fixing it means renaming C++ types, which the prompt
  excluded); splitting `BridgeMain.cpp` and `optimisation/utils.*`;
  a top-level `CMakeLists.txt` (the per-project build is a recorded
  decision); renaming the `Vicon*` files (those names are correct — only
  the directory was vendor-named); renaming the CMake targets; a top-level
  `analysis/` area for the run-log scripts; and deleting the unreferenced
  `GEN3_custom.urdf` (unreachable is not obsolete). Also shown, not
  adopted, and worth remembering: only ONE file merge survived scrutiny
  (`Arrival.h` into `Controller.h`), because most small files in this
  repository are small so that a dependency-light test can link — the
  request to "reduce file fragmentation" was answered by separation, not
  by merging, and the honest count was reported rather than padded.
  (Prompt: raw-prompt-log 2026-08-18 19:47:03, which asked for rejected
  moves and for the design to be falsified.)
- 2026-08-21 (panel scene editor): adopted keeping the named static scene
  inside the existing planner YAML, with the panel as editor/renderer and
  `MakeMountSdf()` as the sole collision conversion. Considered and dismissed
  for the first slice: a separate `scene.yaml` (extra runtime/archive/digest
  path before scene size justifies it) and invoking a planner executable on
  every drag (a process boundary on the interaction path). Deferred rather
  than dismissed: freezing a running session onto its archived planner config,
  which is stronger than the adopted panel save gate but changes runtime
  configuration ownership. Also adopted: retire the per-arm goal box rather
  than preserve two obstacle-definition paths; first-slice primitives are
  mount-axis-aligned boxes and finite +Z cylinders, while planner-exported
  collision spheres remain a later diagnostic slice rather than copied into
  JavaScript. (Explicit approval in the current 2026-08-21 session; design:
  `docs/superpowers/specs/2026-08-21-panel-scene-editor-design.md`.)
