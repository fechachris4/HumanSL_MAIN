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
- Working mode, standing: equations before code, always. Every
  control-behaviour change is presented first as the equation that
  changes, in his notation, then as the code diff realising it, with a
  file:line map; a derivation doc in the repo ties the full law to code
  symbols. His stated why: "i am not able to see what and when you create
  changes" without the math — the math is how he audits the work and how
  the thesis gets written. (Prompt: raw-prompt-log 2026-08-13 16:25;
  interactive answer same turn.)

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

- **`min_duration_s` blocks the confirmed goal, and nobody has decided
  what to do about it.** With the 4.0 s floor, a 90 degree joint move
  peaks near 22 deg/s — so the arm cannot approach 79.64 deg/s however
  high the velocity clip is set. Demonstrating full capability therefore
  needs the floor lowered, not just the clip raised, and a short fast move
  is a different hardware risk from a long slow one. Ranked highest here
  because the goal is confirmed while the means to it is not, which is
  exactly the state that invites someone to act without asking.
- **Does Christian want the commanded clip raised now, and to what?** It
  sits at 50 deg/s (raised from 45 in the working tree, uncommitted)
  against live hard limits of 80.0021 and 70.004 deg/s.
  `qdot-limit-raise.md` says raising it back toward the table limits "is a
  deliberate decision, not a cleanup", and its safety note asks for small
  nearby targets and an e-stop in hand on the first runs after. Asking
  costs one question; guessing wrong moves a real arm faster.
- **Should `motion.min_duration_s` change?** At 4.0 s it, not
  `nominal_speed_mps`, is what actually paces every reachable
  point-to-point move: the distance term only exceeds the floor beyond
  1.0 m at the current 0.25 m/s cap. Christian asked about the speed knob
  he had found; he may not know it is inert at that setting. Recorded here
  rather than acted on because it is a motion-path behaviour change.

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
