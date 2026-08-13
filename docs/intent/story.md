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
  The why, confirmed: to demonstrate in the thesis that the hardware was
  driven to its real capability. Peak speed is the objective itself, not a
  means to tracking performance — so the target is the Kinova table limits
  (79.64 deg/s joints 1-4, 69.91 deg/s joints 5-7), and success is
  measured by what the arm was shown to reach, not by tracking error.
  (Prompt: raw-prompt-log 2026-08-13 14:38:07, repeated 14:41:05 and
  15:12:06/15:12:35; why confirmed via interactive question 2026-08-13
  ~15:20.)
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
