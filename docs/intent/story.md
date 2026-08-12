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

## Interpretations (hypotheses)

"He asked for X, likely because Y" — cited, awaiting confirmation.

*(none yet — capture began 2026-08-12)*

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

## Open questions

Unconfirmed whys and ambiguities, ranked by (chance I'm wrong) x (cost if
wrong). Proceeding on anything listed here must be said out loud.

*(none yet)*

## Examples

Concrete anchors: one case that must work, one that must not, per goal.
These become acceptance criteria in Christian's words.

*(none yet)*

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
