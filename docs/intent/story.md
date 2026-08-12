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

## Interpretations (hypotheses)

"He asked for X, likely because Y" — cited, awaiting confirmation.

*(none yet — capture began 2026-08-12)*

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
