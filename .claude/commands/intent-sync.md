---
description: Fold newly captured prompts into docs/intent/story.md as cited hypotheses, then show Christian the diff
---

Bring the interpretation layer up to date with ground truth, then hand
Christian a diff to approve.

## What to read

1. `docs/intent/story.md` in full — its rules, and what is already recorded.
2. `docs/intent/raw-prompt-log.md`, but only the entries that postdate
   story.md's last commit. Find the boundary with:
   `git log -1 --format=%ad --date=short -- docs/intent/story.md`

Read the repository and git history before spending Christian's time. A why
you can find yourself is not a question worth asking him.

## What to write

Add entries to the section that fits: **Approved goals** only for things
Christian explicitly confirmed, **Interpretations (hypotheses)** for
everything you inferred, **Open questions** for unconfirmed whys,
**Examples** for concrete cases that must or must not work, **Superseded
decisions** when a later prompt retracts an earlier one.

Hold to the story's own rules:

- Every claim cites the log entries it rests on, by timestamp. An uncited
  claim is suspect, so do not write one.
- Record goals as **outcomes, never methods**. "The arm must never reach a
  joint limit near a person", not "add a repulsive null-space term". Intent
  captured too concretely is imagination captured too early.
- A new goal or a change of direction needs its **why**. Mine the repo,
  git history and past prompts for it first; if it is still unclear, add it
  to Open questions as a proposal — "I think this is because Y" — so
  confirming costs Christian a yes or no.
- Never edit `raw-prompt-log.md`. It is ground truth and only the hook
  writes to it. Supersession is recorded in the story, never by rewriting
  the log.
- Nothing here outranks Christian's live word. Where a prompt conflicts
  with the story, the prompt wins and the story records the supersession.

## What to show

Print `git diff docs/intent/story.md` and walk Christian through it briefly:
what was added, what it was inferred from, and which parts are hypotheses
awaiting his confirmation. Ask him to correct it.

Do not commit unless he asks. If he does, commit the story on its own, with
no code in the same commit, so reverting `docs/intent/` can never touch
working code.
