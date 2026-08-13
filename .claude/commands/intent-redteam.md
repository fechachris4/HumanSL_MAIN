---
description: Reconstruct Christian's goals from the raw prompt log alone, then diff that against story.md to expose ambiguity
---

Check the interpretation layer against ground truth by rebuilding it
independently.

The principle this rests on: **never let an interpreter check its own
interpretation.** A story that has drifted still reads as coherent to
whoever wrote it. The only way to catch that is to reconstruct the goals
from the raw prompts by someone who has not seen the story, and see whether
the two agree.

## Step 1 — the blind reconstruction

Dispatch a subagent with the Agent tool. Its instructions must be, in
substance:

> Read `docs/intent/raw-prompt-log.md` in this repository, in full. It is a
> verbatim record of every prompt Christian has sent, oldest first.
>
> From those prompts ALONE, reconstruct what he is trying to achieve:
> his confirmed goals, his apparent goals, the whys you can infer, the
> places where a later prompt retracts an earlier one, and the points where
> his intent is genuinely ambiguous — where two different readings both fit
> the evidence.
>
> Record goals as outcomes, never methods. Cite the timestamps each
> conclusion rests on.
>
> You may read code, docs and git history to understand what the prompts
> refer to.
>
> You must NOT read `docs/intent/story.md`, and you must not read any
> other file under `docs/intent/` except the raw prompt log. If you open it
> by accident, say so in your output — a contaminated reconstruction is
> worthless and it is better to report that than to hide it.

That prohibition is the whole point of the exercise. Do not relax it, and
do not paste any of story.md's content into the subagent's prompt.

## Step 2 — the diff

Read `docs/intent/story.md` yourself and compare it against what came back.
Report only what actually differs:

- **Present in the story, absent from the reconstruction.** Either the
  story rests on something Christian said outside the log — say where, and
  note that it is uncited — or it drifted.
- **Present in the reconstruction, absent from the story.** Intent that
  was captured but never interpreted.
- **Different but both plausible.** The most valuable finding: genuine
  ambiguity in what Christian asked for. These become Open questions.
- **Contradictory.** One reading is wrong. Say which you think it is and
  why, then ask.

Where the two agree, say so briefly and move on. Agreement is the expected
result and does not need evidence laid out.

## Step 3 — what to do about it

Put real divergences to Christian as questions, proposing your reading so
answering costs him a yes or no. Do not silently edit the story to match
the reconstruction: the reconstruction is another hypothesis, not an
authority, and only Christian's confirmation settles which is right.
