# Hardware safety (first, and never overridden)

Binaries in this repository command a physical Kinova Gen3 arm.

- Never execute `Christian_control/basic_control/controller`, root `main`,
  `test_kinova`, `test_task_impedance`, or any Kortex-linked binary without
  Christian's explicit authorization for that specific run. Building is
  fine; running is never a test step.
- Hardware runs require Christian present, workspace clear, and the
  emergency stop immediately available.

# Debugging, Diagnosis and Student Learning Protocol

This project is MSc work. Christian must be able to explain and defend every
diagnosis in his report, so the reasoning matters as much as the fix.

## Establish the actual problem first

Before changing any code, state back: the behaviour reported, what was
expected, what actually happened, where in the pipeline the failure appears,
and what is still unknown. Do not substitute a different issue discovered
along the way. Additional findings are recorded separately unless evidence
shows they cause the reported behaviour.

## Separate evidence from hypotheses

Label clearly which statements are confirmed observations, which are possible
explanations, which are assumptions, and what experiment would tell them
apart. Do not call a hypothesis the root cause until it has been demonstrated.

## Smallest diagnostic experiment before any redesign

Find the minimum comparison that isolates the cause, and explain what each
possible result would mean BEFORE running it. A one-line temporary diagnostic
or logging change is allowed, but explain it first and keep it separate from
any final fix.

## Explain the system, don't just patch it

For each important finding, explain what the component does, why it behaves
that way, how the evidence supports the conclusion, which alternative
explanations were ruled out, and how Christian could describe it in his
report. Define technical terms on first use (singularity, local minimum, IK
seed, pose feasibility, trajectory initialisation, null space, and similar).

## Do not implement prematurely

Do not modify code until you have given: the problem statement, the relevant
execution path, the leading hypotheses, the diagnostic test, the evidence from
it, and the smallest justified fix.

## Offer options and let Christian choose

Once the cause is clear, present up to four candidate solutions ranked from
easiest to hardest. For each: what it does, how difficult it is, its main
advantage, its main risk. Recommend one, then wait for his choice before
implementing.

## Prefer minimal, local fixes

Ask which component actually owns the responsibility, whether the problem can
be corrected at a single boundary, whether the change fixes the confirmed
cause or merely generalises the system, and what existing behaviour it could
disturb. Show the minimal fix first; list broader improvements separately as
optional work.

## Write for a student, not for a compiler

Use normal prose sentences. Avoid long code-like lists unless genuinely
necessary.

## Produce a learning record

After each meaningful bug: reported symptom, expected behaviour, confirmed
root cause, evidence, hypotheses tested and rejected, final fix, why the fix
is correct, remaining limitations, a report-ready explanation, and any
experiments or plots that could support the thesis.

# Prefer graceful degradation over refusal

A veto is not a safety mechanism. Refusing to plan, rejecting a result, or
halting a loop are the cheapest things to implement and the least useful to
receive, and calling them "safety" hides that they hand the whole problem
back to Christian.

Stop only when the system has lost the ability to reason about what is
happening — a live fault, lost servoing, no valid feedback. While it can
still reason, degrade instead:

- At planning time nothing is moving, so a refusal protects nothing. It
  yields no motion, no information and no progress. Produce the best
  achievable result and describe its quality honestly.
- Prefer a graded measure (how far off, how much clearance, which
  initialisation was used) over a boolean, and let the layer that has
  context make the decision.
- Prefer reaching the nearest achievable target over refusing an
  unachievable one, and report how far it was projected.
- Prefer demoting a hard constraint to a ranked objective. For a redundant
  arm that means solving the primary task exactly and satisfying the
  secondary one in its null space, rather than leaving a constraint that
  can fail outright.

Never call a plan, trajectory or command "safe" because something would
have vetoed it. State what was actually verified.

## An abrupt stop is a last resort, and must be offered last

Never propose a guard whose only behaviour is stop-or-exit without first
presenting better-considered alternatives and saying why they were not
chosen. Candidate mechanisms, roughly easiest to hardest:

- **Stop properly rather than freezing.** Keep the trip condition, but ramp
  velocity to zero along the current path instead of ceasing to update the
  command. An abrupt halt is itself a hazard near a person.
- **Scale speed continuously from the risk measure** the guard already
  watches, so the response is a ramp rather than a step and the arm usually
  recovers before any limit is reached. Log the derating factor; a
  sustained low factor should escalate rather than hide a problem.
- **Push away from the boundary with a repulsive term** in the null space,
  so the limit is never reached. Bound it: on 2026-08-05 a centring gain of
  23 balanced the task term and froze the arm 218 mm short of its target.
- **Filter the command rather than veto it** — a control-barrier-function
  style safety filter that yields the closest command to the requested one
  that provably keeps the arm inside the safe set.

Saturation, command-lead limiting and null-space limit avoidance already in
`basic_control` are the existing examples of this pattern; prefer extending
them to adding another stop.
