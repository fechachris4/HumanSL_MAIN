# Prediction ledger

Understanding is tested by prediction. Before asking Christian a question,
the predicted answer is written here; his actual answer is recorded
afterwards, and the mismatch is the calibration signal.

The order matters and is the only thing that makes this worth keeping. A
prediction written after seeing the answer measures nothing. If a
prediction was missed at the time, record the entry as `not predicted`
rather than back-filling a guess — an honest gap is data, a fabricated hit
is noise.

This file is interpretation, not ground truth. It is also evidence about
the steward itself: a ledger that stops gaining entries is a visible sign
that the practice lapsed, which is why it lives in git rather than in a
model's memory.

Miss rate is the useful number, and a low one is not automatically good.
Predicting every answer correctly usually means the questions were too safe
to be worth asking.

## Entries

Newest last.

### 2026-08-13 — how much enforcement the intent mechanisms should have

**Predicted:** remind, don't block. Reasoning: he has pushed back on
process overhead repeatedly (the 2026-08-13 15:00 rebuke for sending a
15-agent workflow at a one-line question), and the design already records
that only conflict questions should block. A hook that refuses to end a
session would be exactly the kind of veto CLAUDE.md argues against.

**Actual:** remind, don't block.

**Result:** hit.

### 2026-08-13 — how much of the design to build now

**Predicted:** diffs and staleness only. Reasoning: same impatience with
process, plus the design itself tiers prediction and red-team as "the later
tier", so the smallest thing that fixes the observed failure looked like
the likely choice.

**Actual:** all three.

**Result:** miss. The impatience is with *ceremony that produces nothing*,
not with building capability — he had just been told two of the three did
not exist, and wanted them to exist. Predicting from mood rather than from
what he was actually asking for is the error to avoid repeating.

### 2026-08-13 — which route to wire Vicon into the panel

**Predicted:** nothing — the question was asked before a prediction was
logged, which the protocol exists to prevent. Recorded as a procedure
miss, not as a hit or a miss on the substance. A prediction invented after
reading the answer would be worthless.

**Actual:** controller logs world pose (the panel inherits it through its
existing name-based column reader).

**Result:** no prediction, so no score. What is worth carrying forward: he
chose the option that keeps one source of truth over the two cheaper
options that would have shown a pose the controller never acted on. That
is consistent with the anti-gaming rule in CLAUDE.md — display that cannot
be traced to a real control decision is not evidence.

### 2026-08-13 — the why behind "as fast as possible"

**Predicted:** he will confirm that keeping up with the wearer is the
reason. Reasoning: CLAUDE.md states the project's objective as holding an
end-effector pose in the world frame while the wearer and backpack-mounted
base move, and an arm slower than its wearer cannot do that at all. The
competing readings are that the professor meant something more generic
(demonstrate the hardware's capability in the thesis) or wanted faster
experiment turnaround, both of which would make peak speed an end rather
than a means.

Confidence is moderate, not high. The prompt relays a professor's
instruction rather than stating Christian's own reasoning, so he may
simply not have a why beyond "I was told to" — in which case the honest
answer is that the question was premature and belongs to his supervisor,
not to him.

**Actual:** to show the hardware's full capability — peak speed as an end
in itself, targeting the Kinova table limits.

**Result:** miss. The error is instructive and worth not repeating: I
reasoned from the *system's* purpose to the *request's* purpose, assuming
that because the project exists to track a moving wearer, any speed
requirement must serve tracking. But this is MSc work, and a thesis has
goals the control system does not — demonstrating that the hardware was
driven to its real limits is a result for the report, not a control
objective. Reading every request as though it must serve the running
system will keep missing the ones that serve the degree instead.

The practical consequence was also missed. Under the tracking reading the
answer would have been "you need less speed than you think". Under the
actual reading, `min_duration_s` matters more than the velocity clip: a
4.0 s floor caps a 90 degree joint move at about 22 deg/s, so the arm
cannot come near its 79.64 deg/s capability no matter what the clip says.

### 2026-08-13 — how far to go in removing our own speed rails

**Predicted:** nothing — the question was sent before a prediction was
logged, the same procedure miss as the Vicon entry above. Recorded as such
rather than back-filled.

**Actual:** remove our rails; only Kinova-enforced limits should bind.

**Result:** no prediction, so no score. Worth carrying forward: offered a
stepwise variant that reached the same destination more cautiously, and he
took the direct one. He was not asking to be protected from the speed — he
was asking why anything other than the robot was deciding it. The pattern
matches the "no invented vetoes" line in CLAUDE.md: a limit with no
recorded physical reason has no standing.

### 2026-08-13 — alignment check on the speed work (predictions logged BEFORE asking)

Four questions asked together; predictions first, actuals to be filled in
after his answers.

**Q1 — did he know run 1's circle actually ran and completed?**
Predicted: he saw the arm move (he was present with the e-stop), but the
all-nan status made him doubt the run was healthy — that doubt is why the
paste starts at t=92 in the post-completion hold. So: "saw it move, but
the nans worried me".

**Q2 — why does the professor want maximum speed?**
Predicted: tracking bandwidth — the SRL must compensate wearer motion, so
a slow arm fails the project's stated control objective. Low confidence:
"characterise the platform for the thesis" is nearly as likely, and he may
not have been told a reason at all.

**Q3 — which part of the circle run should get faster?**
Predicted: both lap and approach, lap first — the lap is the demo, and the
11.5 s approach is dead time nobody asked for.

**Q4 — what should I do next?**
Predicted: implement now (limits slice + circle pacing) — he is
mid-hardware-session with the arm connected; a knob map or a sweep design
would land after the lab time is gone.

**Actual:**

- Q1: "Knew it worked, just slow." **Miss.** The nans were my worry, not
  his — he watched the arm do what he expected and his question was about
  speed all along. Lesson: he reads the robot, not the console.
- Q2: all three (tracking the wearer, characterise the platform, demo
  credibility). **Partial.** Predicted tracking as the single why; he
  holds all three at once, so no single-purpose framing should drive the
  design.
- Q3: whole pipeline, all goal types. **Miss.** Predicted lap-first for
  the current circle; he wants fast as the default everywhere, not a
  per-goal tweak.
- Q4: neither plain "implement" nor plain "map" — a limits *derivation*:
  every fault-capable limit (velocity, acceleration, jerk, anything that
  faults the arm) stated from Kinova's documentation from first
  principles, fixed as hard boundaries the controller cannot cross, with
  operation ~5% below them; knob map on paper first, every number with
  its source. **Miss on substance** (predicted implement-now): he wants
  to *understand and own the limits* before any of them move. Consistent
  with the MSc protocol — he must defend every number in the report.

### 2026-08-13 — far-target "fault" in the chicken-head controller

**Q1 — is the failure he means the software joint-limit-warning stop, or a
real firmware fault?** Predicted: the software stop. Five 2026-08-05
reactive runs ended exit_reason=joint_limit_warning with clean fault
banks, and the genuinely-faulting cause (j4/j6 0/0 firmware bands) was
fixed 2026-08-03. Risk to the prediction: his memory may be anchored on
the 08-03 sessions where the arm really did fault.

**Q2 — which remedy will he choose?** Predicted: project the target to
the nearest achievable pose (option 3), because it is the behaviour his
own graceful-degradation section names, and his pattern today is
capability over protection. Second guess: the far-away orientation
relaxation (option 1) alongside it.

**Actual:** (pending)

### 2026-08-13 — scoping questions for "controller logs world pose"

**Predicted:** (1) Link decision: stub-able provider, not a hard SDK link —
he has now twice heard, and not pushed back on, the argument that the
standalone build is worth keeping, and ViconInterface was already written
to stand alone. (2) Ordering: implement only after the stage 0 lab
recording — his own 2026-08-12 decision was that real data comes before
format decisions, and the log-column layout is exactly a format decision.
(3) Column content: log the raw mount-segment pose under an honest name
now rather than wait for stage 2 calibration — he prefers graded/partial
progress over blocking (CLAUDE.md graceful-degradation), and the panel
label can say what it is. (4) Which segments: mount segment only, not all
five — the other four have no consumer yet and he dislikes speculative
surface (accel table removed from panel for exactly that reason).

**Actual:** (1) stub-able provider; (2) after stage 0 recording; (3) raw
segment pose under an honest name; (4) all five Nexus segments — not
mount-only.

**Result:** 3 hits, 1 miss. The miss repeats the earlier lesson in a new
form: "he dislikes speculative surface" applies to *controls nobody uses*
(the accel table), not to *data capture*. Recording everything the lab
streams costs disk, not attention; lab time is the scarce resource, and a
recording you didn't make cannot be re-made for the thesis. Predict data
questions from evidence value, not from UI minimalism.
