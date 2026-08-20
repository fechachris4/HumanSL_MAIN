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

**Actual:** Q1: the software stop — hit. Q2: nearest-achievable
projection with the IK diagnostic first — hit. Both were the recommended
options; per the ledger's own miss-rate note, questions this safe are
cheap confirmations rather than real probes. The information was still
worth having: both answers gate hardware-relevant work.

### 2026-08-13 — which fix for the fast-pacing goal-accuracy regression

Faster pacing (his 1.0 s floor) degraded the canonical 0.2 m point move
to 82.4 mm goal error; offline sweep run before asking. **Predicted:** he
takes the ×10 tighter goal anchor (3.2 mm at full speed, recommended) —
it is the only option that keeps the speed he just chose AND the accuracy,
and his pattern all day has been capability first. Second guess: he asks
for anchor + qc_scale combined.

**Actual:** the ×10 tighter goal anchor. **Hit.**

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

### 2026-08-13 — twist feedforward, torso frame, math-first working mode

**Predicted:** (1) V_base,E feedforward: he picks "build it now, bring up
staged" — his math makes the term central and the code already has the
staged-enable pattern (velocity_enabled), so all-feedback-first then
switch-on-feedforward gives him both the term and a safe bring-up, plus
an ablation (with/without feedforward) that is thesis gold. (2) His
"torso" frame T is the mount plate — the rigid, measured thing; the table
name is the write-up's word for the wearer's back. (3) Math-first
presentation of control changes + a derivation doc mapping equation →
symbol → file:line: yes, emphatically — this prompt IS that request.

**Actual:** (1) his own words: "Its a PD velocity controller but i want
to establish world on hardware using the vicon. which uses only
feedback" — feedback-only first slice, V_base,E deferred. (2) T is a
SEPARATE torso segment, not the mount plate. (3) Equations before code,
always, plus the derivation doc.

**Result:** miss, miss, hit. (1) predicted build-now-staged; he wants
the world frame established with pure feedback before any feedforward
term exists — establish the frame, then earn the term. (2) the bigger
miss: his math's torso frame is a real, sixth tracked body for defining
targets relative to the wearer, not prose for the mount plate. The
five-segment inventory was never the whole intent. Pattern for both:
predicted from implementation convenience and existing artifacts;
he decides from the math and the thesis narrative.

### 2026-08-13 — what "we are using live recording" means

**Predicted:** he means working against the live stream now, and will
choose to capture the two stage-0 recordings immediately when told they
don't exist — the rig is in-volume and streaming, so the capture costs
two minutes and unblocks stage 2, and he has consistently chosen to
gather evidence when it is cheap (all-five-segments decision).

**Actual:** live stream only, no captures — chosen with the cost stated
in the option text (no calibration input, no offline mount-rigidity
answer).

**Result:** miss. The reading that survives: he does not want a parallel
recording pipeline at all. Combined with his earlier route choice ("the
controller logs the world pose it used"), the coherent picture is one
live chain — Vicon → controller → log → panel — where the controller's
own log IS the recording. Files-on-disk deliverables I keep proposing
(captures, replays) are my framing, not his; his framing is the running
system. Predict future data questions from that.

### 2026-08-13 — architecture A/B, timing contract, slice packaging

**Predicted:** (1) Architecture: B (world-reference adapter) — his own
description of B notes the controller "remains unchanged internally",
the unwired PoseReference seam exists for exactly this, and it is the
smallest change to a proven hardware path; the sim shows the law is
identical either way. (2) Timing: the fuller option — snapshot contract
+ ZOH + velocity-on-new-sample + filtering — because his message lists
velocity estimation and filtering under what the first implementation
"should" do, not under "eventually". (3) Packaging: fold the pose-only
world hold and the logging into one slice — "implement pose-only world
holding first" is his sentence, and observed-only logging alone gives
him nothing he has said he wants.

**Actual:** (1) Architecture A — hardware mirrors the sim's frames.py
assembly. (2) Timing: the minimum — contract + ZOH + stale-freeze, no
velocity estimation at all yet. (3) Two slices, log then hold.

**Result:** miss, miss, hit. (1) predicted B from the existing hardware
seam; his authority is the sim he built and trusts — "check the mujoco
project because i done it pretty well there". The sim is the reference
architecture, not the current C++ seam; C++-mirrors-Python was already
the recorded rule and I under-weighted it. (2) predicted more machinery
from his own list; he took the strict minimum — the "should" list was a
roadmap, not a slice-1 spec. Both misses share a shape: when his sim or
his math already embodies a choice, he picks consistency with it over
convenience of the existing hardware code.

### 2026-08-13 — authorization for the slice-1 validation run

**Predicted:** he authorizes me to run it now — "rebuild and lets run it"
is already an instruction, the gate question is confirmation of workspace/
e-stop state, and he has been driving toward exactly this validation all
day.

**Actual:** authorized, run it now.

**Result:** hit.

### 2026-08-13 — hold-slice design gate: activation, staleness, hardware order

**Predicted:** (1) Activation: explicit runtime command while joint hold
stays the default — matches his staged bring-up instinct and the panel
philosophy of deliberate hardware ops; auto-engage puts a brand-new
control path in charge the moment servoing starts, which nothing in his
history suggests he wants. (2) Staleness: 50 ms freeze threshold with
re-anchor-on-recovery — five lost frames is a real dropout, p99 was
9.7 ms so 50 ms has margin both ways; re-anchor avoids the recovery
jump, and he has consistently chosen no-step behaviour near a person.
(3) Hardware order: Nexus GUI reads (axis convention, template origins)
required BEFORE the first world-hold hardware run — his own sign-error
concern ("stabilisation becomes amplification") makes running on an
assumed convention indefensible; code and tests proceed meanwhile.

**Actual:** (1) auto-engage when fresh; (2) 50 ms freeze + re-anchor;
(3) no gate — the first run verifies signs itself, tethered low-gain.

**Result:** miss, hit, miss. The two misses rhyme with every miss today:
he trusts the running system plus his own presence at the e-stop over
procedural gates, and he wants the new capability IN the loop, not
beside it. Engineering answer recorded in the design: respect both
choices, shape the risk — ramp-in on engage, divergence latch that
degrades to joint hold. Predict future gate questions from "he gates
with his hands, not with process".

### 2026-08-13 — world-hold first run: gains and authorization

**Predicted:** authorized, and at CURRENT gains (Kp=10) rather than a
lowered first pass — every gate question today resolved the same way:
he gates with his hands and the e-stop, not with extra steps, and the
ramp + authority-scaled latch exist precisely so the first engage is
survivable at full gains. The spec's low-gain step was my framing.

**Actual:** authorized at current gains.

**Result:** hit — the "gates with his hands" model holds.

### 2026-08-13 — re-authorization after the engage fix

**Predicted:** immediate yes — same session, same conditions, he watched
the whole find-fix-test cycle and the demo is the thing he has been
driving toward for nine hours.

**Actual:** run it.

**Result:** hit.

### 2026-08-13 — plot viewer: tab placement and generation model

**Predicted:** (1) Placement: a card inside the existing RUNS tab, not a
new top-level tab — the panel's own precedent (diagnostics lives inside
SESSION, not its own tab) plus his general minimalism on panel surface
(rejected a speculative accel table earlier this project) both point
the same way. (2) Generation: synchronous with a timeout, matching
plan.py's solve() pattern, not a new background+poll mechanism — CLAUDE.md
explicitly says no new registries/managers, and a second async pattern
duplicating build.py's for a feature this small would be exactly that
kind of unjustified abstraction.

**Actual:** (1) new top-level PLOTS tab. (2) synchronous with a timeout.

**Result:** miss, hit. The miss: he wants a dedicated, visible surface for
analysis rather than folding it into an existing tab as a card — debugging
plots is enough of its own activity to deserve first-class navigation, not
a precedent match to diagnostics-as-card. Update the model: not everything
new is a card by default; ask when the feature is a distinct WORKFLOW
(generate-and-inspect) rather than a fact display bolted onto existing
context.

### 2026-08-14 — audit update: file target and architecture.md

**Predicted:** nothing — process miss. The question (where the updated
architecture/debugging audit should live, and whether to refresh
docs/architecture.md) was asked before a prediction was written here.
No prediction is backfilled; one written after seeing the answer would
be worthless.

**Actual:** (1) not a fresh repo filename — he pointed at the existing
published artifact ("You previously created an artifact with something
like this. Can you check it?"), i.e. the Gen3 Command Path page is the
thing to bring current. (2) architecture.md table refresh: yes,
recommended option.

**Result:** unscored for (1) by the miss above. Model note anyway: he
tracks deliverables he already has and prefers updating them in place
over new parallel documents — check the artifact list before proposing
a new file.

### 2026-08-14 — which Vicon recordings to make for the weekend

**Predicted:** nothing — process miss. The question (which of four
candidate recordings he could make at the rig tonight) was asked before a
prediction was written here. Not backfilled; one written after seeing the
answer would be worthless.

**Actual:** wearer motion, occlusion test, and handheld rigid motion — all
three of the ones needing only the torso cluster. Declined the fourth,
torso-plus-mount, which required re-adding the arm subject in Nexus.

**Result:** unscored. Model note anyway: with limited lab time he takes
everything that needs no reconfiguration and drops the one item gated on
changing the Nexus setup, even though that item answers a longer-standing
open question (mount-plate rigidity). Time at the rig is the binding
constraint, not the value of the measurement.

### 2026-08-17 — alignment questions for the Christian_control refactor goal

**Predicted:** nothing — process miss. Four alignment questions (relation
to the in-flight execution-twin work, how to create the separate working
folder, scope boundary, audit depth) were asked before a prediction was
written here. Not backfilled.

**Actual:** (1) finish Plan 02 first, then audit — not the recommended
pause-and-refactor-now; (2) plain directory copy, not the recommended
checkpoint-commit-plus-worktree; (3) Christian_control plus its entry
points (recommended); (4) thorough audit with committed docs
(recommended).

**Result:** unscored, but two of four answers rejected the recommended
option, which is information: (1) he treats the working humansl_sim as a
prerequisite baseline for the refactor, not competing work — the sim is
his reference implementation and he wants it standing before anything is
reorganized around it; (2) he prefers a plain copy over git plumbing for
the sandbox — simpler mental model, no checkpoint commit forced onto his
branch before he has approved anything.

### 2026-08-17 — Pinocchio warning noise and build job count

**Predicted:** nothing — process miss. Both questions (how to handle the
`-Wmaybe-uninitialized` noise, how to cap build parallelism) were asked
before a prediction was written here. Not backfilled; a prediction
written after seeing the answer is worthless.

**Actual:** (1) rejected every offered option for the warnings — "there is
a reason the warning is showing and its because of how the code was
written"; (2) "lets wait for now" on the job cap.

**Result:** unscored, and answer (1) was a correction, not a choice. I had
called the warning a false positive on the reasoning that `nv()` would be
1 for a mimic of a revolute joint. Christian rejected the framing rather
than the fix, and he was right: `JointModelMimic::nv_impl()` returns 0
(third_party/include/pinocchio/multibody/joint/joint-mimic.hpp:614) while
the compile-time `NV` is 1, so `topRows(nv())` writes nothing and the
following multiply reads uninitialised storage. The lesson is about my
reasoning, not his preference — I asserted a benign explanation without
checking the one function that decided it, then built an options menu on
top of that assertion. Offering suppression options for a defect I had
not verified was benign is the failure mode to watch for.

### 2026-08-17 — what to do with the Dynamics-is-RobotModel conclusion

**Predicted:** nothing — process miss again. The question (record only /
+CMake fix / +full split / nothing) was asked before a prediction was
written here. Not backfilled. For the record, the text recommended
"record only".

**Actual:** "Record + full slice incl. RobotModel split" — the largest
option, two steps past the recommendation.

**Result:** unscored, but consistent with a pattern now visible across
today's answers: when Christian has just articulated a design conclusion
in his own words, he wants it acted on in full, not staged. The staged
options read as delay to him, not caution. Next prediction should weight
"he picks the complete slice" heavily whenever the question follows his
own written summary.

### 2026-08-17 — which fix for the rejected circle plan

**Predicted:** process miss — the question (revert circle + offline
check / revert only / revert + slow lap / diagnose deeper) was asked
before any prediction was written here. Not backfilled; recorded after
seeing the answer, so worth nothing as a forecast. For the record, the
text recommended "revert + offline check".

**Actual:** none of the four. Christian reframed the problem in his own
words: "it is too easy for the planner to reject a trajectory I've
requested, and it's too easy for me not to know where the points I've
requested are in space." He redirected from run-fixing to two capability
gaps — planner refusal instead of graceful degradation, and no spatial
visibility of authored goals.

**Result:** miss (both on process and substance). Consistent with the
day's pattern: options that manage the immediate incident undershoot
when he has already generalised the lesson. He answered the question one
level above the one asked.

### 2026-08-17 — which capability to design first (pre-flight / 3D view / degrade)

**Predicted (before asking):** he picks the full program — all three
designed together — per today's pattern that staged options read as
delay once he has articulated the conclusion himself. Second most
likely: graceful degradation first, since it is the half he named first.

**Actual:** the direction question was first rejected at the tool level
(Christian wanted to answer but the request needed re-issuing), then
answered on the re-ask: "Design all three together". Recorded below.

### 2026-08-17 — "the now target should be in world pose so now we can
### get a real error" (re-ask after tool rejection)

**Predicted (before asking):** my reading is that he wants targets
authored/held as world-fixed poses — not mount-declared — so the
reported tracking error is a genuine world-stabilisation error when the
wearer moves (the SRL master goal). Predicted answer: he confirms that
reading, and picks "design all three together" on the re-asked
direction question with world-frame targets folded in.

**Actual:** both confirmed exactly as predicted: "Yes — targets fixed
in the room" and "Design all three together".

**Result:** hit, on both. The pattern held again — once he has stated
the conclusion himself, he takes the complete slice.

### 2026-08-17 — approve the world-targets / pre-flight / graded design?

**Predicted (before asking):** approve as drafted, likely with the
recommended answers to the three open decisions accepted wholesale —
today's pattern is full-slice adoption once the conclusion is his own.
Risk of miss: he may tighten Part D's confirm thresholds or insist the
shape-repair projection lands first.

**Actual:** (to be recorded)

### 2026-08-19 — how to handle the uncommitted URDF comment edit before
### pushing

**Not predicted.** The question ("commit just the URDF file / push only
what's already committed / stop and let him commit himself") was asked
before a prediction was logged — a process miss, not an honest gap
recorded in real time. Per the ledger's own rule, recording a guess now
would be worthless; this entry exists only so the lapse itself is
visible rather than silently absent.

**Actual:** commit just the URDF file, then push.

### 2026-08-19 — how to handle the three recorded-expectation artefacts
### that the mount-spacing change invalidated

**Not predicted before asking.** A second process miss on the same day:
the AskUserQuestion went out before anything was logged here, so no
honest prediction exists. What the ledger can record is the weaker
signal that the recommendation was visible in the question itself — I
offered "re-record all three" as the recommended option.

**Actual:** "Fixtures only, not the packet" — regenerate the two frozen
control fixtures, leave `moving_mount_control` failing because its
0.739 m / 0.650 m lever arms came from an accepted acceptance packet.

**Result:** miss against the stated recommendation. The lesson worth
carrying: Christian draws a sharper line than I did between artefacts
the repository generates (regenerable at will) and artefacts he has
accepted (his to revisit). Treat "accepted packet" as a stronger claim
on his attention than "frozen fixture" in future options.

### 2026-08-20 — how much of the test suite to delete

**Not predicted before asking.** Third process miss of this kind: the
AskUserQuestion went out before anything was logged here, so no honest
prediction exists and this entry cannot count as a hit. The weaker
recordable signal is that the recommendation was visible in the question
— I recommended "all but 5 safety tests", keeping the ones the
production code cites as the derivation of its constants
(`test_grid_coverage`, `test_waypoints`) or as guards on the 500 Hz path
(`test_execution_core`).

**Actual:** "All 58, no exceptions." Executed as 81 tracked files /
20,773 lines, the true count once `simulation/`, `tracking/` and the
shell and Python tests were included.

**Result:** miss against the stated recommendation. The lesson worth
carrying: the safety-evidence argument that felt decisive to me — that
`WorldSdf.h`'s grid constants have no provenance except the test that
measured them — did not move him. Either the argument was weaker than it
felt, or the cost of carrying 20k lines outweighed it for him. Worth
asking which, rather than assuming, before making the same argument again.

## 2026-08-20 — RobotModel scoping (3 questions)

Predictions were carried by which option I marked "(Recommended)" and placed first.

1. Who owns position limits, given the URDF is silent on joints 1/3/5/7?
   Predicted: one limits file with the URDF as a cross-check.
   Actual: leave as is for now. **Miss** — I read the joint-6 divergence as
   something he wanted closed in this slice; he wants the slice kept narrow.
2. Do planner margins live in RobotModel?
   Predicted: hardware only, margins stay planner config. Actual: same. **Hit.**
3. How far does the slice reach?
   Predicted: read-only design first. Actual: same. **Hit.**

## 2026-08-20 — margin shape (2 questions)

1. Identical margin at both layers, or ordered margins?
   Predicted: ordered (planner 2 deg, controller 1 deg) — I had just argued
   for it, so this was a weak prediction. Actual: ordered. **Hit.**
2. Does the velocity derate fold into the same margin mechanism?
   Predicted: leave velocity as the 0.95 derate, since a proportional derate
   and an absolute degree margin are different quantities. Actual: one
   mechanism for both. **Miss** — he weights one uniform concept above the
   quantity mismatch, consistently with the whole thread's push for fewer
   interacting numbers.

## 2026-08-20 — limits cleanup implementation (3 questions)

1. How does control/Config.h stop duplicating the yaml?
   Predict: build-generated header, matching the existing dh_params pattern —
   he has accepted that pattern before and it avoids giving the 500 Hz
   controller a runtime YAML dependency.
2. Derivation rule for the firmware error thresholds (today 140/150/123, all
   deliberately OUTSIDE the physical range, and inconsistently so)?
   Predict: physical + fixed outward margin. Less sure — j2's 140 vs a 128.9
   physical limit is an 11 deg gap that may encode something unrecorded, and
   he has said before not to change hardware thresholds blind.
3. Controller velocity fraction 0.99 raises the clip from 76/66.5 to
   79.2/69.3 deg/s. Confirm?
   Predict: he confirms — it follows directly from the ordering he specified.
   But I expect him to want it flagged rather than silent.

Actuals:
1. Build-generated header. **Hit.**
2. Neither of my three options. He asked instead whether EnsureJointLimits'
   writes are needed at all, or whether they are this project overriding the
   robot's native safety configuration. **Miss** — I framed it as "which
   derivation rule" and never questioned whether the writes should exist.
   The answer from the repo is that this arm's persisted state IS a
   degenerate 0/0 band that faults outward motion, so the writes are load-
   bearing; see the report.
3. Confirm 0.99, flagged. **Hit.**
