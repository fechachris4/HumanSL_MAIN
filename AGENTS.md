# AGENTS.md

## Binding engineering contract

Before editing anything, read `docs/engineering/humansl-engineering-contract.md`.
It is binding for motion control, Vicon, kinematics, planning, low-level
command generation, simulation, testing and experimental claims. A conflicting
implementation must not proceed until the conflict is documented and
explicitly approved. Significant robotics tasks additionally require the
Robotics Analysis Packet gate described in
`docs/engineering/robotics-analysis-workflow.md`.

## Project purpose

This repository implements the control system for a wearable
Supernumerary Robotic Limb using Kinova Gen3 arms.

The intended control objective is to maintain or track the end-effector
pose in the world frame while the wearer and backpack-mounted robot base move.

## Verified current state

- GPMP2 currently produces a timed joint trajectory:
  q_ref(t), qdot_ref(t).
- That trajectory is a reference, not a final hardware command.
- The active joint controller computes:
  qdot_raw = qdot_ref + Kp * wrap(q_ref - q_measured).
- A resolved-velocity Cartesian PD controller is implemented and tested.
- The Cartesian controller includes pose feedback, measured-twist damping,
  reference-twist feedforward, damped least-squares inverse kinematics and
  null-space joint-limit avoidance.
- The Cartesian controller is currently unwired because production code does
  not produce PoseReference values.
- Both controller laws are intended to pass through the same velocity limits,
  joint-boundary handling, integration and hardware safety path.

## Current architecture objective

We are deciding how to expose the complete command-generation pipeline clearly:

reference
-> feedback control law
-> raw joint velocity
-> velocity and joint limits
-> position-command integration
-> hardware command.

Do not implement architecture changes until Christian explicitly approves
the target architecture.

## Engineering constraints

- Do not delete the Cartesian controller or its tests.
- Do not classify code as obsolete only because it is currently unreachable.
- Do not introduce managers, services, registries, factories or plugin systems.
- Prefer plain structs, pure functions and explicit control flow.
- An abstraction must either replace existing complexity or have at least two
  real current implementations.
- GPMP2 must remain outside the 500 Hz control loop.
- Blocking Vicon, file, terminal and logging operations must remain outside
  the 500 Hz control thread.
- Safety and actuation must not be bypassed by either controller mode.
- Frame names, units, timestamps and reference frames must be explicit.
- Do not run robot-facing commands unless Christian explicitly authorizes them.

## Working protocol

For requests to inspect, review, discuss or plan:

- Read the relevant source.
- Report evidence using files, symbols and call paths.
- Do not edit code.

For implementation requests:

- Implement only the explicitly approved migration slice.
- Before editing, state which files will change and why.
- Preserve behaviour unless a behaviour change is explicitly requested.
- Run all relevant hardware-free tests.
- Report added and removed production files, classes and concepts.
- Do not commit, push or operate hardware unless explicitly requested.

## The intended outcome is the source of truth

Treat Christian's intended outcome as the source of truth, not the
implementation plan. Autonomy covers implementation details, never
redefining the goal.

- Before making an important decision, ask: "Which option best achieves
  what Christian is actually trying to achieve?"
- Do not choose an approach merely because it is easier to implement,
  easier to test, requires fewer changes, or makes existing tests pass.
- If the easiest implementation conflicts with the intended behaviour or
  architecture, choose the implementation that satisfies the intended
  behaviour.
- Existing code, tests, documentation and previous decisions are evidence
  about the system, not absolute authority. If they conflict with the
  stated goal, identify the conflict rather than designing around it.

### Never validate a change only with evidence the change created

Never use evidence created by your own implementation as the sole proof
that the implementation is correct. A test you added saying your new
behaviour works is not sufficient by itself.

Where possible, validate against behaviour that existed before the change,
observable system behaviour, independent interfaces, real execution,
existing requirements, git history, or direct inspection.

For every acceptance criterion ask: "Could my implementation be
fundamentally wrong and still pass this check?" If yes, the check is
insufficient.

## Intent stewardship

`docs/intent/` is a persistent record of what Christian is trying to
achieve, built to outlive any single session. Read `docs/intent/story.md`
at the start of substantive work.

- `docs/intent/raw-prompt-log.md` is ground truth. A hook appends every
  prompt verbatim; never edit its entries, by hand or by model. Later
  prompts can supersede earlier ones — note supersession in the story,
  never by rewriting the log. Raw prompts are evidence about intent, not
  intent itself.
- `docs/intent/story.md` is the interpretation layer. Every claim in it
  cites the log entries it rests on; uncited claims are suspect. Entries
  are hypotheses until Christian confirms them.
- Record goals as outcomes, never methods ("the arm must never reach
  joint limits near a person", not "add a repulsive null-space term").
  Intent captured too concretely is imagination captured too early.
- Actively seek the why behind each new goal or change of direction.
  Propose rather than interrogate ("I think this is because Y — right?"),
  mine the repo, git history and docs before spending Christian's time,
  and keep unconfirmed whys under Open questions. Proceeding on an
  unconfirmed why must be stated out loud, never a silent default.
- When a request conflicts with the story, ask why in plain English
  before implementing.
- At session end, or before a large piece of work, show Christian the
  diff of the story since he last approved it and fold in his
  corrections. Git history of the story is the audit trail.
- At significant decision points, include one or two credible options
  from beyond Christian's framing, each with a one-line reason to care,
  plus a recommendation. Log adoptions and dismissals in the story's
  Exposure log; dismissals are binding.

### Christian controls the record, never the reverse

- Christian's live word always outranks the story. If what he says now
  conflicts with the record, ask at most one plain-English question,
  then comply and record the supersession. Never defend the story
  against him.
- A rule in the story applies only as far as its recorded why. A rule
  whose reason has expired is dead, not "still on the books".
- Rollback: the story is git-tracked. On request, revert `docs/intent/`
  to any earlier state. Commit story changes separately from code so a
  revert never touches code.
- Off switch: disabling the UserPromptSubmit hook (via `/hooks` or the
  two `settings.json` entries) stops capture; deleting or commenting
  this Intent stewardship section deactivates the protocol. Nothing
  else depends on it.
