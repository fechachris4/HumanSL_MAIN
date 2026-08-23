# Motion limits map — where every number comes from

Date: 2026-08-13. Status: survey, then applied the same day (see
"Applied" at the end). Written to answer one question: of everything that
limits how fast the arm moves, which numbers are the robot's, which are
ours, and where does each one live? Values quoted in the body are the
PRE-change state the survey found; the Applied section records what
replaced them.

The design this feeds (Christian, 2026-08-13): every limit that can end in
a hardware fault is stated from Kinova's documentation, fixed as a hard
boundary the controller cannot cross, and normal operation sits about 5%
below it. Nothing in this file changes code; it is the provenance record
that decision will cite.

Terms used below, defined once:

- **Low-level servoing** — our mode: the PC streams a position command to
  every joint at 500 Hz and the arm's own trajectory planner is bypassed.
  Kinova's per-mode "soft limits" do not apply here (Gen3 User Guide R07,
  "Configurable limits"; established in
  `docs/decisions/qdot-limit-raise.md`).
- **Firmware safety bank** — per-actuator checks inside the robot that
  latch a fault and stop the arm regardless of what we stream. These are
  the only robot-side enforcement in low-level servoing.
- **Following error** — the gap between where a joint was commanded to be
  and where it actually is. Grows when the command demands more
  acceleration or torque than the motor can deliver.

## Current implemented planner limits (2026-08-23)

The numbered survey below preserves the pre-change evidence and reasoning. The
active source is now `planning/config/joint_limits.yaml`; it separates each
authored hardware value from the effective value the planner validates:

| Quantity | Authored hardware value | Planner fraction | Effective planner value |
|---|---:|---:|---:|
| Velocity, joints 1–4 | 1.3963 rad/s (80.0021 deg/s) | 0.95 | 1.326485 rad/s (76.002 deg/s) |
| Velocity, joints 5–7 | 1.2218 rad/s (70.004 deg/s) | 0.95 | 1.16071 rad/s (66.504 deg/s) |
| Acceleration, joints 1–4 | 5.2 rad/s² (297.94 deg/s²) | 1.0 | 5.2 rad/s² |
| Acceleration, joints 5–7 | 10.0 rad/s² (572.96 deg/s²) | 1.0 | 10.0 rad/s² |

`createJointLimits` derives both sets once. GPMP2 receives effective position
and velocity constraints. The one executable boundary, `ValidatePlan`, checks
the dense result against each joint's effective velocity and acceleration. If
dynamics alone fail, `PlanSolver` computes the required duration increase and
re-solves from scratch, for at most three duration attempts. It never makes a
failed result look valid by scaling sampled qdot after the solve.

The acceleration table remains an interim planning bound: Kinova Table 43 is
stated for angular-joystick and joint-trajectory modes, not this low-level
position stream. Therefore passing the offline acceleration check is not proof
of low-level torque/following-error margin. Exact completion remains a
Christian-authorized supervised hardware observation; no planner status alone
establishes physical completion or human-safe clearance.

## 1. Limits the robot itself enforces

These can fault the arm. Everything else in this document is software we
wrote and can change.

| Limit | Value | Where the value comes from | How it binds |
|---|---|---|---|
| Joint position, j2/j4/j6 | ±128.9° / ±147.8° / ±120.3° (j1/3/5/7 continuous) | User Guide Table 39 (`Config.h:222`) | Firmware `JOINT_LIMIT_HIGH/LOW` fault at the *configured thresholds*, which we write each connection (see §2) |
| Joint speed, live | **80.0021 deg/s (j1–4), 70.004 deg/s (j5–7)** | `GetKinematicHardLimits`, read from the base at every startup and printed (`Hardware.cpp:196`); today's session log shows exactly these | Startup refuses to run if our clip exceeds them (`Hardware.cpp:205`) |
| Joint speed, documented | 79.64 / 69.91 deg/s | User Guide Table 40 "general limits" (`docs/decisions/qdot-limit-raise.md`) | Reference value; the live number above is the enforceable one |
| Firmware `MAXIMUM_VELOCITY` | **not yet known** | Queryable per actuator via `tools/read_safety_limits` (read-only); no recorded readout exists in `runs/` | The velocity fault that provably ends a low-level stream |
| Firmware `FOLLOWING_ERROR` | **threshold not yet known** | Same query | The fault that actually ends *fast* runs: it trips on commanded-vs-measured gap, i.e. on acceleration/torque demand, before any velocity number is reached |
| Joint acceleration | 57.3 deg/s² (j1–4), 573 deg/s² (j5–7) | User Guide Table 43 — **stated for angular joystick and joint trajectory modes, not low-level** (`planner_bridge/config/joint_limits.yaml`, acceleration section) | Not established that firmware enforces these in low-level; there the physical acceleration ceiling is torque, surfacing as following error |
| Actuator torque | **not in the repository** | User Guide actuator specification tables — never extracted | The true first-principles bound behind acceleration and following error |
| Cartesian speed/accel | n/a in our mode | Guide Cartesian limits govern high-level modes only | In low-level joint streaming there is no robot-side Cartesian limit; Cartesian capability is `v = J(q)·q̇` — it depends on the arm's configuration and has no single number |
| Joint jerk | none published | No jerk table is cited anywhere in the repository | Bounded only indirectly, through torque rate |

Note what this table says: of the quantities the professor's instruction
appeals to, only position and velocity have clean documented numbers. The
acceleration figure is borrowed from a different control mode, and the two
firmware thresholds that actually end fast low-level runs have never been
read from this robot.

## 2. Limits our code imposes

Every one of these is ours. None comes from a Kinova document, except
where noted as derived.

### Controller (`control` + `runtime`)

| Knob | Value | Location | What it gates | Provenance |
|---|---|---|---|---|
| `kModelVelocityLimitsDegS` = `kQdotLimitDegS` | 50 deg/s uniform (uncommitted; was 45) | `Config.h:140`, alias `:232` | Runtime q̇ clip before integration, AND trajectory ingest: per-point velocity and the implied point-to-point average (`JointTrajectory.cpp:165–185`) | Bring-up value (`qdot-limit-raise.md`); the stale "45" comment above it needs rewriting |
| `kTrajFollowingErrorStopDeg` | 8.0° | `Config.h:194` | Our software stop, trajectory mode | Ours, chosen |
| `kFollowingErrorLimitDeg` | 3.0° | `Config.h:329` (switch `:314`) | Our software stop, reactive mode | Ours; measured max in a healthy run was 0.206° (`whole-path-validation.md`) |
| `kJointLimitWarnDeg` / `kJointLimitErrorDeg` | 130/145/118 warn, 140/150/123 error (j2/j4/j6) | `Config.h` | Written INTO the firmware each connection — these become the robot-side position thresholds of §1 | Ours, inside Table 39 for warnings |
| `kJointSoftwareLimitDeg` | Table 39 upper − 2°, capped by warn | `Config.h` | Software stop before outward motion crosses it; also the planner's ingest position gate (`Targets.cpp:44`) | Derived from Table 39 |

### Planner (`planner_bridge`)

| Knob | Value | Location | What it gates | Provenance |
|---|---|---|---|---|
| `joint_limits.yaml` velocity table | 50 deg/s all joints | `config/joint_limits.yaml` | GPMP2 limits, validator gate, approach pacing, time-scaling | **Wrong table**: cites Table 41, which is for admittance and force modes. The low-level numbers are the live 80/70 |
| Acceleration bound | velocity × 2.0 ≈ 100 deg/s² all joints | `PlanSolver.cpp:236` | Validator's dynamic gate, time-scaling | Invented ("reach the limit in ~0.5 s"). Its comment claims no acceleration table exists in `joint_limits.yaml` — **stale: the yaml has the Table 43 section; nothing reads it.** So j1–4 are checked against 100 deg/s² where the guide says 57.3, and j5–7 against 100 where the guide says 573 |
| `nominal_speed_mps` rail | [0.0001, 0.25] m/s | `PlannerConfig.cpp:162` | Rejects the config at load ("got 0.5" — today's failure) | Typo-catcher, no physical meaning (its own comment says so) |
| `min_duration_s` | 4.0 s (floor; rail [0.1, 600]) | `planner.yaml`, `PlannerConfig.cpp` | `duration = max(floor, distance/speed)` (`PlanSolver.cpp:40`) | Ours. Dominates every point-to-point move under 1 m: at the 0.25 m/s cap the distance term never wins inside reach |
| `approach_velocity_fraction` | 0.3 | `planner.yaml` | Paces the approach phase at 30% of the (already wrong) 50 deg/s | Ours. In today's circle run the approach was ~11.5 s of the 23.5 s total |
| `duration_s` (per path) | 12.0 s for the current circle | `goal.yaml`, left block | **The only knob that paces a traced path's lap.** `nominal_speed_mps` does not touch it | Ours, per goal |

### Which knob paces which motion

- **Point-to-point goal**: `max(min_duration_s, distance ÷ nominal_speed_mps)`,
  then stretched further if any joint exceeds the yaml velocity or derived
  acceleration bound (uniform time-scaling, up to 3 passes).
- **Traced path (circle, line)**: lap time is `duration_s` in `goal.yaml`;
  the approach to the path's start is paced from joint displacement over
  `approach_velocity_fraction ×` yaml limits, floored at
  `approach_min_duration_s`.
- Nothing warns when a knob is inert for the current goal type: today a
  ten-fold `nominal_speed_mps` change was attempted against a circle goal
  it could never have affected.

## 3. A latent bug that goes live the moment limits split 80/70

`ValidatePath.cpp:262` passes the dynamic gate with

    max_velocity <= joint_velocity_limits_rad_s.maxCoeff()

— the largest joint speed *anywhere* is compared against the *largest*
limit, not each joint against its own. Same shape for acceleration two
lines below. With today's uniform 50 deg/s table the two are identical, so
the bug is invisible. Write 80 (j1–4) / 70 (j5–7) into the table and a
plan driving joint 6 at 78 deg/s validates — 78 < 80 — while exceeding its
own 70 limit by 11%. The per-joint comparison must land in the same change
as the split limits, or the validator's "dynamic limits ok" becomes false
assurance exactly when it starts mattering.

## 4. Proposed hard boundaries under the 5% rule

What Christian's stated design gives, applied to the numbers we actually
possess today:

| Quantity | Hardware limit | Source quality | Operating bound (−5%) |
|---|---|---|---|
| Joint speed j1–4 | 80.0021 deg/s | live, read from this robot every startup | 76.0 deg/s |
| Joint speed j5–7 | 70.004 deg/s | live, same | 66.5 deg/s |
| Joint position | firmware thresholds we write (130/145/118 warn) | ours-into-firmware, already inside Table 39 | keep as-is |
| Joint acceleration | **no trustworthy number yet** | Table 43 is another mode's; torque tables unextracted; firmware behaviour unread | see §5 |
| Following error | **threshold unread** | queryable | set our 8°/3° stops deliberately below it once known |
| Cartesian speed | no robot-side limit exists | — | if a bound is wanted, it must be computed per configuration from `J(q)` and the joint bounds — a report-friendly derivation, not a constant |

## 5. What first principles cannot give yet, and the routes to it

Three gaps stand between this map and "every boundary stated with a
defensible source":

1. **Firmware thresholds** (`MAXIMUM_VELOCITY`, `FOLLOWING_ERROR`, per
   actuator): one run of `tools/read_safety_limits` — read-only, no
   motion, no writes — fills the entire column. Needs Christian's
   authorization like any Kortex-linked binary.
2. **Torque ratings**: extract the actuator specification tables from the
   Gen3 User Guide R07 into this repository, with page numbers.
3. **A defensible acceleration limit.** Candidate routes, easiest first:
   - *Adopt Table 43 at 95%* (54.4 / 544 deg/s²). One yaml edit + wiring
     the table the planner already carries but never reads. Honest caveat
     for the report: the table is specified for a different control mode.
   - *Measured sweep*: shorten the circle lap stepwise, watch following
     error grow toward the firmware threshold, and report the measured
     envelope. Strongest empirical claim; costs supervised lab time.
   - *Torque check per plan (RNEA)*: Pinocchio is already in the tree;
     inverse dynamics over a candidate trajectory gives each joint's
     demanded torque against its rating — acceleration limits stop being
     a constant and become a physical per-plan check. Most
     first-principles; most work; strongest thesis material.
   These compose: Table 43 now as the stated interim bound, RNEA as the
   principled gate, the sweep as its experimental validation.

## Applied (2026-08-13, same day)

Christian chose to remove the invented rails; the 5% rule went in as
surveyed:

- Controller clip `kModelVelocityLimitsDegS`: **76.0 / 66.5 deg/s**
  (95% of live 80.0021 / 70.004).
- Planner `joint_limits.yaml` velocity table: same 76/66.5 (in rad/s),
  replacing Table 41's 50.
- §3's max-vs-max validator bug fixed: per-joint comparison
  (now the componentwise dynamic checks in `ValidatePlan`) with a regression
  test.
- `nominal_speed_mps` rail 0.25 → 2.0; Christian set the value to 0.25
  and `min_duration_s` to 1.0, `approach_velocity_fraction` 0.9,
  `approach_min_duration_s` 0.1 (his live edits, kept).
- Fast pacing exposed a tuning imbalance: at a 1.0 s floor the canonical
  0.2 m test move missed its goal by 82.4 mm — the smoothness prior
  outweighed the goal anchor. Fixed by Christian's choice of a ×10
  tighter `goal.position_sigma_xyz` ([0.001, 0.01, 0.001], historical
  y-ratio preserved): 3.2 mm at full speed. Traced paths unaffected
  (they use the `path_following` priors).
- Still open, unchanged by this application: the acceleration route
  (§5.3), the unread firmware thresholds (§5.1), and the torque tables
  (§5.2).

## 6. Corrections this survey forces on existing text

- `Config.h:137` comment says the clip is 45; the working tree says 50.
- `planner.yaml`'s `nominal_speed_mps` comment says "the controller's own
  45 deg/s clip"; same staleness.
- `PlanSolver.cpp:236` comment claims the yaml has no acceleration table;
  it has one.
- `joint_limits.yaml`'s header presents Table 41 values as "the" velocity
  limits without saying they belong to admittance/force modes.
