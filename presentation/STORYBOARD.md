# MSc project talk — storyboard (source of truth for the deck)

## Goal of the talk

In under 10 minutes, a technical audience that is **not** from robotics (ML / 3D
graphics engineers) should leave with three beliefs:

1. **Problem:** keeping a wearable robot's hands still in the room while the wearer
   walks is a hard, real 3D problem (every step moves the arms 6–7 cm).
2. **Solution + impact:** I built and validated a controller that removes about
   two-thirds of that motion on real arms worn by 7 people, from simulation to hardware.
3. **How I work:** when tuning stopped helping, I found the cause from the data (the
   velocity signal arrives 61–71 ms late) instead of guessing, and that finding sets
   the next step.

The deck must explicitly cover four headings, in order: **Problem → Approach &
solution → Trade-offs & challenges → Impact.** Each content slide carries a small
section tag with one of those words, so the listener can tick them off.

## Constraints

- Presented **remotely by screen share** → large type (body ≥ 18 pt, no text < 14 pt),
  high contrast, few words, nothing that depends on hover/click.
- Total spoken time **9:30 ± 0:30**; speaker notes hold a timed script.
- **Style:** Imperial College London look (navy dominant, Arial), built as a
  lookalike because no official .potx is available. Swap in the official template
  later if wanted.
- **Clips:** animated GIFs from the msc_project repo, so they autoplay and loop in
  PowerPoint, Google Slides and Keynote with no click during a live share.
- One control-law equation and one delay formula, both explained in words.
- Links: github.com/fechachris4/msc_project (results, sim, figures) and
  github.com/fechachris4/HumanSL_MAIN (C++ real-time controller, `Christian_control/`).

## Slide plan (≈ 12 slides)

| # | Tag | Title (message) | Visual | Time |
|---|---|---|---|---|
| 1 | — | Keeping a robot's hands still while its wearer moves | `hw_turn.gif` hero; name, MSc, Imperial, MUVE Lab, 2026 | 0:30 |
| 2 | Problem | Every step moves your extra hands | `rig.jpg` + stat callouts (63–68 mm RMS, ~1.8 Hz, 2 × 7-DoF arms) | 0:50 |
| 3 | Problem | Goal: hold the hand fixed in the room, not on the body | `task_illustration.png` full width | 0:40 |
| 4 | Approach | Measure the backpack, cancel it through the arm | frame chain diagram world→mount→base→hand + 1 equation | 1:00 |
| 5 | Approach | One control cycle, every 2 ms | `control_loop.png` + 5 numbered steps | 0:50 |
| 6 | Approach | A fast loop that nothing can block | native architecture diagram (C++ runtime) | 0:50 |
| 7 | Solution | Simulation first: same law, physics instead of hardware | `hold_pose.gif` + 31.0 → 6.7 → 2.6 mm | 0:45 |
| 8 | Solution | Seven people walked on a treadmill wearing the arms | `hw_walk.gif` + `hw_replay.gif` | 0:45 |
| 9 | Impact | About two-thirds of walking motion removed | native bar chart 71 / 63 / 56 % + 78 % platform | 0:45 |
| 10 | Challenge | Turning up the gain barely helped (9 % vs 38 % predicted) | `hw_rejection.png` | 0:45 |
| 11 | Challenge | The cause: the velocity signal arrives 61–71 ms late | delay budget bar + 2·sin(ωd/2) + `velocity_delay.png` | 1:00 |
| 12 | Trade-offs | Trade-offs I made | 2×2 cards | 0:50 |
| 13 | Impact | Impact, and what comes next | impact + next steps + what transfers | 0:45 |
| 14 | — | Thank you / questions | GitHub links + QR codes | 0:10 |

## Fact sheet (the ONLY numbers allowed on slides)

Sources: `msc_project/README.md`, `docs/hardware-trials.md`, `docs/simulation.md`,
`docs/scope-and-next-steps.md`; `HumanSL_MAIN/Christian_control/README.md`.

**Setup**
- Two Kinova Gen3 7-DoF arms mounted on a backpack ("mount"); Vicon motion capture.
- Torso-twist clip: wearer twists about 60°; elbows swing, hands stay close to where they were.

**Hardware trials**
- 7 participants on an instrumented treadmill at 0.5, 1.0, 1.5 m/s. Pooled means over
  6 (first session reported separately); 4 of them at 1.5 m/s.
- 128 walking trials; most ran one arm at a time (19 had both arms servoing).
- Share of mount motion removed (arithmetic mean): **71 % (0.5 m/s), 63 % (1.0 m/s), 56 % (1.5 m/s)**.
- Pitching/swaying platform (5 participants): **78 %** removed; pitch steps moved the
  locked point 94 mm RMS vs end-effector 14 mm; sway steps 81 mm vs 17 mm.
- Walking moved the locked point **63–68 mm RMS** at every speed.
- At 1.0 m/s: 85 % removed at 0.25 Hz, 58 % at 1 Hz, 29 % at 2 Hz.
- Scoring: Vicon end-effector path vs the "locked point" (where the hand would go if the
  arm were rigid on the mount). Locked-point model checked on uncontrolled arms: within
  0.5 % (median over 126 arms).
- Trials ran a **400 Hz world-frame Cartesian controller** with a **Kalman filter**
  estimating mount pose and velocity from Vicon; no mount-velocity feedforward. That
  controller is on the MUVE Lab machine, not public. The analysis is Christian's.

**The delay diagnosis**
- Pilot: K_d stepped 0.8 → 2.6; response near 1 Hz fell **9 %**, model predicted **38 %**.
- Delay budget: Kalman estimate lags mount motion **36–46 ms** + sensing age **~10 ms** +
  joint response **~15 ms** = **61–71 ms** total.
- With that measured delay and nothing fitted, the model reproduces the measured response
  (amplitude and phase) better than the no-velocity-term model in **6 of 6** participants.
- Residual of a d-late velocity estimate: **|1 − e^(−jωd)| = 2 sin(ωd/2)**. At 1.8 Hz and
  60 ms that is **two thirds**; at 0.9 Hz, a third.
- Steps come at roughly **1.8 Hz**.

**Simulation (MuJoCo, Python, 500 Hz, offline)**
- Mount follows sinusoids per axis (bounce 20 mm, sway 22 mm, fore-aft 8 mm, roll 2°,
  pitch 1.3°, yaw 3°).
- At f = 1.8 Hz, 8 s RMS error: arms locked **31.0 mm**, reactive controller **6.7 mm**,
  + feedforward **2.6 mm**. No velocity term: 7.5 mm.
- Reactive controller removes 94 % at 0.5 Hz, 63 % at 3 Hz.
- Each 10 ms of delay costs the feedforward about 1 mm; past ~30 ms the K_d term is worse
  than no velocity term; at 60 ms both sit at 8.2–8.6 mm.
- Safety filter: 13 spheres per arm kept outside a cylinder around the wearer; if a
  command violates a barrier constraint, a QP (OSQP) finds the closest safe joint velocity.

**Control law (per arm, per cycle)**
- Compose world → mount → arm base → end-effector (pose + Jacobian through the mount).
- PD on world-frame pose and twist error → task-space velocity.
- Damped least squares → joint velocities, + null-space term toward mid-range joints.
- Safety filter, then integrate to a joint position command.

**C++ real-time controller (`HumanSL_MAIN/Christian_control`)**
- 500 Hz loop (2 ms), both arms; Vicon at 100 Hz on its own acquisition thread, handed
  over through a wait-free slot.
- Mount velocity estimated by differencing poses + first-order low-pass (τ = 50 ms).
- Pinocchio kinematics from one URDF; DLS + damped null-space law; velocity clamp +
  position integration; Kortex cyclic exchange to the arms.
- Planning (GPMP2) runs on a separate worker thread, never inside the 500 Hz cycle.
- Every plan passes one validator before it can run → REACHED / GOAL_BLOCKED / FAILED.
- Only the runtime links the Kortex SDK; `control/` cannot touch hardware; a MuJoCo twin
  runs the same `control/` core.

**Other limits found (trade-offs slide)**
- 12 of 128 walking trials: the motion planner moved the reference > 100 mm from target;
  those had the largest end-effector motion → a planner setting.
- At 1.5 m/s a commanded joint speed was at its limit 32 % of the time (mostly wrist) vs
  ~10 % at slower speeds.
- Constant 18–21 mm offset from goal follows kinematic-model vs Vicon disagreement →
  calibration.

**Next steps**
- Faster motion sensing / velocity estimator; feedforward on hardware (sim: 6.7 → 2.6 mm
  with no delay); both arms together; MPC that compensates the measured delay; replay
  recorded gait in simulation.
