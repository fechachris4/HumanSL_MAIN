# Dual-arm MuJoCo execution twin

This project runs two instances of the shared execution core
(`humansl_execution_core`, built by `../basic_control`) against a dual-arm
MuJoCo model with exact HumanSL kinematics. It exists to test execution
correctness of the world-frame command pipeline — frames, signs, units,
lever arms, timestamps and stop verdicts — not to model Kinova servo
dynamics. Every result it produces is labelled `simulation` evidence
(engineering contract section 13).

## Authority split

- **Kinematic truth** — joint order, axes, limits, mount-to-base
  transforms, TCP frames — comes from the production URDF
  `../basic_control/config/GEN3_dual_mounted.urdf` through
  `DualArmKinematics`. The right TCP is `ConfiguredTool_Link` (configured
  tool); the left TCP is `leftEndEffector_Link` (bare flange). They are
  different physical points by design.
- **Simulator mechanics** — meshes, inertials, generic position-actuator
  gains — come from the audited `msc_project` Gen3 MJCF. Those gains are
  declared generic plant parameters, not Kinova values, and that model's
  own scene mounting is never copied.

`model/model_provenance.yaml` records both sources with SHA-256 hashes;
`tests/test_model_provenance.py` (ctest `sim_model_provenance`) verifies
the record against the actual files on every test run.

## Layout and build

This is its own CMake project, following the per-component pattern
(`basic_control`, `planner_bridge`, `vicon`). It will obtain
`humansl_execution_core` via `add_subdirectory` of `../basic_control`;
there is deliberately no top-level CMakeLists.

```
cd Christian_control/simulation
cmake -S . -B build
ctest --test-dir build --output-on-failure
```

## How the Mount is represented

The Mount is a MuJoCo **mocap body**: welded to world at whatever pose
sits in `mjData.mocap_pos` / `mocap_quat`, with both arms as its direct
jointed children (`nq = nv = 14` — the Mount itself contributes no
state). `MujocoBackend::Exchange` writes the tick's Mount pose there once,
before the physics substeps. The arms therefore hang from the Mount under
full gravity and are carried wherever it goes, while nothing they do can
move it.

Christian adopted this on **2026-08-17**, from the outcome he stated: the
arms must be attached to a possibly moving base, as they are on the real
rig and in his `msc_project` simulation. It replaced a Mount freejoint
whose `qpos`/`qvel` were rewritten before every substep. That pinned
position and velocity but left the freejoint's *acceleration* free, so
inside each substep the whole rig was in free fall — and in a freely
falling frame the arms are weightless. Probe evidence for the superseded
plant (2026-08-17, home keyframe, MuJoCo 3.10.0): mount
`qacc_z = -9.810 m/s^2` while every arm-joint `qacc` was below
`1e-12 rad/s^2` against gravity bias torques up to `8.16 N*m`, and a held
command moved joint 2 by exactly `0.0 rad` over a full tick.

What changed, measured:

- The arms are gravity-loaded. Each servo now settles where its force
  balances gravity, `droop = -qfrc_bias / kp`: 0.234 deg on joint 1,
  0.090 deg on joint 2 at the home keyframe, matching MuJoCo's own bias
  torques and the model's own gains to within 1.5 % (ctest
  `mujoco_backend`).
- The ideal-mode hold error rose honestly, because it is no longer
  understated: 0.548 mm max over the Plan 02 acceptance run before,
  **2.255 mm** (right) / 1.732 mm (left) after, on the same
  `humansl_sim --headless --duration-s 2` run. It is a transient: the
  peak lands between 0.05 s and 0.1 s and never grows afterwards, which
  is the servo settling and then the world-hold outer loop taking the sag
  back. The 0.5 s dual-hold ctest moved 0.186 mm -> 2.295 mm.
- Two `<exclude>` contact pairs became necessary. MuJoCo skips
  parent/child contacts unless the parent is welded to world, which every
  `<side>_base_link` now is; without them the home posture has 8
  penetrating contacts 12.0 mm deep. They restore the **same-arm**
  base/shoulder filtering the freejoint model got automatically — not all
  of its filtering. Two **cross-arm** pairs (`left_base_link` ↔
  `right_shoulder_link` and `right_base_link` ↔ `left_shoulder_link`)
  were filtered before as parent/child of the shared Mount weld and are
  live now that each `base_link` is welded to world in its own right:
  105 collidable body pairs here against 103 in a reconstructed freejoint
  model (probe 2026-08-17, all geom margins forced to 50 m so only the
  filter decides). That is the honest model — two separate arms
  interfering is a real collision the old weld tree was hiding — but it
  changes what `ncon == 0` means: a cross-arm base/shoulder approach now
  registers where the Plan 02 model would have ignored it. `ctest
  sim_model_contract` pins the exclude list to exactly those two same-arm
  pairs.
- Kinematic parity is unaffected: the Task 3 gate re-ran with identical
  maxima to the last digit (9.437e-16 m, 1.394e-15 rad, 4.441e-08) at
  unchanged tolerances.

## How the Mount moves

`src/MountMotion.h` is the one description of the Mount trajectory, and
`SampleMountMotion(config, t, anchor)` returns the pose the plant is
given together with the exact derivative of that pose. One call per
control tick, in `DualSimulationRunner::Step`, is what keeps the plant
and the sensed twist from ever describing different Mounts.

The motion is a scalar sinusoid `s(t) = A sin(2*pi*f*t + phi)` driving a
translation along a world axis, a rotation about a world axis through the
Mount's **own** origin, both, or neither (`kStatic`, the default). It is
analytic, so it is seed-free and reproducible: every sample is computable
from `t` alone, with no integrated orientation to drift. `AngleAxisd`
about a fixed world axis makes the angular velocity exact — `omega =
b * sdot` — rather than a small-angle approximation.

Defaults are 0.05 m, 0.1 rad and 0.1 Hz. They come from the feasibility
arithmetic in the header: the worst-case joint-speed demand of a world
hold under base motion, `||qdot|| <= ||V|| / sigma_min(J6)` with the
pessimistic home `sigma_min = 0.152`, stays under 60 % of the tightest
`kQdotLimitDegS` clip for translation, rotation and combined motion
alike, so a scenario exercises the hold rather than the command clip.
`ctest mount_motion` checks the twist against central finite differences
of the pose (residuals ~4e-11 against a 1e-8 tolerance) with independent
Rodrigues and rotation-log oracles, and quantifies the spatial-vs-body,
pre-vs-post-multiply and Mount-origin-vs-world-origin mutations it must
be able to catch.

The wiring — that the running simulation actually moves the Mount this
way — is gated separately by `ctest dual_simulation_runner`, which steps
all three moving kinds for 0.5 s and checks that the sample both cores
receive and the pose the plant holds after each tick are both
`SampleMountMotion(t_k)`, bit for bit. Two mutations were compiled
outside the repository to show those checks bite: writing the previous
tick's Mount pose to the plant fails by exactly `|v| dt` = 6.3e-5 m, and
leaving the cores on the static anchor fails the sample and vacuity
checks while the hold-error number alone still looks healthy.

## Does the world hold actually hold?

That is a different question from "is the Mount wired up", and `ctest
moving_mount_control` is where it is asked. Everything it measures is
simulator truth — the MuJoCo TCP site and the Mount pose read back from
the plant's own mocap row — so the controller cannot pass by agreeing
with its own kinematics.

Writing a TCP in the Mount frame, `p_M_E(t) = R(t)^T (p_W_E(t) -
p_W_M(t))`, gives two numbers that move in opposite directions. A world
hold that works keeps `p_W_E` still and therefore drags `p_M_E` through
the Mount's whole excursion; an arm that is merely carried by the base
does the reverse. Each scenario asserts **both**, plus the direction:

| motion (defaults) | max world-hold TCP error | Mount-frame excursion | predicted |
| --- | --- | --- | --- |
| static | 2.255 mm right / 1.732 mm left | same (Mount still) | 1–4 mm droop |
| translation, 0.05 m | 3.698 / 3.696 mm | 48.69 mm | 5.8 mm, 50 mm |
| rotation, 0.1 rad | 4.815 / 4.571 mm | 73.54 / 64.85 mm | 7.5 mm, chord |
| combined | 7.486 / 2.974 mm | 109.8 / 42.8 mm | — |

Measured 2026-08-17 over half a period (5 s at 0.1 Hz), one physics
substep. The first three rows are the gated `moving_mount_control`
scenarios; the combined row is a `humansl_sim --headless --duration-s 5
--motion combined` run, reported rather than gated — its two channels
add on the right arm and largely cancel on the left, which is why its
left figures are the smallest in the table.

The gates are twice the packet's predicted peaks (12 mm and 15 mm); the
measured values are reported by every run and were never used to set
them. The rotation case carries the left/right asymmetry a translation
cannot show: the measured lever arms are 0.7388 m and 0.6501 m
(independently matching the accepted packet's 0.739/0.650), and the
right/left excursion ratio 1.1342 reproduces the lever-arm ratio 1.1363,
so the asymmetry is geometry rather than an artifact.

Two mutants compiled outside the repository say what those checks are
worth. Telling the cores the Mount is static while the plant still moves
it fails 13 checks: the world error becomes 48.9 mm (translation) and
71.7 mm (rotation) — the full amplitude, the arm merely carried — while
the Mount-frame excursion collapses to the 2.3 mm droop. Zeroing only the
Mount **twist** in the sample is **not** caught by these scenarios: it
inflates the error to 5.12 and 7.11 mm, 1.38x and 1.48x, exactly the
packet's predicted 1.3–1.5x for the `Kd` term flipping from damping to
disturbance, and stays inside gates set at twice the prediction. It is
caught elsewhere — `ctest dual_simulation_runner` fails on it — which is
the distinction the table below draws.

So these scenarios gate the **pose** half of the base-motion compensation,
and it is worth being exact about what stands behind the **twist** half,
because two different things get called "gated". `ctest mount_motion` says
the twist is the derivative of the pose, and `ctest dual_simulation_runner`
says the cores receive that call's result bit for bit. Both are statements
about the SAMPLE — the right number is handed in — and neither says
anything about the control law using it correctly.

Measured on 2026-08-17, at the shipped 0.05 m / 0.1 rad / 0.1 Hz
scenarios:

| twist mutation | `moving_mount_control` | caught by |
| --- | --- | --- |
| Mount twist zeroed in the `WorldSample` | passes: 5.12 / 7.11 mm | `ctest dual_simulation_runner` (bit-for-bit sample check): 3 checks fail, twist err 3.142e-02 (translation), 6.283e-02 (rotation, combined) |
| Mount twist SIGN-FLIPPED in the `WorldSample` | passes: 6.59 / 9.42 mm (55 % / 63 % of the 12 / 15 mm gates) | `ctest dual_simulation_runner` (bit-for-bit sample check) |
| transport term sign-flipped INSIDE the law (`world_frames::ArmControllerState`, basic_control `src/Frames.h`) | passes: same numbers | nothing in this project; only basic_control's `ctest frames` |

The last row is the one to remember: a bit-equality check on the input
cannot see what the law does with the input. `ctest frames` catches it
because it cross-validates the whole world-frame assembly at 1e-12 against
fixtures generated by RUNNING the reference simulation's `frames.py`, with
nonzero Mount twists in every case — an independent oracle rather than a
re-derivation. (The two `WorldSample` mutants were reproduced here; the
in-law mutant is the 2026-08-17 adversarial review's measurement, 12
failures in `test_frames`.)

Gating the twist BEHAVIOURALLY would need a faster scenario, and the
measurement says how much faster: at 0.3 Hz the correct run is 10.65 mm
(translation) and 14.63 mm (rotation) against 19.71 and 28.15 mm
sign-flipped — a factor ~1.9 a derived bound could separate — where at the
shipped 0.1 Hz the pair is 3.70 against 6.59 mm and no bound set at twice
the prediction can tell them apart. That change is not made here: the
scenario amplitudes and frequency came from the accepted packet, and
raising them to make a test discriminate is choosing a scenario for the
test's convenience rather than for what it represents.

Plan 03 Task 3 Step 1 still owns the remainder: the same coverage under
realistic 100 Hz sensing.

## Running it

```
humansl_sim [--headless] [--viewer] [--duration-s <s>]
            [--physics-substeps <n>]
            [--motion static|translation|rotation|combined]
            [--motion-axis x|y|z | --motion-axis <x> <y> <z>]
            [--amplitude-m <m>] [--amplitude-rad <rad>]
            [--frequency-hz <hz>] [--phase-rad <rad>]
```

`--motion-axis` applies to the channels `--motion` selects, so for
`combined` it sets both; the preamble prints the axis, amplitude,
frequency and phase actually in force, and says which channels the chosen
kind does not consult at all. Amplitudes and frequency default to the
values derived in `src/MountMotion.h` (0.05 m, 0.1 rad, 0.1 Hz).

A headless run must state a positive `--duration-s`, because it produces
numbers and a number needs a horizon; the default is 2 s. Under
`--viewer`, omitting `--duration-s` or passing 0 runs until the window
closes. `Ctrl-C` ends any run cleanly at a tick boundary and still prints
the summary. Nothing in the program reads a clock into its output and the
Mount motion is analytic, so the same command line yields a byte-identical
summary — `ctest sim_cli` checks that by running the binary twice, and
checks the flags are read at all by requiring two different motions to
produce different output.

Exit status is 0 for a run that ended without a stop and without contact
(including a `Ctrl-C` or a closed window), 1 for a stop verdict, a contact
or a rejected model, and 2 for a bad command line.

## Known representation limits (ideal mode)

- **No reaction force reaches the wearer.** The Mount is prescribed, not
  simulated: it has infinite effective mass, so the arms' weight and
  accelerations push back on nothing. The simulation answers "what does
  the arm do when the base moves", never "what does the arm do to the
  person". This is deliberate and contract-honest — the Mount trajectory
  is an input, not a result.
- **The Mount has no velocity the physics can see.** A mocap body is
  teleported between steps, so base-motion fictitious forces on the links
  are not modelled. At the shipped scenario defaults (0.05 m, 0.1 rad,
  0.1 Hz) the peak base acceleration is `A (2 pi f)^2` = 0.020 m/s^2 from
  the translation plus 0.029 m/s^2 tangential at the 0.739 m right-TCP
  lever arm: 0.049 m/s^2 together, 0.50 % of g. That makes it a
  second-order omission AT THOSE VALUES — but it is an omission, and it
  grows with the SQUARE of the frequency, so a faster scenario is not
  free: `--motion combined --frequency-hz 2 --amplitude-m 0.2` reaches
  44.7 m/s^2, over four times g, at which point the omitted term is the
  dominant physics of the run rather than a footnote. Because the CLI can
  leave the defaults, the figure is no longer quoted anywhere the reader
  meets it: `humansl_sim` computes it from the run's own resolved motion
  and the seeded lever arm and prints it in the preamble, and says so
  loudly above a tenth of g. `ctest sim_cli` checks that figure against a
  hand-derived `A (2 pi f)^2` at two frequencies, so a figure that stops
  following the flags fails a test. The consequence for
  code: Mount twist is a *sensing* quantity, owned by whoever builds the
  `WorldSample`, and must come from the analytic Mount motion — never
  from differencing the mocap poses the plant was given, and never
  configured beside the pose as an independent number. `SimulationConfig`
  therefore has no twist field (the Plan 02 `world_V_mount` was deleted on
  2026-08-17): nothing could have checked it against the pose, so a run
  could have told the controller the base was moving while the plant held
  it still. Pose and twist therefore come from one
  `SampleMountMotion(t)` call per tick (`src/MountMotion.h`, Plan 03
  Task 1), evaluated in one place in `DualSimulationRunner`: the pose
  goes to the plant and the sample, the twist to the sample only. With
  the default `kStatic` motion the Mount stands at its anchor and the
  twist is the exact derivative of a constant pose — zero.
- **The Mount pose is zero-order held across a tick's substeps.** The
  Mount trajectory is a staircase with steps of at most `v * dt`
  (0.16 mm at the Plan 03 scenario speeds). This keeps the plant's Mount
  pose bit-identical to the pose that tick's `WorldSample` reported.
  Measured with the Mount actually moving (`ctest mujoco_backend`,
  2026-08-17): under a constant-velocity translation every arm body sits
  exactly `p(t_k)` from its stationary twin (3.4e-16 m, with joint
  trajectories agreeing to 1.7e-16 deg — a constant-velocity frame is
  inertial, so that is the correct physics, not an artifact), and under a
  rotating, translating Mount the stepped world TCP matches
  `world_T_mount(t_k) · mount_T_tcp(q)` from the production Pinocchio
  kinematics to 1.2e-15 m. Both checks also record what a one-tick-late
  Mount write would have cost — about 1 mm — so the ordering is falsified,
  not assumed.
- **The servos are generic.** `kp`/`kv` come from the mechanics source
  and are declared generic plant parameters, not Kinova values, so droop
  magnitudes are representative rather than predictive of the real arm.

## Safety

Nothing in this project links Kortex, contains a robot address, or can
command hardware. Headless operation is the default and the only mode
tests use; the passive viewer is optional. `humansl_sim` is hardware-free
and safe to run.
