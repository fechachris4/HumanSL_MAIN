# Realistic Vicon and Simulation Scenarios Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add deterministic scripted Mount disturbances, a switchable 100 Hz Vicon emulator that exercises the production estimator, optional shared obstacle geometry, and executed-clearance monitoring.

**Architecture:** The true Mount trajectory belongs to the simulator. Ideal sensing reads it exactly at 500 Hz. Realistic sensing samples only pose at 100 Hz, queues delivery with configured latency/noise/dropout, feeds the production `MountTwistEstimator` only on advancing sequences, and zero-order holds the coherent result into the shared core. Executed geometry is monitored from actual MuJoCo states and kept separate from GPMP2 planned-path reports.

**Tech Stack:** C++17, MuJoCo, Eigen, production Vicon snapshot/estimator code, deterministic PRNG, CMake/CTest.

## Global Constraints

- Plans 01–02 must be accepted first.
- Mount representation: RESOLVED 2026-08-17, no longer blocking. Christian
  chose the mocap Mount body — welded to world at the pose written into
  `mjData.mocap_pos`/`mocap_quat`, arms as its direct jointed children,
  `nq = nv = 14` — because the arms must hang from a possibly moving base
  as they do on the real rig. It is implemented (generator, pinned MJCF,
  `ModelContract`, `MujocoBackend` and their tests), so the arms now carry
  real gravity load and moving-Mount scenarios here no longer understate
  servo error: the ideal-mode hold error rose from 0.548 mm to 2.255 mm,
  which is the honest number. Two consequences bind this plan's tasks:
  the plant has no Mount velocity (a mocap body is teleported between
  steps), so scenario twist must come from the analytic `SampleMountMotion`
  truth and never from differencing mocap poses; and no reaction force
  reaches the wearer, so nothing here may claim to model base loading.
  See `Christian_control/simulation/README.md`, "How the Mount is
  represented" and "Known representation limits".
- Contact filtering changed with that rework and this plan's clearance
  work must not be surprised by it. Welding each `base_link` to world
  un-filtered two cross-arm pairs (`left_base_link` ↔
  `right_shoulder_link`, `right_base_link` ↔ `left_shoulder_link`): 105
  collidable body pairs now against 103 under the freejoint. The two
  generated `<exclude>` pairs cover the same-arm base/shoulder contacts
  only. So an `ncon == 0` assertion here is a stricter check than the same
  line was in Plan 02 — a cross-arm base/shoulder approach registers
  contacts the frozen Plan 02 model would have ignored. Treat such a
  contact as real interference to be reported, never as a filtering bug to
  be excluded away.
- Mount pose and Mount twist must come from ONE `SampleMountMotion(t)`
  call per control tick. The plant gets the pose, the sensor path gets the
  pose and twist, and no configuration field anywhere may carry a twist
  that is independent of the pose being prescribed: an unenforceable pair
  of numbers can describe a Mount the plant is not moving, and the
  controller would feed forward a base velocity that does not exist.
  Plan 02's `SimulationConfig::world_V_mount` was deleted on 2026-08-17
  for exactly this reason; do not reintroduce that shape.
- `docs/engineering/humansl-engineering-contract.md` is binding for every task in this plan.
- Simulation sources/tests belong to the standalone `Christian_control/simulation/` CMake project created in Plan 02; existing basic_control/vicon suites stay in their own projects.
- Control remains 500 Hz; realistic Vicon emits at exactly 100 Hz.
- Repeated 500 Hz reads of one 100 Hz frame must never produce a new derivative sample.
- Realistic mode must never read MuJoCo Mount velocity directly.
- Ideal mode remains exact 500 Hz pose/twist.
- Noise, latency, dropout, and occlusion occur only at the simulated sensor boundary.
- Raw and filtered Mount twist, sequence, timestamps, validity, and age are observable.
- Scripted motion only: static, translation, rotation, combined. Replay/manual dragging remain deferred.
- Planned `q(t)` remains outside control. Executed clearance is monitoring, not a guarantee.
- No robot-facing command or commit without explicit authorization.

---

### Task 1: Implement deterministic scripted Mount motion

**Files:**
- Create: `Christian_control/simulation/src/MountMotion.h`
- Create: `Christian_control/simulation/src/MountMotion.cpp`
- Create: `Christian_control/simulation/tests/test_mount_motion.cpp`
- Modify: `Christian_control/simulation/src/SimulationConfig.h`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
enum class MountMotionKind { kStatic, kTranslation, kRotation, kCombined };

struct MountMotionConfig {
    MountMotionKind kind;
    Eigen::Vector3d translation_axis_world;
    double translation_amplitude_m;
    Eigen::Vector3d rotation_axis_world;
    double rotation_amplitude_rad;
    double frequency_hz;
    double phase_rad;
};

struct MountTruth { CartesianPose world_T_mount; Twist world_V_mount; };
MountTruth SampleMountMotion(const MountMotionConfig&, double time_s,
                             const CartesianPose& world_T_mount_at_zero);
```

- [x] **Step 1: Write analytic pose/twist tests**

Test `t=0`, quarter-period, half-period, arbitrary phase, normalized/non-normalized axes, zero amplitude, invalid frequency, and combined motion. Validate angular velocity against a central finite difference of the rotation log and linear velocity against the analytic derivative.

`SimulationConfig` gains `MountMotionConfig` and `world_T_mount_at_zero` only. It must not gain a twist field: the twist is `SampleMountMotion(t).world_V_mount`, produced by the same call that produces the pose the plant is given, so the two can never disagree. (Plan 02 carried `world_T_mount` and `world_V_mount` as independent fields; `world_V_mount` was deleted on 2026-08-17 because nothing could enforce that it was the derivative of the pose.) Christian's reference simulation enforces the same coupling by requiring the torso pose and twist callables to be installed together and evaluating both at one `data.time` (`msc_project/sim/world.py`, `configure_torso_driver` / `_refresh_torso`).

- [x] **Step 2: Run and observe missing implementation**

Run: `cmake --build Christian_control/simulation/build --target test_mount_motion -j2`

- [x] **Step 3: Implement SE(3) motion without Euler-angle state**

Use `Eigen::AngleAxisd` for orientation and analytic scalar sinusoid derivatives. Reject non-finite values and nonzero amplitude with a zero axis.

- [x] **Step 4: Run deterministic motion tests**

Run: `ctest --test-dir Christian_control/simulation/build -R '^mount_motion$' --output-on-failure`

Review checkpoint: formulas, frames, metres/radians, and world-axis conventions are documented next to symbols.

### Task 2: Add a 100 Hz Vicon sensor emulator

**Files:**
- Create: `Christian_control/simulation/src/SimViconSource.h`
- Create: `Christian_control/simulation/src/SimViconSource.cpp`
- Create: `Christian_control/simulation/tests/test_sim_vicon_source.cpp`
- Modify: `Christian_control/vicon/src/MountTwistEstimator.h`
- Modify: `Christian_control/vicon/src/MountTwistEstimator.cpp` only if a hardware-independent public input/output seam is missing
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
enum class SimViconMode { kIdeal, kRealistic };

struct SimViconConfig {
    SimViconMode mode;
    double sensor_rate_hz;
    double delivery_latency_s;
    double position_noise_std_m;
    double orientation_noise_std_rad;
    std::vector<std::pair<double, double>> dropout_intervals_s;
    std::uint64_t seed;
};

class SimViconSource {
public:
    std::optional<BasePoseSample> Advance(double control_time_s,
                                          const MountTruth& truth);
};
```

- [ ] **Step 1: Write exact 100/500 scheduling tests**

Over 50 control ticks require exactly 10 advancing Vicon sequences at ticks `0,5,10,...,45`; four repeated reads between frames; age increases by 2 ms per repeated control tick; estimator update count equals advancing sequence count; reported frame rate is 100 Hz.

- [ ] **Step 2: Write latency/noise/dropout/reset tests**

Assert delivery latency changes receive time but not capture time; seeded noise is byte-reproducible; dropout intervals produce invalid/no-new samples; estimator resets after `kViconMountTwistResetGapS`; recovery's first pose has invalid twist and never differentiates across the gap.

- [ ] **Step 3: Run and observe missing source**

Run: `cmake --build Christian_control/simulation/build --target test_sim_vicon_source -j2`

- [ ] **Step 4: Implement ideal and realistic modes**

Ideal mode returns exact pose/twist every control tick. Realistic mode samples pose only on the 100 Hz grid, applies deterministic tangent-space orientation noise and position noise, queues by delivery timestamp, and calls the production estimator only for a newly delivered advancing sequence.

- [ ] **Step 5: Run source and production estimator suites**

Run: `ctest --test-dir Christian_control/simulation/build -R '^sim_vicon_source$' --output-on-failure`

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(mount_twist_estimator|base_pose|frames)$' --output-on-failure` (existing production suites stay in the basic_control project)

Review checkpoint: add a test double that throws if realistic mode asks for truth velocity; the suite must still pass.

### Task 3: Wire sensor modes and scripted motion into the simulation runner

**Files:**
- Modify: `Christian_control/simulation/src/DualSimulationRunner.h`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.cpp`
- Modify: `Christian_control/simulation/src/SimMain.cpp`
- Create: `Christian_control/simulation/tests/test_moving_mount_control.cpp`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- `DualSimulationRunner` consumes `MountMotionConfig` and `SimViconConfig`; each cycle applies truth to MuJoCo Mount and feeds only the selected sensor output to both arm cores.

The `MountMotionConfig` half of this was done in Task 1 on 2026-08-17,
deliberately and reported as a deviation: adding the config
field without a consumer would have left a run able to declare a moving
Mount that the plant held still — the shape this plan's global
constraints ban. `DualSimulationRunner::Step` therefore already evaluates
one `SampleMountMotion(config.mount_motion, t, config.world_T_mount_at_zero)`
per tick and `Start()` seeds the plant from that truth at `t = 0` (so a
nonzero phase does not step the Mount between seed and first tick).
Static behaviour was verified unchanged — `ctest dual_simulation_runner`
green and the acceptance run still 2.255 mm / 1.732 mm.

That wiring is now GATED (2026-08-17, review follow-up):
`tests/test_dual_simulation_runner.cpp` gained `TestScriptedMountWiring`,
which steps translation, rotation and combined motion for 0.5 s each and
checks that (a) `kStatic` ignores the amplitude fields, reproducing the
static command trace bit for bit; (b) the Mount pose and twist BOTH cores
receive at tick `k` are `SampleMountMotion(t_k)` — position and twist
bit-equal, rotation to 1.1e-16; and (c) the pose the plant holds after
tick `k` is that same pose, not tick `k-1`'s. A vacuity guard requires
the Mount to actually leave the anchor. Two mutations compiled outside
the repository confirm the checks bite: a one-tick-late plant write fails
by exactly `|v| dt` = 6.283e-05 m, and feeding the cores the static
anchor fails the sample and vacuity checks in all three kinds while the
TCP-drift number alone still looks healthy. Measured world-hold drift
under motion: 3.67 mm translation, 5.04 mm rotation, 7.79 mm combined,
against 2.30 mm static.

Step 1's ideal-sensing half and Step 4's CLI were then completed on
2026-08-17 (see the step notes below). What remains of this task is the
sensor modes (`SimViconConfig`, Task 2) and Steps 2, 3 and 5, which need
them.

- [ ] **Step 1: Add static and moving-Mount integration tests**

For ideal sensing, hold fixed world TCP references under translation, rotation, and combined Mount truth. Assert both cores receive one coherent Mount sample and that disabling Mount motion reproduces the static trace.

This is the first gate that steps physics with a Mount pose that actually changes between ticks, and it must check the ordering as well as the tracking: the plant's Mount pose after tick `k` equals the pose that tick's `WorldSample` reported (bit-identical, not within a tolerance), and the world pose of every arm body carries the Mount motion of tick `k`, not of tick `k-1`. A Mount write that landed one tick late, or after the substeps instead of before them, is off by `|v|·dt` — sub-millimetre, invisible in a hold-error number, and it would bias every moving-base result. `tests/test_mujoco_backend.cpp` already gates this at the backend boundary (constant-velocity translation carry and rotating-Mount parity against Pinocchio, added 2026-08-17); this step gates it through the full runner with the cores in the loop.

The IDEAL-SENSING half of this step landed 2026-08-17 as
`tests/test_moving_mount_control.cpp` (ctest `moving_mount_control`).
Static, translation and rotation each assert BOTH halves of the physical
claim in simulator truth only: the world TCP error stays inside a bound
derived from the accepted packet (12 mm translation, 15 mm rotation,
twice the predicted peaks), AND the TCP travels through the MOUNT frame
by the excursion the geometry demands, in the opposite direction —
measured `p_M_E(t) = R(t)^T (p_W_E(t) - p_W_M(t))` with `R`, `p_W_M` read
back from the plant's own mocap row. Measured: 2.255/1.732 mm static,
3.698/3.696 mm under a 0.05 m translation (excursion 48.69 mm against the
50 mm amplitude), 4.815/4.571 mm under a 0.1 rad rotation (excursions
73.54/64.85 mm against lever arms 0.7388/0.6501 m, which independently
reproduce the packet's 0.739/0.650 m, and an excursion ratio of 1.1342
against the lever-arm ratio 1.1363). Two mutants compiled outside the
repository bound what that is worth: telling the cores the Mount is
static while the plant moves fails 13 checks (world error 48.9 mm
translation / 71.7 mm rotation — the arm merely carried — with the
Mount-frame excursion collapsed to the 2.3 mm droop), while zeroing only
the Mount TWIST in the sample is NOT caught BY THESE SCENARIOS, inflating
the error by 1.38x and 1.48x, exactly the packet's predicted 1.3-1.5x,
and staying inside the gates (it does fail `ctest
dual_simulation_runner`, whose bit-for-bit sample check sees it; measured
2026-08-17, 3 checks, twist err 3.142e-02 translation and 6.283e-02
rotation). No twist-disabling switch was added to production code to make
that gateable, because the packet specified that negative control as a
statement rather than a switch.

What this step therefore gates is the POSE half of the base-motion
compensation. The twist half has two gates behind it and they cover
different things, so the distinction is recorded here rather than left to
be misread: `ctest mount_motion` (twist is the derivative of the pose) and
`ctest dual_simulation_runner` (the cores receive that call's result bit
for bit) are statements about the SAMPLE, not about the control law's use
of it. Measured 2026-08-17: sign-flipping the Mount twist in the
`WorldSample` — the spatial-vs-body convention error — IS caught by
`dual_simulation_runner` while `moving_mount_control` passes it at 6.59 mm
translation / 9.42 mm rotation (55 % / 63 % of its gates); sign-flipping
the transport term inside `world_frames::ArmControllerState`
(`basic_control/src/Frames.h`) passes every test in the simulation project
and is caught only by basic_control's `ctest frames`, which cross-validates
that assembly at 1e-12 against fixtures generated by running the reference
`frames.py`. Behavioural gating in this project would need a faster
scenario: at 0.3 Hz correct is 10.65 / 14.63 mm against 19.71 / 28.15 mm
sign-flipped (a factor ~1.9), where at the shipped 0.1 Hz it is 3.70
against 6.59 mm. The scenario values come from the accepted packet and
were not raised to make a test discriminate.

What is left of this step: the same coverage under realistic sensing
(needs Task 2), and the per-arm-body carry check, which
`tests/test_mujoco_backend.cpp` already holds at the backend boundary.

- [ ] **Step 2: Add realistic derivative-noise evidence tests**

Use nonzero seeded pose noise; assert raw estimated twist is nonzero/noisy, filtered twist has lower RMS than raw for the fixed case, sequence is stepped at 100 Hz, and the 500 Hz joint command reflects held twist samples without extra estimator updates.

- [ ] **Step 3: Implement runner wiring**

At each 2 ms tick sample truth, update the MuJoCo Mount body, advance the sensor source, populate both `RobotState` world fields from the same coherent sample, step both cores, and then advance physics.

- [ ] **Step 4: Expose explicit CLI settings**

Add `--vicon ideal|realistic`, `--motion static|translation|rotation|combined`, amplitude/frequency/axis/phase, latency/noise/dropout, and seed. Validate before starting the simulation; print a complete run preamble.

The MOTION half landed 2026-08-17: `--motion`, `--motion-axis` (`x|y|z`
or three numbers), `--amplitude-m`, `--amplitude-rad`, `--frequency-hz`
and `--phase-rad`, validated at the argv boundary by the same
`ValidateMountMotionConfig` the sampler uses, with a preamble that prints
the resolved motion (including which channels the chosen kind does not
consult), the ideal-sensing declaration, the mocap-Mount representation
limits and the evidence-class caveats. Run length moved out of
`SimulationConfig` into `SimMain`, which owns the loop: a headless run
needs a positive `--duration-s` (default 2 s), while `--viewer` with 0 or
no duration runs until the window closes, and SIGINT ends any run cleanly
at a tick boundary. `ctest sim_cli` runs the binary: identical arguments
must give a byte-identical summary, a different `--motion` or
`--phase-rad` must not, the named motion must show up in the Mount
excursion the plant reports, `--headless --duration-s 0` and a
non-numeric amplitude must exit 2. The `--vicon` flag and the sensor
settings wait on Task 2.

Review follow-up (2026-08-17): the preamble's quantitative caveat about
the unmodelled base-motion (fictitious) term was a literal, "0.5 % of g at
these defaults" — correct while the motion was fixed, and stale the moment
these flags let a run leave the defaults (`--motion combined
--frequency-hz 2 --amplitude-m 0.2` is 44.7 m/s^2, over 4 g, where the
omitted term is the run's dominant physics). `SimMain` now derives it from
the resolved motion and the seeded geometry: the upper bound
`A_t w^2 + A_r w^2 r (1 + A_r)` over the ACTIVE channels, with `w = 2 pi f`,
`r` the farther TCP's distance from the Mount measured at the seed, and
`g` read from the model, plus a louder reading above a tenth of `g`. The
preamble therefore prints after `Start()`, since one of its numbers is
measured rather than configured. `ctest sim_cli` gained a check against a
hand-derived `A (2 pi f)^2` at two frequencies and a static run, which a
frequency-blind figure fails (verified by mutation).

- [ ] **Step 5: Run moving-Mount tests and headless scenarios**

Run: `ctest --test-dir Christian_control/simulation/build -R '^(moving_mount_control|sim_vicon_source|dual_simulation_runner)$' --output-on-failure`

Run one 2 s headless case per motion kind in ideal and realistic modes. Expected: deterministic completion or an explicitly classified controller stop; never an unlabelled exception.

Review checkpoint: inspect raw/filtered twist and Cartesian derivative-command traces; expected 100 Hz stepping is not smoothed or hidden in logging.

### Task 4: Add shared static scene parsing and MuJoCo geometry

**Files:**
- Create: `Christian_control/simulation/src/SharedScene.h`
- Create: `Christian_control/simulation/src/SharedScene.cpp`
- Create: `Christian_control/simulation/config/free_space.yaml`
- Create: `Christian_control/simulation/config/example_box_scene.yaml`
- Create: `Christian_control/simulation/tests/test_shared_scene.cpp`
- Modify: `Christian_control/planner_bridge/src/WorldSdf.h`
- Modify: `Christian_control/planner_bridge/src/WorldSdf.cpp`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Produces one bounded scene contract consumed by planner SDF construction and MuJoCo body/geom creation:

```cpp
struct StaticBox {
    std::string id;
    CartesianPose world_T_box;
    Eigen::Vector3d size_m;
};

struct SharedScene { bool enabled; std::vector<StaticBox> boxes; };
SharedScene LoadSharedScene(const std::string& yaml_path);
```

- [ ] **Step 1: Write parser and equivalence tests**

Reject duplicate IDs, nonpositive size, non-unit quaternion, non-world frame, non-finite values, and unsupported shapes. For the example box, assert planner bounds/pose and MuJoCo geom bounds/pose come from the same parsed record.

- [ ] **Step 2: Run and observe missing parser**

Run: `cmake --build Christian_control/simulation/build --target test_shared_scene -j2`

- [ ] **Step 3: Implement one minimal box-only scene format**

Do not add a general scene graph. `free_space.yaml` sets `enabled: false` and is the default. When disabled, neither planner collision geometry nor MuJoCo obstacle geoms are created.

- [ ] **Step 4: Adapt `WorldSdf` to consume parsed boxes**

Preserve existing SDF bounds/resolution and validation. Echo exact scene IDs/poses/sizes into planner reports and simulation provenance.

- [ ] **Step 5: Run planner and simulation scene tests**

Run: `ctest --test-dir Christian_control/planner_bridge/build -R '^(world_sdf|shared_scene)$' --output-on-failure`

Run: `ctest --test-dir Christian_control/simulation/build -R '^shared_scene$' --output-on-failure`

Review checkpoint: free-space default remains behaviourally unchanged; the same example box is visibly and numerically identical in planner and MuJoCo.

### Task 5: Monitor executed clearance and infeasible holds

**Files:**
- Create: `Christian_control/simulation/src/ExecutedClearance.h`
- Create: `Christian_control/simulation/src/ExecutedClearance.cpp`
- Create: `Christian_control/simulation/tests/test_executed_clearance.cpp`
- Create: `Christian_control/simulation/tests/test_infeasible_world_hold.cpp`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.h`
- Modify: `Christian_control/simulation/src/DualSimulationRunner.cpp`
- Modify: `Christian_control/simulation/src/SimMain.cpp`
- Modify: `Christian_control/simulation/CMakeLists.txt`

**Interfaces:**
- Produces simulation-only telemetry:

```cpp
struct ExecutedClearanceSample {
    double inter_arm_surface_clearance_m;
    double obstacle_surface_clearance_m;
    bool contact;
    std::string closest_pair;
};
```

- [ ] **Step 1: Write geometry/contact tests**

Use known separated, touching, and overlapping MuJoCo configurations. Assert surface-to-surface sign, closest pair identity, and contact. Explicitly label planner clearance and executed clearance as different fields.

- [ ] **Step 2: Write an infeasible-hold scenario**

Choose a scripted Mount disturbance that drives one bounded joint outward. Assert null-space avoidance acts first; once the existing software boundary rejects the outward integrated proposal, all seven commands hold and both-arm session stop is `joint_limit_warning`. Assert no automatic escape plan is submitted.

- [ ] **Step 3: Implement non-authoritative monitoring**

Compute executed clearance from current simulated link/contact geometry after each physics exchange. Logging/contact may fail the scenario, but no result is fed into the production shared core or described as a pre-execution guarantee.

- [ ] **Step 4: Run clearance and hold tests**

Run: `ctest --test-dir Christian_control/simulation/build -R '^(executed_clearance|infeasible_world_hold)$' --output-on-failure`

- [ ] **Step 5: Add CLI summaries**

At run end report minimum executed inter-arm/obstacle clearance, first contact, minimum joint margin, and exact stop reason. Print `planner path clearance != executed clearance` whenever a planned report is present.

Final gate: repeat each seeded scenario twice and require identical sensor sequences, controller events, stop reason, and numeric trace. Plan 04 may begin after realistic 100 Hz timing and negative hold/clearance cases pass.
