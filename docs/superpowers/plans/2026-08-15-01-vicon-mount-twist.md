# Filtered Vicon Mount Twist Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Estimate, filter, publish, and log the Vicon Mount segment's world-frame twist without changing any command sent to a robot.

**Architecture:** A pure `MountTwistEstimator` consumes only advancing validated `ViconSnapshot` values and returns a fixed-size estimate. The existing Vicon acquisition thread stores that estimate beside the pose in `BasePoseSample`; the 500 Hz loop copies it only to telemetry in this slice.

**Tech Stack:** C++17, Eigen, existing Vicon snapshot library, CMake/CTest.

**Method evidence:** Use the principal `SO(3)` rotation vector for
`R_k R_(k-1)^T`. The bundled Pinocchio convention already uses `log3` for
world-frame rotation errors, while Eigen's `AngleAxisd` constructs the same
principal angle-axis representation with angle in `[0, pi]`. This avoids
Euler-angle differencing and is invariant to quaternion sign (`q` versus
`-q`).

## Global Constraints

- Frames: pose is `T_W_M`; linear and angular velocity are `V_W_M`, expressed in Vicon world axes.
- Units: metres, seconds, metres/second, and radians/second.
- Differentiate only advancing Vicon frame numbers; repeated ZOH samples never update the derivative.
- `dt = (frame_k-frame_prev)/frame_rate_hz` and `alpha = 1-exp(-dt/tau_filter)`.
- Reset across invalid Mount poses, non-positive/non-finite frame rates, non-advancing frames, and prolonged gaps.
- No Vicon, file, pipe, terminal, or logging I/O in the 500 Hz thread.
- This slice must not copy Mount twist into `RobotState` or change controller output.
- Preserve the existing velocity/safety/actuation path and all unrelated dirty-tree edits.
- Do not run a robot executable. Do not commit unless Christian separately authorizes it.

---

### Task 1: Pure advancing-frame Mount twist estimator

**Files:**
- Create: `Christian_control/vicon/src/MountTwistEstimator.h`
- Create: `Christian_control/vicon/src/MountTwistEstimator.cpp`
- Create: `Christian_control/vicon/tests/test_mount_twist_estimator.cpp`
- Modify: `Christian_control/vicon/CMakeLists.txt`

**Interfaces:**
- Consumes: `const ViconSnapshot&` and constructor values `filter_tau_s`, `reset_gap_s`.
- Produces: `MountTwistEstimate MountTwistEstimator::Update(const ViconSnapshot&)` and `void Reset()`.

- [ ] **Step 1: Add failing tests for translation, rotation, filtering, and invalid timing**

```cpp
MountTwistEstimator estimator(/*filter_tau_s=*/0.0, /*reset_gap_s=*/0.20);
Check(!estimator.Update(Snapshot(100, 100.0, Pose(0, 0, 0))).valid,
      "first frame seeds history");
const auto translated = estimator.Update(
    Snapshot(101, 100.0, Pose(0.01, 0, 0)));
Check(translated.valid && Near(translated.linear_m_s.x(), 1.0),
      "10 mm per 10 ms is 1 m/s");

estimator.Reset();
estimator.Update(Snapshot(200, 100.0, YawPose(0.0)));
const auto rotated = estimator.Update(Snapshot(201, 100.0, YawPose(0.01)));
Check(rotated.valid && Near(rotated.angular_rad_s.z(), 1.0),
      "rotation log has world-frame sign and units");

const auto repeated = estimator.Update(Snapshot(201, 100.0, YawPose(2.0)));
Check(!repeated.updated, "repeated frame does not differentiate ZOH data");
```

Also assert reset/no estimate for occlusion, frame-rate zero/NaN, backwards frame number, a frame gap over `reset_gap_s`, non-finite pose, and quaternion hemisphere changes representing the same rotation.

- [ ] **Step 2: Run the new test and confirm the expected compile failure**

Run: `cmake --build Christian_control/vicon/build --target test_mount_twist_estimator -j2 && ctest --test-dir Christian_control/vicon/build -R '^vicon_mount_twist$' --output-on-failure`

Expected before implementation: build fails because `MountTwistEstimator` does not exist.

- [ ] **Step 3: Implement the fixed-size estimator**

```cpp
struct MountTwistEstimate {
    Eigen::Vector3d linear_m_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_rad_s = Eigen::Vector3d::Zero();
    std::uint32_t source_frame_number = 0;
    bool valid = false;
    bool updated = false;
};

class MountTwistEstimator {
public:
    MountTwistEstimator(double filter_tau_s, double reset_gap_s);
    MountTwistEstimate Update(const ViconSnapshot& snapshot);
    void Reset() noexcept;
private:
    // previous valid T_W_M, frame number, filtered V_W_M, and seeded flag
};
```

Find the valid segment named exactly `Mount`; compute `Log(R_k R_prev.transpose())/dt` with a numerically stable Eigen angle-axis/log implementation; set `updated=false` for a repeated frame while retaining the last estimate internally. A zero filter time means `alpha=1` for deterministic tests.

- [ ] **Step 4: Build and run all Vicon hardware-free tests**

Run: `cmake --build Christian_control/vicon/build -j2 && ctest --test-dir Christian_control/vicon/build --output-on-failure`

Expected: all Vicon tests pass, including `vicon_mount_twist`.

### Task 2: Publish twist coherently with the existing pose snapshot

**Files:**
- Modify: `Christian_control/basic_control/src/BasePose.h`
- Modify: `Christian_control/basic_control/src/ViconSource.cpp`
- Modify: `Christian_control/basic_control/src/Config.h`
- Modify: `Christian_control/basic_control/tests/test_base_pose.cpp`

**Interfaces:**
- Consumes: `MountTwistEstimate` from Task 1.
- Produces: `BasePoseSample::mount_linear_world_m_s[3]`, `mount_angular_world_rad_s[3]`, `mount_twist_valid`, and `frame_rate_hz` in the same triple-buffer publication as pose and provenance.

- [ ] **Step 1: Extend `test_base_pose` with a failing coherent-copy test**

```cpp
const BasePoseSample mapped = ToBasePoseSample(snapshot, 7, estimate);
Check(mapped.sequence == 7 && mapped.vicon_frame_number == 101,
      "pose and twist share provenance");
Check(mapped.mount_twist_valid && Near(mapped.mount_linear_world_m_s[0], 1.0),
      "Mount twist maps into the same sample");
slot.Publish(mapped);
BasePoseSample read;
slot.ReadLatest(read);
Check(read.mount_angular_world_rad_s[2] == mapped.mount_angular_world_rad_s[2],
      "triple buffer cannot tear twist from pose");
```

- [ ] **Step 2: Run `base_pose` and confirm it fails on missing fields/signature**

Run: `cmake --build Christian_control/basic_control/build --target test_base_pose -j2 && ctest --test-dir Christian_control/basic_control/build -R '^base_pose$' --output-on-failure`

- [ ] **Step 3: Add configuration and acquisition wiring**

Add `config::kViconMountTwistFilterTauS` and use the existing `kWorldHoldReanchorAfterS` (`0.20 s`) as the reset-gap argument. In `SdkViconSource::Run`, call the estimator once per newly accepted Vicon frame and pass its result to `ToBasePoseSample`. Leave `Runner.cpp` and `RobotState` unchanged in this slice.

- [ ] **Step 4: Re-run `base_pose`, `frames`, `controller`, and `world_hold`**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(base_pose|frames|controller|world_hold)$' --output-on-failure`

Expected: all pass and controller tests prove no command-law input changed.

### Task 3: Log estimator evidence as a schema version bump

**Files:**
- Modify: `Christian_control/basic_control/src/Hardware.h`
- Modify: `Christian_control/basic_control/src/Hardware.cpp`
- Modify: `Christian_control/basic_control/src/Runner.cpp`
- Modify: `Christian_control/basic_control/src/Main.cpp`
- Modify: `Christian_control/basic_control/tests/test_log_schema.cpp`
- Modify: `Christian_control/basic_control/scripts/runlog.py`
- Modify: `Christian_control/basic_control/tests/test_runlog_compat.py`

**Interfaces:**
- Produces log format 12 columns: `vicon_frame_rate_hz`, `vicon_mount_vx_mps`, `vy`, `vz`, `vicon_mount_wx_radps`, `wy`, `wz`, and `vicon_mount_twist_valid`.

- [ ] **Step 1: Make log-schema tests expect format 12 and exact new columns**

```cpp
Check(value_of("vicon_mount_vx_mps") == "nan",
      "absent twist is NaN, never plausible zero");
Check(value_of("vicon_mount_twist_valid") == "0",
      "absent twist is invalid");
```

Add a populated sample and assert all six components and validity round-trip by column name. Update Python compatibility tests to accept formats `2..12`.

- [ ] **Step 2: Run schema tests and observe the expected failure**

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(log_schema|runlog_compat)$' --output-on-failure`

- [ ] **Step 3: Append fields without reordering prior columns**

Copy the values from the one `BasePoseSample` already read at cycle start into `LoopLogSample`; append headers and rows in `WriteCsvHeader`/`WriteCsvRow`; update preamble text to `log_format = 12`. Do not read the slot a second time.

- [ ] **Step 4: Verify both projects and inspect the diff**

Run: `ctest --test-dir Christian_control/vicon/build --output-on-failure`

Run: `ctest --test-dir Christian_control/basic_control/build -R '^(base_pose|frames|world_hold|reactive_law|controller|log_schema|runlog_compat)$' --output-on-failure`

Run: `git diff --check`

Review checkpoint: confirm nonzero Mount twist appears only in snapshot/log fields and nowhere in `RobotState`, `Frames::ArmControllerState`, or `TrackingController::DesiredVelocity`.
