# Vicon integration: capability audit, frame contract and phased design

Date: 2026-08-08
Status: proposal for approval. No code written. No file in project 907 was read
other than read-only, and none was modified.

## Problem

We want a Vicon-driven monitor in `HumanSL_MAIN` that reports where the rig
actually is, compares it against the URDF, and can be developed without
continuous lab access. Project 907 was believed to hold a reusable Vicon
subsystem worth porting.

The audit contradicts that belief, and the contradiction is the single most
important finding in this document. It is stated first because it changes what
"integration" means here.

## The finding that reframes everything

**Project 907's Vicon code has never run.** Its `DataStreamClient.h` is not the
Vicon SDK — it is a 167-line hand-written stub in which every method is an
inline no-op returning a default-constructed struct
(`907/DataStreamClient.h:1-3`, `:128-151`). The defaults are chosen so the
program appears to succeed: `Output_Connect` carries `Result = Success` even
with no network (`:31`), and `SubjectCount` defaults to `0` with the author's own
comment `// 0 → marker loops skipped` (`:68`). The root `CMakeLists.txt` links
no Vicon library at all (`907/CMakeLists.txt:59-65`), and the project root is on
the include path specifically so the stub resolves (`:28`). The root target
cannot even build, because `CMakeLists.txt:13` points at a `Christian/` examples
directory that is empty.

Two consequences follow. First, no marker, force or registration result 907 ever
produced is evidence of anything; the code describes intent recovered from
source, not a working pipeline. Second, the stub is missing exactly the methods
the original code never called — `SetAxisMapping`, `GetLatencyTotal`,
`GetFrameRate`, `GetSegmentGlobal*` — which independently confirms their absence
rather than merely failing to find them.

**907 also does not contain most of what we assumed we were porting.** It has no
Nexus segment pose (zero hits for `GetSegmentGlobalTranslation`,
`GetSegmentGlobalRotationMatrix`, `GetSegmentGlobalRotationQuaternion`,
`GetSegmentCount`, `GetSegmentName`), no circle fitting, no sphere fitting, no
latency query, no frame-number tracking, no replay, no MATLAB or visualisation
of any kind, and no torso or backpack frame. Its entire dual-arm frame model is
a single X-axis sign flip at `01-actuator_low_level_velocity_control.cpp:2013`,
and the URDF it depends on lives at `/home/hector/Documents/urdf3/...` — another
user's home directory, absent from the repository.

**Meanwhile, `HumanSL_MAIN` already has more Vicon capability than 907 does.**
It vendors the real SDK (version 1.12.0, revision 145507, extracted from the
binary's `GetVersion` immediates) at `third_party/vicon_api/`, committed to git.
It has a working `ViconInterface`, and `ViconDataStream/src/ViconInfo.cpp`
contains a complete, purpose-built torso-marker-to-arm-base registration —
including the ring-circumcentre frame fit that the proposal treated as a novel
capability to add.

So this is not a port. It is a **rewrite of `HumanSL_MAIN`'s own Vicon layer,
using `HumanSL_MAIN`'s existing algorithms as the reference implementation, with
907 contributing one 40-line function's worth of ideas.** The rest of this
document is written on that basis.

---

# 1. Capability matrix

## 1.1 Summary

Verdicts: **Port now** = build in phase 1–3. **Port later** = real, but after the
core lands. **Adapt** = the idea is right, the existing implementation is not.
**Reject** = do not carry forward.

| # | Capability | In HumanSL_MAIN | In 907 | Verdict |
|---|---|---|---|---|
| 1 | Vicon connection | `ViconInterface.cpp:18-45`, real SDK | `01-…cpp:2937-2979`, stub only | **Adapt** — rewrite; keep host, fix the hang |
| 2 | Frame acquisition | `ViconInterface.cpp:55-63` | `01-…cpp:1860-1863` | **Adapt** — both are wrong for their stream mode |
| 3 | Marker lookup | `ViconInterface.cpp:78-157` | `01-…cpp:1876-1905` | **Adapt** — keep the API shape, fix occlusion + O(n) scan |
| 4 | Nexus segment pose | **Absent** — `EnableSegmentData()` never called | **Absent** — zero hits | **Port now** — new code, no precedent in either project |
| 5 | Marker-based frame fitting | `ViconInfo.cpp:1130-1212` `calculateFramePose` | `01-…cpp:452-493` Kabsch (dead), `:2118-2138` Gram-Schmidt (dead) | **Adapt** (ring fit) + **Port now** (general Kabsch) |
| 6 | Object quality | `MarkerData::occluded` only | Nothing | **Port now** — new; graded, not boolean |
| 7 | Latency and frame rate | `getFrameRate()` exists, **dead**; no latency | **Absent** entirely | **Port now** |
| 8 | Force, moment, CoP | Force only, `ViconInterface.cpp:185-203` | Force in loop; moment/CoP read once, discarded | **Port later** (phase 5) |
| 9 | Recording and replay | **Absent** (raw marker CSV in `main.cpp` only) | Recording yes (19 `.txt` files), replay **absent** | **Port now** — phase 1, both halves |
| 10 | Vicon→robot registration | `ViconInfo.cpp:335-749`, complete and purpose-built | `01-…cpp:679-867`, sliding-window Kabsch, never run | **Adapt** from HumanSL_MAIN; **Reject** 907's |
| 11 | Kalman filtering | **Absent** | `01-…cpp:438-448` `Kalman3D` | **Port later** (phase 6) — rebuild, do not lift |
| 12 | URDF frame comparison | `print_dual_arm_fk`, `compare_end_effector_pose` (robot-vs-URDF, not Vicon) | **Absent** | **Port now** — phase 3, extend the existing pattern |

## 1.2 Per-capability detail

### 1. Vicon connection

*HumanSL_MAIN*: `ViconInterface::connect()`,
`ViconDataStream/src/ViconInterface.cpp:18-45`. Default host
`"localhost:801"`; callers pass `"192.168.128.206"`
(`test_vicon.cpp:14`, `main.cpp:345`). *907*: `01-…cpp:2939`, same IP hardcoded,
no port, no override.

*Dependencies*: `libViconDataStreamSDK_CPP.so` plus four bundled Boost 1.75.0
libraries, all at `third_party/vicon_api/lib/` and committed to git. Nothing
external needs installing.

*Frames / units*: none — this is a transport concern.

*Timing*: `connect()` blocks unboundedly. `ViconInterface.cpp:39-41` is
`while (client_->GetFrame().Result != Result::Success) { sleep_for(10ms); }`
with no timeout and no retry cap.

*Error handling*: TCP failure logs to `stderr` and returns `false`. Both callers
treat that as fatal.

*Known failure modes*: if the TCP connect succeeds but the server is not
streaming — capture stopped, no subjects loaded — the loop above **never
exits**, silently, with no log line. This is the most likely failure in a real
lab session and currently presents as a hung process. In 907 the failure mode is
worse: the stub reports success unconditionally.

*Verdict*: **Adapt.** Keep the host and the SDK binding; replace the blocking
loop with a bounded wait that reports how long it waited and why it gave up.

### 2. Frame acquisition

*HumanSL_MAIN*: `getFrame()` (`ViconInterface.cpp:55-58`) and `getFrameNumber()`
(`:60-63`). *907*: `01-…cpp:1862`, return value discarded.

*Dependencies*: the SDK's stream mode, which neither project sets correctly.
`HumanSL_MAIN` never calls `SetStreamMode` at all, so it runs in the documented
default **ClientPull** (`DataStreamClient.h:1892`). 907 sets `ServerPush` in
`main` (`:2979`) and then overwrites it with `ClientPullPreFetch` inside the
worker thread (`:1813`).

*Timing*: `main.cpp:361-375` polls `getFrameNumber()`, sleeps 2 ms if unchanged,
and only calls `getFrame()` when it has changed. **This is inverted for
ClientPull.** The SDK documents `GetFrameNumber()` as "the number of the last
frame *retrieved*" (`DataStreamClient.h:2124`), and in ClientPull nothing is
retrieved until `GetFrame()` is called — so the number cannot change until the
call the loop is gating on. `test_vicon.cpp:31-40` replicates it; `main.cpp`'s
record thread (`:488-497`) uses the pattern without ever calling `getFrame()` at
all. 907 has an unconditional extra `sleep_for(10ms)` at `:2093` after a
correctly phase-locked `sleep_until` at `:2028`, halving its intended 100 Hz to
roughly 50 Hz while still passing `dt = 0.01` to its filter.

*Known failure modes*: silent duplicate frames; a nominal rate that is neither
measured nor verified against `getFrameRate()`; a `dt` that is wrong by 2× in
907.

*Verdict*: **Adapt.** Set the stream mode explicitly, drive the loop from
`GetFrame()`, and derive freshness from the frame number the SDK reports after
that call rather than before it.

### 3. Marker lookup

*HumanSL_MAIN*: `getMarkerPosition(name)`, `getMarkerPositions(vector<name>)`,
`getMarkerPositions(prefix)`, `getUnlabeledMarkers()`
(`ViconInterface.cpp:78-182`). *907*: index enumeration over subjects, with
subject names `"Original"` and `"Effinal"` (`01-…cpp:1804-1805`).

*Frames*: Vicon global. *Units*: **millimetres** from the SDK
(`DataStreamClient.h:4469`); `ViconInterface` does **not** convert, so
`MarkerData` is in mm. Conversion happens one layer up as a bare literal
`/1000` at eleven separate sites in `ViconInfo.cpp` (`:19-21`, `:30-32`,
`:39-41`, `:185-187`, `:241-243`, `:266-268`, `:291-293`, `:322-324`, `:1083`,
`:1113`, `:1116`) with no named constant. 907 converts at the read site
(`:1893-1895`), which is better.

*Error handling*: a name that is not found returns `occluded = true` with
**`x`, `y`, `z` left uninitialised** (`ViconInterface.cpp:79-83`, `:109`).
Missing and occluded are indistinguishable to the caller. When a marker is
genuinely occluded the SDK still returns `Result::Success` with stale or zero
translation, and `ViconInterface` copies those numbers in alongside the flag
(`:98-103`) — so the flag is the only valid signal and the coordinates must be
discarded.

*Known failure modes*: `main.cpp:549` defends with `std::isnan`; `ViconInfo.cpp`
does not. Three averaging loops skip occluded markers from the sum but still
divide by the full count (`ViconInfo.cpp:237-255` head, `:262-280` left hand,
`:287-305` right hand), so one occluded marker silently drags the average toward
the Vicon origin — and that average feeds `state_monitor`, which drives the
state machine. `updateHumanInfo` (`:184-226`) ignores the occlusion flag
entirely and its `*_occluded` fields are only ever written `false`. Separately,
each lookup is an O(subjects × markers) scan with a string compare per marker,
performed roughly 40 times per frame while holding the mutex the 500 Hz control
threads contend on (`ViconInfo.cpp:359`, `main.cpp:758`, `:771`).

*Verdict*: **Adapt.** Keep name-based lookup as the user-facing idea; move the
scan to a once-per-frame index build, convert to metres exactly once at the
boundary, and make "missing", "occluded" and "valid" three distinct states
rather than two overloaded ones.

### 4. Nexus segment pose

*HumanSL_MAIN*: **absent.** `EnableSegmentData()` is never called
(`ViconInterface.cpp:31-36` enables only marker, unlabeled-marker and device
data), so no segment query would return anything. A `SegmentData` struct is
declared at `ViconInterface.h:23-39` and is entirely unused. *907*: **absent.**
`EnableSegmentData()` is called once (`:1812`) and no segment is ever queried.

*Dependencies*: requires `EnableSegmentData()` before `GetFrame()`, plus a
subject actually labelled and solved in Nexus.

*Frames*: segment global translation and rotation are in Vicon global.
*Units*: translation **mm**, rotation as matrix / quaternion / Euler depending
on the accessor.

*Known failure modes*: a segment whose subject exists but is unlabelled in the
current frame returns an occluded result; a segment template built in Nexus
carries its **own** origin and axis convention, which is not the URDF's and not
the marker cluster's centroid.

*Verdict*: **Port now.** This is genuinely new code with no precedent in either
project, and it is the preferred source for structure tracking. It is also the
single largest new capability in this design.

### 5. Marker-based frame fitting

*HumanSL_MAIN*: `calculateFramePose(p1, p2, p3, p4, z_offset, p1_in_positive_x,
p4_in_positive_z)`, `ViconInfo.cpp:1130-1212`. It computes the circumcentre of
three markers, resolves the plane-normal sign with a fourth point, sets +x from
centre toward p1, and offsets along +z. This is a **ring/circle fit** and is
already in the repository — the proposal treated it as new.

*907*: `compute_rigid_transform` (`01-…cpp:452-493`) is a clean, correct Kabsch
with reflection correction returning a 4×4 — and has **no call site anywhere**.
`build_frame_from_markers` (`:2118-2138`) is a three-marker Gram-Schmidt, also
dead. `computeSafeTarget` (`:548-582`) is a three-point cross-product plane fit
whose only call site is commented out and whose input matrix is never populated.

*Frames*: markers in Vicon global; output is a pose of the fitted structure in
Vicon global. *Units*: `calculateFramePose` takes metres (callers divide by 1000
first) and its `z_offset` is metres — **0.133 m** for an arm base marker ring
(`ViconInfo.cpp:1119`) and **0.12 m** for an end-effector marker set (`:1088`).

*Timing*: per frame, no state.

*Error handling*: collinear points fall back to the centroid when the
determinant is below `1e-10` (`ViconInfo.cpp:1155-1157`) — which silently
returns a position with a **meaningless orientation** rather than reporting
degeneracy. A separate sentinel path writes `direction = (0,0,1)` and
`centroid = (0,0,0)` for a faulty tube (`ViconInfo.cpp:136-139`), detected by
exact float comparison against zero (`:124-134`), so a legitimately near-zero
coordinate is misread as invalid.

*Known failure modes*: circumcentre of three markers is highly sensitive to
marker noise when the three are near-collinear; the fit uses exactly three
markers regardless of how many are visible, so extra markers add no robustness;
there is no residual reported, so a bad fit is indistinguishable from a good one.

*Verdict*: **Adapt** the ring fit (keep the geometry, add a residual and an
explicit degeneracy result, make the marker count variable), and **port now** a
general least-squares rigid registration in the shape of 907's
`compute_rigid_transform`. That function is 40 lines and dead in 907; writing it
fresh against our own template type is cheaper than lifting it, and the
reflection-correction detail is the only thing worth copying.

### 6. Object quality

*HumanSL_MAIN*: a single `bool occluded` per marker
(`ViconInterface.h:15`). *907*: nothing — it never reads the `Occluded` field at
all, only `.Result`, so it would accept `(0,0,0)` as a valid position.

*Verdict*: **Port now**, as new work, and deliberately **graded rather than
boolean**, per the project's standing preference for a measure over a veto. A
tracked structure should report visible marker count, fit residual and staleness,
so the layer with context decides what to do — not the tracker.

### 7. Latency and frame rate

*HumanSL_MAIN*: `getFrameRate()` exists (`ViconInterface.cpp:65-74`) and has
**zero callers**. No latency, no timecode, no timestamp of any kind is read;
`ViconInterface.h:8` includes `<chrono>` and uses it for nothing. `MarkerData`
carries no timestamp, so no consumer can tell how old a sample is or compute a
correct `dt`. *907*: **absent entirely** — zero hits for `GetLatencyTotal`,
`GetLatencySampleCount`, `GetFrameRate`, `GetTimecode`, `GetFrameNumber`.

*Timing assumptions in force today*: `main.cpp:429` comments "records at default
100 Hz"; `README.md:91` says "~100 Hz"; the robot loop runs at 500 Hz
(`main.cpp:21`). None of these is verified against the SDK, and there is no
interpolation or staleness handling between the two rates.

*Known failure modes*: a real 100 Hz Vicon pipeline carries roughly 10–30 ms of
latency. Every current consumer treats the sample as current-instant.

*Verdict*: **Port now.** `GetLatencyTotal()`, `GetFrameRate()`, `GetTimecode()`
and the per-sample latency breakdown are cheap to read and are the only way to
make a recording replayable with correct timing.

### 8. Force, moment and centre of pressure

*HumanSL_MAIN*: `getForcePlateVector(fplate_left, fplate_right)`,
`ViconInterface.cpp:185-203`. Force only. It does not check `connected_`, does
not check either plate's `Result`, and unconditionally writes indices `[0..2]`
of both output vectors; callers happen to size them at 3 (`main.cpp:275-276`), so
it is safe today but the contract is unenforced. Note the deliberate swap: plate
index 1 is reported as left, index 0 as right. *907*: force read every cycle for
plates 0 and 1 (`01-…cpp:1865-1866`), logged and never used for control; moment
and CoP read **once at startup** and never read again (`:3000-3019`).

*Units*: Vicon returns force in **newtons**, moment in **newton-millimetres**,
CoP in **millimetres**. Neither project converts moment or CoP, and 907 does not
document force units at all — a data-interpretation trap for anyone reading
`zforce*.txt`.

*Known failure modes*: force plates typically stream at 1000 Hz against 100 Hz
mocap. Neither project reads subsamples, so roughly 90% of the force data is
discarded with no anti-aliasing. Neither checks the plate count before indexing.

*Verdict*: **Port later**, phase 5, and optional. It is genuinely useful for a
supernumerary-limb project — ground reaction tells you what the wearer's balance
is doing — but it is not needed for geometry, it doubles the units surface, and
subsample handling is real work.

### 9. Recording and replay

*HumanSL_MAIN*: no Vicon recorder exists. `main.cpp`'s record thread writes raw
marker columns into the run CSV (`:546-604`) **in millimetres**, in the same rows
as tube and base-frame columns that are in metres. *907*: 19 append-mode `.txt`
files plus one true CSV (`ee_error_log.csv`, columns `time_s,e_x,e_y,e_z,d`).
Two of the files stream a 4×3 Eigen matrix through `operator<<`, which renders as
four newline-separated rows with the timestamp prefixing only the first, so those
files are not row-aligned with the others. Files open in append mode with no run
delimiter, so consecutive runs concatenate. The timestamp is whole seconds, which
is not a usable index at 50 Hz. **No replay exists in either project.**

*Verdict*: **Port now**, phase 1, and both halves together. This is the item that
unblocks everything else: with a recorder and a replay source, phases 2, 3 and 4
are developed and tested offline, and the lab is needed only to capture. The
existing formats are not a starting point — the recorder should follow the house
run-log convention instead (see §5, phase 1).

### 10. Vicon-to-robot registration

*HumanSL_MAIN*: this already exists and is purpose-built for our rig.
`updateViconInfo` (`ViconInfo.cpp:335-749`) reads a reference body
`human_ref_base1..4`, arm base rings `right_base1..3` and `left_base1..3`
(`:371-384`), fits each with `calculateFramePose`, computes
`ref_to_left = ref_base⁻¹ · left_base` and the right equivalent (`:499-500`),
averages them over 100 frames on SE(3) via `Rot3::Logmap`/`Expmap`
(`:660-685`) with an outlier gate rejecting rotations more than 0.3 rad from the
running mean (`:518-524`), then latches and thereafter derives
`left_base = avg_ref_base · ref_to_left` (`:554-555`).

*907*: a sliding-window Kabsch (`01-…cpp:679-867`) pairing the end-effector
marker centroid against `tool_pose_{x,y,z}` from Kortex, window 200, minimum 50
samples, EMA smoothing with α = 0.1, a condition-number gate at 1000 with a
Tikhonov fallback.

*Frames*: HumanSL_MAIN maps Vicon global → per-arm base pose in Vicon global.
907 maps Vicon global → the left arm's Kortex base frame. *Units*: metres on
both sides once converted.

*Known failure modes, HumanSL_MAIN*: `ref_base_current` is pushed into the
history array at `:507` even on paths where the pose update was skipped at
`:494`, so a default-constructed identity pose can pollute the running average.
`updateViconInfo` and `calibrate_base_frame` are roughly 320 near-identical
lines (`:335-749` vs `:752-1073`) with the filter and averaging blocks copied
verbatim, so every fix must be applied twice; `calibrate_base_frame` has no
callers.

*Known failure modes, 907*: `R_v2k` and `t_v2k` are output-only parameters that
are **not written until 50 samples accumulate**, so for the first ~50 cycles the
robot is commanded from uninitialised stack memory (`:939-946`). The
`rms_error = 1e3` "do not trust" sentinel is generated and **never checked by any
caller**, and the diagnostic that would have printed it is commented out
(`:733`). Methodologically the fit pairs a marker-cluster centroid against the
Kortex tool origin — two physically different points whose offset rotates with
the wrist — which is the classic case requiring hand-eye (AX = XB) calibration,
not point-set Kabsch. And the buffer fills from an arm servoing toward one fixed
target, so the samples are near-collinear, which is exactly the degeneracy the
condition gate is patching over.

*Verdict*: **Adapt HumanSL_MAIN's**, which is the right method for our geometry
(rigid marker sets on known structures, not a moving tool). **Reject 907's**
outright: it is unvalidated, it commands from uninitialised memory, and its
formulation is wrong for the problem it solves.

### 11. Kalman filtering

*HumanSL_MAIN*: absent. *907*: `Kalman3D` (`01-…cpp:438-448`) — a 3-state
position-only filter with `F = I`, `B = dt·I`, `H = I`, isotropic
`P₀ = 1e-3`, `Q = 1e-2`, `R = 1e-3`. It is a function-local `static` (`:949`), so
it is not thread-safe and cannot be reset without restarting the process. With
`Q` ten times `R` the steady-state gain is roughly 0.9 — it barely smooths. `Q`
is not scaled by `dt`, and `dt` is hardcoded to 0.01 while the true loop runs at
about half that rate. It uses the naive `(I − K)P` covariance update rather than
the Joseph form, so symmetry can degrade over a long run. The guard before the
measurement update, `matrixeff.rows() >= 3`, tests a **static allocation size**
that is always true and says nothing about data freshness.

907 also constructs a Butterworth filter that is never applied on the live path
(`:1818-1822`, marked unused in the signature at `:916`) and whose implementation
is wrong three ways: it subtracts past *inputs* instead of past *outputs* so it
is an FIR not an IIR, it indexes `b[3]`, `b[4]`, `a[3]`, `a[4]` on three-element
vectors which is out-of-bounds, and it passes input through unfiltered until its
history fills.

Neither project has any outlier rejection on Vicon data — no RANSAC, no
Mahalanobis gating on the innovation, no marker jump limit, no rigidity check.

*Verdict*: **Port later**, phase 6, as an experiment rather than a component, and
**rebuild rather than lift**. The filter we would want estimates pose on SE(3)
with a velocity state and gates on the innovation; almost nothing of `Kalman3D`
survives that change except the idea.

### 12. URDF frame comparison

*HumanSL_MAIN*: the *pattern* exists but not against Vicon.
`compare_end_effector_pose` and `print_dual_arm_fk`
(`Christian_control/basic_control/CMakeLists.txt:235`, `:329`) compare Pinocchio
FK against the robot's own reported pose; `print_dual_arm_fk` is explicitly
"cannot connect or command, safe to run with the robot off". `FramePrint.h`
already owns the dual-arm FK verification table. *907*: absent — it queries an
unqualified `"EndEffector_Link"` on a 14-DoF model (`:1992`, `:2000`), which
could not be right for a genuine dual-arm URDF.

*Dependencies*: Pinocchio 3.4.0, vendored. **Constraint**: GTSAM and Pinocchio
must never share a translation unit — colliding `boost::serialization` overloads
for `Eigen::Matrix` (`PinocchioKinematicsAdapter.h:6-13`). The existing Vicon
code is written in `gtsam::Pose3`; new Vicon code must therefore be **Eigen-only**
if it is to be linked from both sides.

*Verdict*: **Port now**, phase 3, extending the existing comparison pattern
rather than inventing one.

---

# 2. Canonical frame and transform contract

## 2.1 Notation

`^A T_B` is the pose of frame B expressed in frame A: the rigid transform that
takes a point written in B and returns it written in A.

```
p_A = ^A T_B · p_B          composition:  ^A T_C = ^A T_B · ^B T_C
                            inverse:      ^B T_A = (^A T_B)⁻¹
```

In code this is `Eigen::Isometry3d`, named `A_T_B`. Translations are **metres**,
angles **radians**, rotations stored as matrices. Millimetres exist only inside
the Vicon boundary layer and are converted exactly once, at the point of SDK
read, in one named function — not as eleven scattered `/1000` literals.

## 2.2 The frames

These are distinct frames. The contract's whole purpose is to stop them being
conflated.

| Frame | What it physically is | Source of truth | Exists today? |
|---|---|---|---|
| `vicon_world` | The Vicon global origin as configured on the host | Vicon Nexus/Tracker | Yes, implicitly |
| `torso_body` | The pose Nexus reports for the torso subject's segment | Nexus segment template | No — segments not enabled |
| `backpack_mount` | The rigid structure the two arms are bolted to | Physical rig | No name in code |
| `mount` | The URDF root link, at the **midpoint of the two base origins** | `GEN3_dual_mounted.urdf:50` | **Yes** |
| `left_base_marker_frame` | Frame fitted to the left arm's base marker ring | `left_base1..3` markers | Yes, via `calculateFramePose` |
| `right_base_marker_frame` | Frame fitted to the right arm's base marker ring | `right_base1..3` markers | Yes, same |
| `left_base_link` | URDF link `leftbase_link` | `GEN3_dual_mounted.urdf:329` | Yes |
| `right_base_link` | URDF link `base_link` | `GEN3_dual_mounted.urdf:58` | Yes |
| `right_tool` | URDF link `ConfiguredTool_Link` | `GEN3_dual_mounted.urdf:299` | Yes — **right arm only** |
| `left_flange` | URDF link `leftEndEffector_Link` | `GEN3_dual_mounted.urdf:554` | Yes |
| `tool_marker_frame` | Frame fitted to a marker cluster on the tool | Markers | Partially — `updatePoseInfo1`, offset 0.12 m |
| Nexus segment template frames | Whatever origin and axes the Nexus template defines per subject | Nexus `.vsk` | No |

Three naming facts that must not be smoothed over. First, the URDF's link
naming is **not** a symmetric prefix scheme: the right arm is unprefixed
(`base_link`, `Actuator1`) and the left carries a bare `left` prefix with no
separator (`leftbase_link`, `leftActuator1`). Second, `mount` is deliberately not
called `world` — `Christian_control/docs/decisions/planner-frame-mount.md:15-19`
records the reason: once the rig is worn, a room frame becomes real and must be
called `world`, and renaming later would leave half the codebase meaning the
backpack by that name. Third, **`mount` and `backpack_mount` are not obviously
the same frame**. `mount` is a derived construction at the midpoint of two base
origins; the physical backpack plate is a different object with its own natural
origin. They may coincide by choice, but that is a decision to make, not a fact
to assume.

## 2.3 Required fixed transforms

### `^mount T_right_base_link` — **known, exact, and checked in**

`GEN3_dual_mounted.urdf:52-56`:
```
rpy = (1.2085, 0, 0)     xyz = (0, -0.0567075, 0)
```

### `^mount T_left_base_link` — **known, exact, and checked in**

`GEN3_dual_mounted.urdf:323-327`:
```
rpy = (-1.2085, 0, 0)    xyz = (0, +0.0567075, 0)
```

URDF `<origin>` composes as translation then rotation. Both derive from
`Christian_control/basic_control/config/dual_arm_mounting.yaml`
(`base_separation_m: 0.113415`, `mount_tilt_rad: 1.2085`,
`mount_origin: base_midpoint`), and the agreement between YAML and URDF is
enforced to 1e-9 by `tests/test_dual_arm_mounting.cpp:133-154`.

**Genuinely fixed?** Yes, in the sense that the two arms are bolted to one plate.
But the numbers are **inherited, not measured** — `dual_arm_mounting.yaml:11-17`
says so in as many words: "They arrived fully formed in commit af116a5c … with no
measurement note … They describe the mount's intended geometry, not a survey of
the physical rig." Any Vicon comparison will be measuring the real rig against an
*intended* geometry. That is precisely what makes phase 3 worth doing, and it is
also why phase 3 must report a discrepancy rather than assert a fault.

Note also that `/home/christian/msc_project/sim/scene.xml:18,22` encodes a
**different** mounting geometry — tilt 0.8457 rad and separation 0.2 m against
the URDF's 1.2085 rad and 0.113415 m. The two projects do not agree, neither
cites a survey, and `docs/superpowers/specs/2026-08-07-shared-robot-io-boundary-design.md`
lists resolving this as an open question.

### `^EndEffector_Link T_ConfiguredTool_Link` — **known, exact, and provenanced**

`GEN3_dual_mounted.urdf:306-310`: `xyz = (0, 0, 0.12)`, no rotation. Read from
the robot itself on 2026-08-05 via `ControlConfig::GetToolConfiguration`. It is a
live robot **setting**, not a property of the Gen3 — if the mounted tool changes,
re-read it. The left arm has **no** tool link; its chain ends at the bare flange.
Left and right "tool" points are therefore not the same point on the arm and must
never be compared as though they were (`Config.h:88-92`).

### `^backpack_mount T_left_base_marker_frame`, `^backpack_mount T_right_base_marker_frame` — **unknown, must be measured**

What exists today is the *fitted* frame plus a hand-chosen axial offset:
`calculateFramePose(..., z_offset = 0.133, ...)` at `ViconInfo.cpp:1119`. That
0.133 m is the marker-ring-to-`base_link` distance along the ring normal. It is a
scalar with no recorded provenance, and it captures only the axial component —
the ring's rotation about its own normal is set by "+x points from centre toward
marker 1", which depends entirely on which physical marker was labelled `1`.

**Genuinely fixed?** Yes if the markers are stuck to the arm bases, which they
almost certainly are given the names. But the full six-component transform is not
written down anywhere, and the two components that are — 0.133 m and the marker-1
convention — are unverified.

### `^torso_body T_backpack_mount` — **unknown, and possibly not fixed at all**

This is the transform the proposal most needs and the one we can say least about.

The existing code's reference body is `human_ref_base1..4`
(`ViconInfo.cpp:371-374`). The name is ambiguous in exactly the wrong way:
`human_` suggests the wearer, `_ref_base` suggests a reference base. The code
**treats it as rigidly related to the arm bases** — it averages
`ref_base⁻¹ · left_base` over 100 frames, latches it, and thereafter derives both
arm bases from the reference body forever (`:499-500`, `:554-555`).

That assumption is safe if the four markers are on the rigid backpack plate. It is
**wrong** if they are on the wearer's torso, because then the transform is a
function of posture — and the failure would be silent and plausible-looking: the
100-frame average would converge fine during a calibration in which the wearer
stood still, then drift as soon as they moved, with the arms' apparent poses
moving with the wearer's breathing and gait.

**I cannot determine this from the code, and neither can any amount of further
reading.** It is a fact about tape and Velcro. It must be settled physically, and
§4 gives the experiment that settles it from a recording alone.

### `^tool_marker_frame T_right_tool` — **partially known, weakly grounded**

`updatePoseInfo1` (`ViconInfo.cpp:1078-1106`) fits an end-effector marker set with
`z_offset = 0.12` and then runs `inverseForwardKinematics` to back out a base
pose. The 0.12 numerically matches the flange-to-tool offset, which is suggestive
but not evidence — it may be coincidence, or the marker plate may genuinely sit
at the flange. Unresolved; phase 6.

### `^vicon_world T_mount` — **the thing the whole system computes**

Not fixed — this is the live output. Today it does not exist at all: `mount` has
no Vicon-side definition. `Config.h:74-79` and `PathFrames.h:32-34` both reserve
the seam for it, documented rather than stubbed, "because an identity function
nothing calls is clutter".

## 2.4 What must not be assumed

A marker frame, a Nexus segment frame and a URDF frame are three different things
even when they describe the same physical object. The marker frame's origin and
axes come from where the markers were stuck and which one was labelled `1`. The
Nexus segment frame's origin and axes come from the `.vsk` template, chosen by
whoever built the subject in Nexus. The URDF frame's origin and axes come from
Kinova's kinematic convention. **Every one of these three needs its own measured
transform to the others; none of them is identity by default.** Assuming
otherwise is the single most likely source of a constant offset that looks like a
calibration error and is actually a definition error — the same class of mistake
as the 120 mm tool-versus-flange bug of 2026-08-06.

---

# 3. Revised architecture

## 3.1 The three options

**A — several independent getters on `ViconInterface`.** Each downstream
consumer asks for what it needs, when it needs it.

*Cost*: this is what exists, and its failure mode is already visible.
`updateViconInfo` makes roughly 40 separate O(subjects × markers) scans per frame
(`ViconInfo.cpp:371-409`), each with a string compare per marker, all while
holding the mutex that the 500 Hz control threads contend on. Worse, there is no
guarantee that two getters called in sequence describe the *same* frame — the SDK
buffer can advance between them. Every consumer independently re-derives units,
re-handles occlusion, and re-decides what "stale" means, which is precisely how
the three divergent occlusion behaviours in `ViconInfo.cpp` arose. **Reject.**

**B — one `ViconSnapshot` produced once per frame, with pure downstream
modules.** One `GetFrame()`, one pass over the SDK, one immutable value
containing everything that frame carried. Everything downstream is a pure
function of that value.

*Benefit*: frame coherence becomes a type-level guarantee rather than a
convention. Units convert once. Occlusion is handled once. And decisively — a
snapshot is a value, so it can be **written to disk and read back**, which is
what makes replay possible at all. This also matches `docs/architecture.md:58`,
which already states the principle for the robot side: "hardware feedback is
converted once into an immutable state snapshot per tick." **Recommended, and it
matches your stated preference.**

**C — copy or share 907's subsystem.** *Reject*, on evidence rather than taste.
There is no subsystem to copy: no real SDK, no segment pose, no circle fit, no
latency, no replay, no torso frame, and a stub that guarantees none of it has ever
executed. Its one clean liftable function, `compute_rigid_transform`, is 40 lines
and dead. Sharing is also impossible in the ordinary sense — 907 is not a git
repository, so there is no history, no branch, and no mechanism to track
divergence. It stays what you designated it: a read-only reference to consult.

## 3.2 The snapshot

Eigen-only. No SDK types, no GTSAM, no Pinocchio, no Kortex. That last point is
what lets it be linked from the planner side and the controller side without
reopening the GTSAM/Pinocchio translation-unit conflict.

```
ViconSnapshot
  frame_number          uint32     from the SDK, after GetFrame()
  host_time_s           double     steady_clock at the moment of the read
  frame_rate_hz         double     GetFrameRate()
  latency_total_s       double     GetLatencyTotal()
  latency_samples[]     (name, seconds) pairs from the SDK breakdown
  markers[]             name, position_m (Vicon world), status
  unlabeled_markers[]   position_m, sdk_marker_id
  segments[]            subject, segment, ^vicon_world T_segment, status
  force_plates[]        force_N, moment_Nm, cop_m, status      (phase 5)
  acquisition_ok        bool + reason
```

Four points on this shape. Marker `status` is a three-valued enum — `kValid`,
`kOccluded`, `kNotPresent` — never a bool, because collapsing "the SDK says this
marker is hidden" and "no marker by that name exists" is the current bug.
Positions are in metres, converted once. Segments carry a full pose, which is what
distinguishes them from markers. And unlabeled markers keep the **SDK's**
`MarkerID`, not the loop index — `ViconInterface.cpp:175` currently assigns `i`,
which makes the IDs meaningless across frames.

Lookup by name is a map built once when the snapshot is constructed, so a
downstream module doing twenty lookups pays one pass, not twenty scans.

## 3.3 The source interface

```
class ViconSource {
  virtual std::optional<ViconSnapshot> Next() = 0;    // nullopt = stream ended
  virtual SourceInfo info() const = 0;                // live host, or file + range
};
```

Two implementations: `ViconLiveSource` wraps the SDK; `ViconReplaySource` reads a
recording. **Every downstream module takes a `ViconSnapshot`, never a
`ViconSource` and never the SDK.** That is what makes phases 2, 3 and 4 testable
with no lab and no hardware.

`ViconLiveSource` owns its own SDK client rather than wrapping `ViconInterface`,
for two concrete reasons: `ViconInterface` never calls `EnableSegmentData()`, so
it structurally cannot deliver segment poses; and changing its enable set or its
`connect()` loop would change behaviour for `main.cpp`, which your constraint
forbids. `ViconInterface` therefore stays byte-identical and keeps working. The
migration path — reimplementing `ViconInterface` on top of the snapshot once the
snapshot is trusted — stays open but is explicitly not in this plan.

## 3.4 Module boundaries

Chosen by responsibility, not by count. Three bands, and the dependency arrows
only ever point downward.

**Boundary band** — the only code that includes the Vicon SDK header:
`ViconSnapshot.h` (the data contract), `ViconSource.h` (the interface),
`ViconLiveSource.{h,cpp}`, `ViconRecorder.{h,cpp}`, `ViconReplaySource.{h,cpp}`.

**Geometry band** — pure, Eigen-only, no SDK, no I/O, trivially unit-testable:
`FrameContract.h` (frame names and the fixed transforms, derived from the URDF,
never hardcoded), `MarkerTemplate.{h,cpp}` (a named marker set plus least-squares
rigid registration onto it), `RingFit.{h,cpp}` (the adapted circle/plane fit),
`StructureTracker.{h,cpp}` (source selection and quality).

**Application band**: `UrdfCompare.{h,cpp}` (Pinocchio FK against tracked poses —
the only place Pinocchio enters) and `VicalMonitorMain.cpp` (the terminal
display). Neither is linked by anything else.

## 3.5 Where it lives, and why not in `ViconDataStream/`

`ViconDataStream/` is built only by the root project, and **the root project
cannot configure**: `CMakeLists.txt:118` adds a `TrajectoryRealTime`
subdirectory whose `CMakeLists.txt` does not exist, and
`TrajectoryExecution/CMakeLists.txt:9-10` lists a `Connect.cpp` that was deleted.
Both are recorded as findings B1 and B2 in `docs/codebase_cleanup_audit.md:389-390`,
and `docs/architecture.md:21-24` says to treat the top-level build as incomplete
until an explicit task resolves it. Putting new code there would make it
unbuildable on arrival.

Both active projects — `Christian_control/basic_control` and
`Christian_control/planner_bridge` — are **standalone CMake projects** that
deliberately do not depend on the root build, naming the sources they need by
path. The proposal is to follow that precedent exactly:
**`Christian_control/vicon_monitor/`**, its own `CMakeLists.txt`, linking the
vendored Vicon SDK, Eigen and (for phase 3 only) Pinocchio. **Not Kortex.**

That last point discharges your constraint directly and mechanically: the monitor
binary cannot command the robot because `libKortexApiCpp.a` is not on its link
line. This is checkable in one grep rather than by reading the source, and it is
the same argument `probe_direction`, `expand_run_poses` and `print_dual_arm_fk`
already rely on.

---

# 4. General structure tracking

## 4.1 What the tracker is for

Given a snapshot and a named structure, produce that structure's pose in
`vicon_world`, together with an honest account of how it was obtained and how
much to trust it. The output type is the same regardless of which source
succeeded — that uniformity is the whole point of the abstraction, and it is what
lets a caller degrade gracefully instead of branching on source.

## 4.2 Source ladder

1. **Nexus segment pose** — preferred. Nexus has the subject's full marker
   template, solves it against all visible markers, and handles partial occlusion
   properly. It is strictly better than anything we would write, when available.
2. **Marker-template rigid registration** — fallback. Least-squares fit of a
   known local marker template onto the visible subset, by SVD with reflection
   correction (the Kabsch construction, in the shape of 907's dead
   `compute_rigid_transform`). Needs at least three non-collinear visible
   markers, and works with any subset above that.
3. **Ring fit** — *optional and specialised*. The circumcentre-plus-normal
   construction adapted from `calculateFramePose`, applicable **only** to
   structures that genuinely are circular marker rings, which for us means the
   two Kinova base rings.

The ordering is deliberate and the third rung is deliberately narrow. Circle
fitting is not the general abstraction: it applies to circles. Making it the
general case would be fitting the abstraction to one accident of our current
marker layout, and it would silently produce a plausible-looking pose for a
structure that is not a ring.

## 4.3 The result type

```
StructurePose
  structure_name        string
  world_T_structure     Eigen::Isometry3d          in vicon_world, metres
  source                {kNexusSegment, kMarkerTemplate, kRingFit, kNone}
  frame_number          uint32                     which snapshot produced it
  host_time_s           double
  valid                 bool                       usable at all
  markers_visible       int
  markers_expected      int
  fit_residual_m        double                     RMS; 0 for segment source
  worst_marker_m        double                     largest single residual
  condition_number      double                     geometric degeneracy measure
  note                  string                     why, when not valid
```

Two design choices worth defending in the report. The residual and condition
number are reported **always**, not only on failure, so a caller can watch quality
degrade continuously rather than discover a cliff. And `valid == false` still
carries whatever pose was computed, plus the reason — so a monitor can show "this
is the best fit available and here is why it is poor" rather than showing nothing.
That follows the project's standing preference for a graded measure over a veto,
and at monitoring time nothing is moving, so a refusal would protect nothing and
would cost the information.

## 4.4 The torso-attachment experiment

This is how §2.3's unresolved question gets answered, and it needs only a
recording — no robot motion, no hardware command path.

Record a session in which the arms are **stationary** and the wearer moves:
lean, twist, breathe, take a step. For every frame compute

```
^ref T_leftbase = (^world T_ref)⁻¹ · ^world T_leftbase
```

and plot the translation norm and rotation angle of that transform over time
against the wearer's motion.

- If the transform is **constant to within noise** — my expectation is a few
  millimetres and a few tenths of a degree — the reference markers are on the
  rigid backpack plate, `^torso_body T_backpack_mount` is a genuine fixed
  transform, and the existing latch-and-derive scheme is sound.
- If it **varies systematically with posture**, the markers are on the body. The
  latch is then invalid, the derived arm poses are wrong whenever the wearer
  moves, and the reference body must be demoted from "the frame we derive the
  arms from" to "the wearer's pose, tracked separately".

Stating the interpretation of both outcomes before running it is the point: this
is a genuine discriminating test, not a confirmation. It is also directly
report-ready — it is a measurement with a predicted result, an alternative
hypothesis, and a decision that follows from it.

---

# 5. Phased implementation plan

Every phase lands on its own branch in its own worktree, is reviewed
independently, and leaves the tree green. No phase modifies `ViconInterface.h`,
`ViconInterface.cpp`, `ViconInfo.h`, `ViconInfo.cpp` or `main.cpp`.

## Phase 0 — audit, frame contract, behavioural baseline

*Hardware*: none. *Files*: documentation only — this document, plus a decision
record at `Christian_control/docs/decisions/vicon-frame-contract.md`.

*Behaviour added*: none. The deliverable is the written contract of §2 and an
agreed marker-and-subject naming table for the Vicon session, recorded before
anyone stands in the lab.

*Tests*: none — nothing executes.

*Acceptance*: every frame in §2.2 has a name, an owner and a stated source of
truth; every transform in §2.3 is marked known-exact, known-weak or unknown; and
the marker naming table is agreed with whoever labels the Nexus subjects.

*Excluded*: any code; any change to the existing Vicon stack; resolving the
`msc_project` versus URDF mounting disagreement (that belongs to the RobotIo
work).

## Phase 1 — acquisition, snapshot, recorder, replay

*Hardware*: **Vicon only**, and only to capture. No robot, no Kortex on the link
line. One lab session produces the recordings that phases 2–4 develop against.

*Files*: new project `Christian_control/vicon_monitor/` with `CMakeLists.txt`;
`src/ViconSnapshot.h`, `src/ViconSource.h`, `src/ViconLiveSource.{h,cpp}`,
`src/ViconRecorder.{h,cpp}`, `src/ViconReplaySource.{h,cpp}`; a `record_vicon`
executable.

*Behaviour added*: connect with a **bounded** wait that reports elapsed time and
reason on giving up; explicit `SetStreamMode`; `EnableMarkerData`,
`EnableUnlabeledMarkerData` and — new — `EnableSegmentData`, with every enable
result checked; one snapshot per frame carrying frame number, host time, frame
rate, total latency and the latency breakdown; millimetre-to-metre conversion in
exactly one named function; three-valued marker status; record to disk and replay
from it.

*Recording format*: follow the house run-log convention rather than 907's. A
per-run directory under `runs/YYYY-MM-DD/vicon_<timestamp>/` containing
`meta.txt` (a `#`-prefixed preamble with `vicon_format = 1`, host, SDK version,
stream mode, frame-rate, and the index-to-name tables for markers and segments)
and two CSVs: `frames.csv`, one row per frame with the scalars, and
`entities.csv`, one row per entity per frame in long format so variable-length
data needs no schema change. `vicon_format` increments like `log_format` does,
and the same rule applies: parsers read by column name, never by index.

*Tests* (all hardware-free, bare `main()` per house convention, registered as
`add_test(NAME <subject> COMMAND test_<subject>)`):
`test_vicon_snapshot` — construction, name lookup, three-valued status, unit
conversion; `test_vicon_record_replay` — a synthetic snapshot sequence survives a
write/read round-trip **bit-identically** on every field; `test_vicon_replay_edge`
— truncated file, unknown `vicon_format`, missing column, empty recording, all
degrade with a stated reason rather than crashing.

*Acceptance*: a recording made in the lab replays offline and yields snapshots
byte-identical to those captured live; the recorded frame rate matches
`GetFrameRate()`; latency is present and non-zero; the connect path reports
failure within a bounded time when Nexus is not streaming; `main` and
`test_vicon` are untouched and still compile as before.

*Excluded*: geometry, tracking, comparison, display, force plates, any Kortex
linkage.

## Phase 2 — canonical structure tracking

*Hardware*: none — developed entirely against phase 1 recordings.

*Files*: `src/FrameContract.h`, `src/MarkerTemplate.{h,cpp}`,
`src/RingFit.{h,cpp}`, `src/StructureTracker.{h,cpp}`.

*Behaviour added*: the source ladder and `StructurePose` of §4; marker templates
loaded from configuration rather than hardcoded, replacing the roughly forty
hardcoded marker-name literals in `ViconInfo.cpp:371-409`; residual, condition
number and visible-marker count on every result.

*Tests*: `test_marker_template` — exact recovery of a known transform from
synthetic markers; correct behaviour under a deliberately mirrored point set
(this is what the reflection correction is for); graceful degradation with three,
two and zero visible markers; residual grows monotonically with injected noise.
`test_ring_fit` — recovers a known ring pose; flags near-collinear markers as
degenerate **instead of** silently returning a centroid with meaningless
orientation, which is the current `ViconInfo.cpp:1155-1157` behaviour.
`test_structure_tracker` — falls back down the ladder in the right order as
sources are removed; the source field always matches what was actually used.

*Acceptance*: on a phase 1 recording, both arm base rings track continuously; the
fallback fires and is correctly reported when markers are occluded; the
torso-attachment experiment of §4.4 runs from a recording and produces a plot and
a stated conclusion.

*Excluded*: URDF comparison, filtering, any use of the tracked pose to command
anything.

## Phase 3 — URDF geometry comparison

*Hardware*: none — recordings plus the URDF.

*Files*: `src/UrdfCompare.{h,cpp}`; Pinocchio enters the build here and only
here.

*Behaviour added*: compare the Vicon-measured relationship between the two arm
bases against the URDF's `^mount T_right_base_link` and `^mount T_left_base_link`;
report separation and tilt discrepancy in millimetres and degrees; optionally,
given joint angles, compare a Vicon-tracked tool marker frame against Pinocchio
FK.

*Tests*: `test_urdf_compare` — a synthetic "perfect" Vicon observation
constructed from the URDF itself yields zero discrepancy (this is the test that
proves the comparison is measuring the rig and not our own arithmetic); a
deliberately perturbed observation yields exactly the injected perturbation.

*Acceptance*: the comparison runs on a recording and reports numbers with stated
uncertainty. **Note explicitly**: a non-zero discrepancy is a *finding*, not a
failure — `dual_arm_mounting.yaml:11-17` records that the URDF numbers are
inherited rather than surveyed, so this phase is quite likely to be the first
real measurement of the rig. It must report, not judge.

*Excluded*: changing the URDF; resolving the `msc_project` disagreement; any
automatic correction.

## Phase 4 — terminal monitor and MATLAB visualisation

*Hardware*: optional — runs on live Vicon or on a recording, identically.

*Files*: `src/VicalMonitorMain.cpp`; `scripts/plot_vicon.m` alongside the
existing `show_frames.m` and `show_dual_arm_frames.m`.

*Behaviour added*: a terminal display refreshing at **10 Hz**, showing connection
state, frame number, frame rate, latency, per-structure pose, source, visible
marker count and residual, and the phase 3 discrepancy.

**Acquisition and recording run in their own thread at the Vicon frame rate; the
display samples the most recent snapshot at 10 Hz.** The display never gates
acquisition and never drops a recorded frame — the recorder consumes every
snapshot, the display consumes the latest. This is the same producer/consumer
split the run-log writer already uses (`Hardware.h:369-370`: the loop is the only
producer, the writer thread the only consumer).

*Tests*: `test_monitor_format` — rendering is a pure function from a snapshot
plus tracker results to a string, so the layout is testable with no terminal;
`test_monitor_decimation` — a 100 Hz snapshot stream drives exactly 10 renders
per second and zero dropped recorder frames.

*Acceptance*: the monitor runs against a recording with no Vicon present; `ldd`
on the binary shows no Kortex; a reviewer can confirm the absence of a command
path from the link line alone.

*Excluded*: any control, any actuation, any robot connection, any 3D rendering.

## Phase 5 — force plates (optional)

*Hardware*: Vicon with plates.

*Files*: force-plate fields in `ViconSnapshot.h`; reading in `ViconLiveSource`;
recorder and replay extension; `vicon_format` increments to 2.

*Behaviour added*: force, moment and centre of pressure with **explicit unit
conversion** — newtons stay newtons, newton-millimetres become newton-metres,
millimetre CoP becomes metres — plus plate count checked before indexing, and
subsample handling made explicit rather than implicitly dropping ~90% of the data
as both projects currently do.

*Tests*: `test_force_units` — conversions pin the documented SDK units;
`test_force_plate_absent` — zero plates degrades cleanly.

*Acceptance*: plate data round-trips through record and replay; units are stated
in the file preamble; a `vicon_format = 1` recording still replays.

*Excluded*: any use of force in a control decision.

## Phase 6 — tool registration and filtering experiments

*Hardware*: Vicon plus, for tool registration only, robot joint angles — which
may be **read from an existing run log rather than live**, keeping this phase
hardware-command-free.

*Files*: `src/ToolRegistration.{h,cpp}`, `src/PoseFilter.{h,cpp}`.

*Behaviour added*: estimate `^tool_marker_frame T_right_tool` properly — this is
a hand-eye (AX = XB) problem, not point-set Kabsch, and 907's conflation of the
two is documented in §1.2 item 10 as the thing not to repeat. Separately, an
SE(3) pose filter with a velocity state, `dt` taken from the measured frame
interval rather than assumed, innovation gating for outlier rejection, and the
Joseph covariance form.

*Tests*: `test_hand_eye` — exact recovery on synthetic data with a known answer,
including the degenerate near-collinear case that must be *detected* rather than
smoothed over; `test_pose_filter` — convergence, correct behaviour across a
dropped frame, innovation gate rejects an injected outlier.

*Acceptance*: both run on recordings and report their own quality; the filter is
compared against unfiltered data on the same recording so the benefit is
measured, not assumed.

*Excluded*: putting either in a control path.

---

# 6. Workflow

**Plan mode per phase.** Each phase starts in plan mode with its own short
design note in `docs/superpowers/plans/`, following the house pattern of a plan
sharing its spec's slug. The note states the phase's files, its tests and its
acceptance criteria before any code is written, and is approved before exiting
plan mode. This document is phase 0's.

**Read-only research agents, investigation only.** Exactly as used to produce
this document: `Explore`-type agents, which are tool-enforced read-only — no
`Edit`, no `Write`. They are dispatched in parallel when the questions are
independent, and their findings are synthesised by one writer. Project 907 is
only ever read this way. **Only one agent — or one session — ever writes to
`HumanSL_MAIN`.**

**Isolated worktrees.** One branch and one worktree per phase, e.g.
`vicon/phase1-snapshot`. `.gitignore` already excludes agent worktree checkouts
(commit `98d3f858`), so this is established practice. The worktree keeps a
half-finished phase from ever being what is on disk when the robot is used for
something else.

**Independent reviewer per phase.** A reviewer that did not write the code, given
the phase's plan note and its diff, and asked specifically: does the code do what
the plan said; are the units right at every boundary; is any frame assumed
identity that was not measured; and — the standing check for every phase —
**does anything link Kortex**. Per the project's light-process preference,
phases 1, 5 and 6 get one review pass; phases 2 and 3 get an adversarial review
because they are where a wrong transform would look plausible.

**The hardware rule, restated concretely.** No target in
`Christian_control/vicon_monitor/` links `libKortexApiCpp.a` in any phase of this
plan. The monitor reads Vicon and the URDF and nothing else. Because the
constraint is expressed as a link-line fact rather than a code-review intention,
it can be verified mechanically by a reviewer and by `ldd`.

---

# 7. Unresolved assumptions

1. **Are the `human_ref_base1..4` markers on the rigid backpack plate or on the
   wearer's body?** Not determinable from code; the name is ambiguous and the
   code assumes rigid. §4.4 gives the experiment. Everything about
   `^torso_body T_backpack_mount` depends on the answer.
2. **Is `backpack_mount` the same frame as the URDF's `mount`?** `mount` is a
   derived midpoint construction, not a physical feature. Whether we define them
   to coincide is a decision.
3. **What is the Vicon host's axis convention?** `SetAxisMapping` is never called
   in either project, so the convention is whatever Nexus is configured for and
   is nowhere asserted in code. Existing code silently depends on it —
   `ViconInfo.cpp:157-159` assumes the tube runs along world +Y,
   `ViconInfo.cpp:1266` assumes Z-up. This should be set explicitly and asserted.
4. **Do labelled Nexus subjects and segments exist for the rig at all?** Segment
   data has never been enabled, so nobody has checked. If Nexus has no solved
   segments for our structures, the preferred source in §4.2 is unavailable and
   the marker-template fallback becomes the primary path. This is a five-minute
   check in the lab that changes phase 2's emphasis.
5. **Where does the 0.133 m ring offset come from?** No provenance in the
   repository. It should be measured or re-derived, and the marker-1 convention
   that sets the ring's rotation about its normal should be written down.
6. **Are the URDF mounting numbers right?** They are inherited, not surveyed
   (`dual_arm_mounting.yaml:11-17`), and `msc_project` disagrees. Phase 3 is
   likely to be the first real measurement, and its result feeds the separate
   RobotIo/sim-unification decision rather than being settled here.
7. **Which arm's tool?** The right arm has `ConfiguredTool_Link`; the left ends
   at a bare flange. Tool-marker registration in phase 6 is right-arm-only unless
   a tool is added to the left.
