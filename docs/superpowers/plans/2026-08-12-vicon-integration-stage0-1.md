# Vicon integration — Stage 0 & Stage 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

Status: Tasks 1–6 implemented, reviewed, and committed 2026-08-12
(185161a0..809ad94a, subagent-driven-development, one fix round each on
Tasks 4/5's plan corrections plus one final-review fix wave — see the
ledger at `.superpowers/sdd/2026-08-12-vicon-integration-stage0-1/progress.md`
for the full task-by-task record, since that directory is git-ignored and
this status line is the durable summary). Task 7's Steps 1–2 (the
decision-record template) are committed; Step 3 (the actual lab session)
is Christian's own physical action and remains outstanding — the
individual checkboxes below are left unchecked for that reason, not
because the plan's automated portion is incomplete.

**Goal:** Build the minimal Vicon snapshot/recorder/replay layer (Stage 1)
and the tool it takes to run the lab verification and capture the two
recordings Stage 0 needs — so everything after this plan (calibration, the
controller's world-frame hold, the planner's plan-time frames) develops
offline.

**Architecture:** A pure, Eigen-typed `ViconSnapshot` value (metres,
checked quaternions) built once per frame from `ViconInterface`'s raw
millimetre reads by a pure `BuildSnapshot` function; a `ViconRecorder` that
writes it to the house two-CSV run-log convention; a `ViconReplaySource`
that reads the same files back. A new `record_vicon` tool wires
`ViconInterface` + `BuildSnapshot` + `ViconRecorder` into a live capture
CLI. Everything except `record_vicon` and the `getLatencyTotal()` SDK call
is hardware-free and unit-tested without a Vicon connection.

**Tech Stack:** C++17, CMake/CTest, the vendored Vicon DataStream SDK
(`third_party/vicon_api`), the vendored header-only Eigen
(`third_party/include/eigen3`). No new external dependencies.

## Global Constraints

- No target in `Christian_control/vicon/` links Kortex or any robot-facing
  library — sensing-only, verifiable from the link line alone.
- New Vicon code is Eigen-only (`Eigen::Vector3d` / `Eigen::Quaterniond`),
  matching `basic_control`'s `<Eigen/Dense>` + `<Eigen/Geometry>`
  convention, so it can be linked from either side of the GTSAM/Pinocchio
  divide later.
- Millimetre-to-metre conversion happens exactly once, at the SDK
  boundary (`BuildSnapshot`). Nothing downstream sees millimetres.
- Quaternions are accepted only when finite and within tolerance of unit
  norm; never silently normalised.
- No blocking Vicon I/O may appear anywhere the controller's 500 Hz loop
  could later call it — not relevant to this plan's files directly, but
  every acquisition call stays outside any hot path by construction (no
  hot path exists yet in `Christian_control/vicon/`).
- No robot-facing binary is built or run by this plan; `record_vicon` and
  `connect_vicon` are sensing-only and still require Christian's presence
  to run against the live lab server.
- `-Wall -Wextra` stays clean on first-party code; vendor headers
  (Vicon SDK, Eigen) are included `SYSTEM`, matching the existing
  `vicon/CMakeLists.txt`.
- Every test executable added by this plan calls `enable_assertions_for`
  (added to `CMakeLists.txt` after Task 2). This project defaults to a
  Release build, whose `-DNDEBUG` silently strips `assert()` to a no-op —
  confirmed live during Task 2's review, when Task 1's already-"passing"
  test turned out to verify nothing at runtime. Without this call, a new
  test target would build, run, and report PASS regardless of whether the
  code under test is correct.
- Follow the house run-log convention already used in
  `basic_control/src/Main.cpp` / `Hardware.cpp`: a `#`-prefixed preamble
  (parsers skip `#` lines), `key = value` pairs, a `vicon_format` version
  number, and column headers read by name.

---

## File structure

```
Christian_control/vicon/
  src/
    ViconInterface.h/.cpp        MODIFY — add getLatencyTotal()
    ViconSnapshot.h              NEW — data contract (structs only)
    SnapshotBuilder.h/.cpp       NEW — mm→m + validation, pure function
    ViconRecorder.h/.cpp         NEW — writes frames.csv + entities.csv
    ViconReplaySource.h/.cpp     NEW — reads them back
  tools/
    record_vicon.cpp             NEW — live capture CLI (for Stage 0)
  tests/
    test_segment_data.cpp        MODIFY — add getLatencyTotal assertion
    test_vicon_snapshot.cpp      NEW — data contract shape
    test_snapshot_builder.cpp    NEW — conversion + validation cases
    test_vicon_recorder.cpp      NEW — file content
    test_vicon_replay.cpp        NEW — round trip + degradation
  CMakeLists.txt                 MODIFY — new library, tool, test targets

Christian_control/docs/decisions/
  vicon-frame-contract.md        NEW — Stage 0 lab checklist + decision record
```

`ViconInterface`, `SegmentData`, `MarkerData` are unchanged except for the
new `getLatencyTotal()` getter — every other method and existing test
keeps passing untouched.

---

### Task 1: `ViconSnapshot` data contract

**Files:**
- Create: `Christian_control/vicon/src/ViconSnapshot.h`
- Test: `Christian_control/vicon/tests/test_vicon_snapshot.cpp`
- Modify: `Christian_control/vicon/CMakeLists.txt`

**Interfaces:**
- Produces: `MarkerSample { name, position_m (Eigen::Vector3d), valid (bool), invalid_reason (string) }`; `SegmentSample { subject_name, segment_name, position_m, orientation (Eigen::Quaterniond), valid, invalid_reason }`; `ViconSnapshot { frame_number (unsigned int), host_time_s, frame_rate_hz, latency_total_s (double), markers (vector<MarkerSample>), segments (vector<SegmentSample>) }`. Every later task in this plan constructs, reads, or serialises exactly these three types with these field names.

- [ ] **Step 1: Write the failing test**

Create `Christian_control/vicon/tests/test_vicon_snapshot.cpp`:

```cpp
#include "ViconSnapshot.h"

#include <cassert>

int main() {
    MarkerSample marker;
    assert(marker.name.empty());
    assert(marker.position_m == Eigen::Vector3d::Zero());
    assert(!marker.valid);
    assert(marker.invalid_reason.empty());

    SegmentSample segment;
    assert(segment.subject_name.empty());
    assert(segment.segment_name.empty());
    assert(segment.position_m == Eigen::Vector3d::Zero());
    assert(segment.orientation.coeffs() == Eigen::Quaterniond::Identity().coeffs());
    assert(!segment.valid);
    assert(segment.invalid_reason.empty());

    ViconSnapshot snapshot;
    assert(snapshot.frame_number == 0);
    assert(snapshot.host_time_s == 0.0);
    assert(snapshot.frame_rate_hz == 0.0);
    assert(snapshot.latency_total_s == 0.0);
    assert(snapshot.markers.empty());
    assert(snapshot.segments.empty());
    return 0;
}
```

- [ ] **Step 2: Wire the CMake target before the header exists, to see it fail**

Edit `Christian_control/vicon/CMakeLists.txt`. Insert this function right
after the `project(vicon CXX)` / compiler-options block (after the
`set(CMAKE_EXPORT_COMPILE_COMMANDS ON)` line, before
`add_library(vicon_interface STATIC ...)`):

```cmake
# The bundled Eigen, as a SYSTEM include so -Wall stays quiet inside it.
# Mirrors basic_control/CMakeLists.txt's add_bundled_eigen_to.
function(add_bundled_eigen_to target)
    target_include_directories(${target} SYSTEM PRIVATE
            ${HUMANSL_ROOT}/third_party/include/eigen3
    )
endfunction()
```

Inside the existing `if (BUILD_TESTING)` block, after the
`test_segment_data` registration, add:

```cmake
    add_executable(test_vicon_snapshot tests/test_vicon_snapshot.cpp)
    target_include_directories(test_vicon_snapshot PRIVATE src)
    add_bundled_eigen_to(test_vicon_snapshot)
    add_test(NAME vicon_snapshot COMMAND test_vicon_snapshot)
```

- [ ] **Step 3: Run to confirm it fails**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_vicon_snapshot`
Expected: FAIL — `ViconSnapshot.h` does not exist yet (`fatal error: ViconSnapshot.h: No such file or directory`).

- [ ] **Step 4: Write the header**

Create `Christian_control/vicon/src/ViconSnapshot.h`:

```cpp
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <vector>

// One labelled marker in a validated Vicon frame, position in metres,
// Vicon-world frame. valid=false means position_m must not be used;
// invalid_reason says why (e.g. "occluded").
struct MarkerSample {
    std::string name;
    Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
    bool valid = false;
    std::string invalid_reason;
};

// One subject/segment pose in a validated Vicon frame, Vicon-world frame,
// metres and a checked quaternion. valid=false means position_m and
// orientation must not be used.
struct SegmentSample {
    std::string subject_name;
    std::string segment_name;
    Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool valid = false;
    std::string invalid_reason;
};

// One validated Vicon frame: every marker and segment the SDK returned,
// converted to metres, with quaternions checked (finite, unit-norm)
// rather than silently normalised. No SDK types, no I/O — a plain value.
struct ViconSnapshot {
    unsigned int frame_number = 0;
    double host_time_s = 0.0;      // steady_clock at the read, seconds
    double frame_rate_hz = 0.0;    // SDK-reported server rate
    double latency_total_s = 0.0;  // SDK-reported total latency
    std::vector<MarkerSample> markers;
    std::vector<SegmentSample> segments;
};
```

- [ ] **Step 5: Run to confirm it passes**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_vicon_snapshot && /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build/test_vicon_snapshot`
Expected: builds and exits 0.

- [ ] **Step 6: Commit**

```bash
git add Christian_control/vicon/src/ViconSnapshot.h Christian_control/vicon/tests/test_vicon_snapshot.cpp Christian_control/vicon/CMakeLists.txt
git commit -m "vicon: add ViconSnapshot data contract"
```

---

### Task 2: `ViconInterface::getLatencyTotal()`

**Files:**
- Modify: `Christian_control/vicon/src/ViconInterface.h`
- Modify: `Christian_control/vicon/src/ViconInterface.cpp`
- Modify: `Christian_control/vicon/tests/test_segment_data.cpp`

**Interfaces:**
- Consumes: `ViconInterface` (existing class), `client_->GetLatencyTotal()` (SDK method returning `Output_GetLatencyTotal { Result::Enum Result; double Total; }`, declared in `third_party/vicon_api/include/IDataStreamClientBase.h:437-442`).
- Produces: `double ViconInterface::getLatencyTotal() const` — 0.0 when not connected or the SDK could not report it, otherwise the SDK's `Total` (seconds). Task 6 (`record_vicon`) calls this every frame.

- [ ] **Step 1: Write the failing test**

Edit `Christian_control/vicon/tests/test_segment_data.cpp`, adding one line
after the existing `assert(vicon.getSegmentPoses().empty());`:

```cpp
    assert(vicon.getSegmentPoses().empty());
    assert(vicon.getLatencyTotal() == 0.0);
    return 0;
```

(Replaces the old `assert(vicon.getSegmentPoses().empty()); return 0;`
pair with the three lines above.)

- [ ] **Step 2: Run to confirm it fails**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_segment_data`
Expected: FAIL to compile — `'class ViconInterface' has no member named 'getLatencyTotal'`.

- [ ] **Step 3: Implement**

Edit `Christian_control/vicon/src/ViconInterface.h`. After the existing:

```cpp
    // Camera system frame rate in Hz, as reported by the server.
    double getFrameRate() const;
```

add:

```cpp

    // Cumulative Vicon-reported latency for the current frame, in
    // seconds (sum of the SDK's own latency breakdown). 0.0 when not
    // connected or the SDK could not report it.
    double getLatencyTotal() const;
```

Edit `Christian_control/vicon/src/ViconInterface.cpp`. After the existing
`ViconInterface::getFrameRate()` definition (ends just before
`MarkerData ViconInterface::getMarkerPosition(...)`), add:

```cpp

double ViconInterface::getLatencyTotal() const {
    if (!connected_) return 0.0;

    Output_GetLatencyTotal latencyOutput = client_->GetLatencyTotal();
    if (latencyOutput.Result == Result::Success) {
        return latencyOutput.Total;
    }

    return 0.0;
}
```

- [ ] **Step 4: Run to confirm it passes**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_segment_data && /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build/test_segment_data`
Expected: builds and exits 0.

- [ ] **Step 5: Commit**

```bash
git add Christian_control/vicon/src/ViconInterface.h Christian_control/vicon/src/ViconInterface.cpp Christian_control/vicon/tests/test_segment_data.cpp
git commit -m "vicon: add ViconInterface::getLatencyTotal"
```

---

### Task 3: `SnapshotBuilder` — mm→m conversion and validation

**Files:**
- Create: `Christian_control/vicon/src/SnapshotBuilder.h`
- Create: `Christian_control/vicon/src/SnapshotBuilder.cpp`
- Test: `Christian_control/vicon/tests/test_snapshot_builder.cpp`
- Modify: `Christian_control/vicon/CMakeLists.txt`

**Interfaces:**
- Consumes: `MarkerData`, `SegmentData` (existing, `ViconInterface.h`); `MarkerSample`, `SegmentSample`, `ViconSnapshot` (Task 1).
- Produces: `ViconSnapshot BuildSnapshot(unsigned int frame_number, double host_time_s, double frame_rate_hz, double latency_total_s, const std::vector<MarkerData>& markers, const std::vector<SegmentData>& segments)`. Task 4 and 6 call this directly; Task 5's round-trip test uses it to build fixtures.

- [ ] **Step 1: Write the failing test**

Create `Christian_control/vicon/tests/test_snapshot_builder.cpp`:

```cpp
#include "SnapshotBuilder.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace {

MarkerData MakeMarker(const std::string& name, double x, double y, double z,
                       bool occluded) {
    MarkerData m;
    m.name = name;
    m.x = x;
    m.y = y;
    m.z = z;
    m.occluded = occluded;
    return m;
}

SegmentData MakeSegment(const std::string& subject, const std::string& segment,
                         double x, double y, double z, double qx, double qy,
                         double qz, double qw, bool occluded) {
    SegmentData s;
    s.subject_name = subject;
    s.segment_name = segment;
    s.x = x;
    s.y = y;
    s.z = z;
    s.qx = qx;
    s.qy = qy;
    s.qz = qz;
    s.qw = qw;
    s.occluded = occluded;
    return s;
}

}  // namespace

int main() {
    // Frame metadata passes through unchanged.
    {
        const auto snapshot = BuildSnapshot(42, 1.5, 100.0, 0.02, {}, {});
        assert(snapshot.frame_number == 42);
        assert(snapshot.host_time_s == 1.5);
        assert(snapshot.frame_rate_hz == 100.0);
        assert(snapshot.latency_total_s == 0.02);
        assert(snapshot.markers.empty());
        assert(snapshot.segments.empty());
    }

    // Valid marker: millimetres convert to metres exactly.
    {
        const std::vector<MarkerData> markers = {
            MakeMarker("M1", 1000.0, 2000.0, 3000.0, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, markers, {});
        assert(snapshot.markers.size() == 1);
        const auto& m = snapshot.markers[0];
        assert(m.name == "M1");
        assert(m.valid);
        assert(m.invalid_reason.empty());
        assert(std::abs(m.position_m.x() - 1.0) < 1e-12);
        assert(std::abs(m.position_m.y() - 2.0) < 1e-12);
        assert(std::abs(m.position_m.z() - 3.0) < 1e-12);
    }

    // Occluded marker: invalid, reason stated.
    {
        const std::vector<MarkerData> markers = {
            MakeMarker("M2", 0.0, 0.0, 0.0, true)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, markers, {});
        assert(!snapshot.markers[0].valid);
        assert(snapshot.markers[0].invalid_reason == "occluded");
    }

    // Valid segment: unit quaternion, millimetres convert to metres.
    {
        const std::vector<SegmentData> segments = {MakeSegment(
            "Dr Octopus Christian", "Mount", 100.0, 200.0, 300.0, 0.0, 0.0,
            0.0, 1.0, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(snapshot.segments.size() == 1);
        const auto& s = snapshot.segments[0];
        assert(s.subject_name == "Dr Octopus Christian");
        assert(s.segment_name == "Mount");
        assert(s.valid);
        assert(s.invalid_reason.empty());
        assert(std::abs(s.position_m.x() - 0.1) < 1e-12);
        assert(std::abs(s.position_m.y() - 0.2) < 1e-12);
        assert(std::abs(s.position_m.z() - 0.3) < 1e-12);
        assert(std::abs(s.orientation.w() - 1.0) < 1e-12);
    }

    // Occluded segment: invalid, reason "occluded" (checked before the
    // quaternion, since occlusion is the SDK's own definitive signal).
    {
        const std::vector<SegmentData> segments = {
            MakeSegment("S", "Seg", 0, 0, 0, 0, 0, 0, 1, true)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(!snapshot.segments[0].valid);
        assert(snapshot.segments[0].invalid_reason == "occluded");
    }

    // Non-finite quaternion: invalid, reason mentions "finite".
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const std::vector<SegmentData> segments = {
            MakeSegment("S", "Seg", 0, 0, 0, nan, 0, 0, 1, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(!snapshot.segments[0].valid);
        assert(snapshot.segments[0].invalid_reason == "non-finite quaternion");
    }

    // Non-unit-norm quaternion: invalid, reason mentions "unit-norm".
    {
        const std::vector<SegmentData> segments = {
            MakeSegment("S", "Seg", 0, 0, 0, 2.0, 0, 0, 0, false)};
        const auto snapshot = BuildSnapshot(0, 0.0, 0.0, 0.0, {}, segments);
        assert(!snapshot.segments[0].valid);
        assert(snapshot.segments[0].invalid_reason == "quaternion not unit-norm");
    }

    return 0;
}
```

- [ ] **Step 2: Wire the CMake target, run to confirm it fails**

Edit `Christian_control/vicon/CMakeLists.txt`. Insert a new library
target after the existing `vicon_interface` library block (after its
`set_target_properties(vicon_interface PROPERTIES BUILD_RPATH ...)`
closing `)`, before the `add_executable(connect_vicon ...)` block):

```cmake

add_library(vicon_snapshot STATIC
        src/SnapshotBuilder.cpp
)
target_include_directories(vicon_snapshot PUBLIC src)
target_include_directories(vicon_snapshot SYSTEM PUBLIC
        ${HUMANSL_ROOT}/third_party/include/eigen3
)
target_link_libraries(vicon_snapshot PUBLIC vicon_interface)
```

Inside `if (BUILD_TESTING)`, after the `test_vicon_snapshot` registration
added in Task 1, add:

```cmake
    add_executable(test_snapshot_builder tests/test_snapshot_builder.cpp)
    target_link_libraries(test_snapshot_builder PRIVATE vicon_snapshot)
    enable_assertions_for(test_snapshot_builder)
    add_test(NAME vicon_snapshot_builder COMMAND test_snapshot_builder)
```

`enable_assertions_for` was added to `CMakeLists.txt` after Task 2, fixing a
build defect found during Task 2's review: this project defaults to a
Release build, whose `-DNDEBUG` silently compiles every `assert()` to a
no-op — so without this call, `test_snapshot_builder`'s assertions below
would build, run, and report PASS regardless of whether `BuildSnapshot` is
correct. Call it for every new test executable added in this plan.

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_snapshot_builder`
Expected: FAIL — `SnapshotBuilder.h` does not exist yet.

- [ ] **Step 3: Write the header**

Create `Christian_control/vicon/src/SnapshotBuilder.h`:

```cpp
#pragma once

#include "ViconInterface.h"
#include "ViconSnapshot.h"

#include <vector>

// Converts one frame's raw SDK reads (ViconInterface's millimetre,
// occluded-bool convention) into a validated ViconSnapshot: metres, and
// quaternions accepted only when finite and within tolerance of unit
// norm -- never silently normalised. Pure function: no SDK calls, no I/O,
// no state.
ViconSnapshot BuildSnapshot(unsigned int frame_number, double host_time_s,
                             double frame_rate_hz, double latency_total_s,
                             const std::vector<MarkerData>& markers,
                             const std::vector<SegmentData>& segments);
```

- [ ] **Step 4: Write the implementation**

Create `Christian_control/vicon/src/SnapshotBuilder.cpp`:

```cpp
#include "SnapshotBuilder.h"

#include <cmath>

namespace {

constexpr double kMmToM = 0.001;

// How far a quaternion's norm may sit from 1 and still be trusted as
// numerically valid. The SDK does not guarantee unit norm on every frame;
// tighter than this starts rejecting good data, looser risks treating
// garbage as a rotation.
constexpr double kQuaternionNormTolerance = 1e-3;

bool IsFiniteQuaternion(double qx, double qy, double qz, double qw) {
    return std::isfinite(qx) && std::isfinite(qy) && std::isfinite(qz) &&
           std::isfinite(qw);
}

}  // namespace

ViconSnapshot BuildSnapshot(unsigned int frame_number, double host_time_s,
                             double frame_rate_hz, double latency_total_s,
                             const std::vector<MarkerData>& markers,
                             const std::vector<SegmentData>& segments) {
    ViconSnapshot snapshot;
    snapshot.frame_number = frame_number;
    snapshot.host_time_s = host_time_s;
    snapshot.frame_rate_hz = frame_rate_hz;
    snapshot.latency_total_s = latency_total_s;

    snapshot.markers.reserve(markers.size());
    for (const auto& marker : markers) {
        MarkerSample sample;
        sample.name = marker.name;
        sample.position_m = Eigen::Vector3d(marker.x * kMmToM, marker.y * kMmToM,
                                             marker.z * kMmToM);
        if (marker.occluded) {
            sample.valid = false;
            sample.invalid_reason = "occluded";
        } else {
            sample.valid = true;
        }
        snapshot.markers.push_back(sample);
    }

    snapshot.segments.reserve(segments.size());
    for (const auto& segment : segments) {
        SegmentSample sample;
        sample.subject_name = segment.subject_name;
        sample.segment_name = segment.segment_name;
        sample.position_m = Eigen::Vector3d(segment.x * kMmToM, segment.y * kMmToM,
                                             segment.z * kMmToM);
        sample.orientation = Eigen::Quaterniond(segment.qw, segment.qx, segment.qy,
                                                 segment.qz);

        if (segment.occluded) {
            sample.valid = false;
            sample.invalid_reason = "occluded";
        } else if (!IsFiniteQuaternion(segment.qx, segment.qy, segment.qz,
                                        segment.qw)) {
            sample.valid = false;
            sample.invalid_reason = "non-finite quaternion";
        } else if (std::abs(sample.orientation.norm() - 1.0) >
                   kQuaternionNormTolerance) {
            sample.valid = false;
            sample.invalid_reason = "quaternion not unit-norm";
        } else {
            sample.valid = true;
        }
        snapshot.segments.push_back(sample);
    }

    return snapshot;
}
```

Note the check order in each branch: occlusion is tested first (the SDK's
own definitive signal), then finiteness, then norm. Finiteness must be
checked *before* the norm comparison — `std::abs(NaN - 1.0) > tolerance`
evaluates to `false` in IEEE 754 (any comparison against NaN is false), so
a NaN quaternion would silently fall through to `valid = true` if the norm
check ran first.

- [ ] **Step 5: Run to confirm it passes**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_snapshot_builder && /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build/test_snapshot_builder`
Expected: builds and exits 0.

- [ ] **Step 6: Commit**

```bash
git add Christian_control/vicon/src/SnapshotBuilder.h Christian_control/vicon/src/SnapshotBuilder.cpp Christian_control/vicon/tests/test_snapshot_builder.cpp Christian_control/vicon/CMakeLists.txt
git commit -m "vicon: add SnapshotBuilder (mm to m conversion, quaternion validation)"
```

---

### Task 4: `ViconRecorder` — write the two-CSV recording

**Files:**
- Create: `Christian_control/vicon/src/ViconRecorder.h`
- Create: `Christian_control/vicon/src/ViconRecorder.cpp`
- Test: `Christian_control/vicon/tests/test_vicon_recorder.cpp`
- Modify: `Christian_control/vicon/CMakeLists.txt`

**Interfaces:**
- Consumes: `ViconSnapshot`, `MarkerSample`, `SegmentSample` (Task 1).
- Produces: `class ViconRecorder { ViconRecorder(std::ostream& frames_csv, std::ostream& entities_csv, std::string host, std::string subject); void WriteHeader(); void Write(const ViconSnapshot&); }`. Task 5's round-trip test and Task 6's `record_vicon` tool both construct and drive this exact class.

File layout this task establishes (the house run-log convention extended
for a variable entity count):

`frames.csv` — one row per frame, `#`-prefixed preamble, then:
```
frame_number,host_time_s,frame_rate_hz,latency_total_s
```

`entities.csv` — one row per marker/segment per frame (long format, joined
to `frames.csv` by `frame_number`), then:
```
frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,invalid_reason
```
`kind` is `marker` or `segment`. Markers leave `qx,qy,qz,qw` blank (no
orientation). `valid` is `1`/`0`.

**Precision policy (decided by Christian, 2026-08-12, after Task 4's first
attempt):** every numeric field is written fixed-point to 4 decimal
places (`std::fixed << std::setprecision(4)`), not the exact-round-trip
`max_digits10` (17 significant digits) the design originally specified.
This is a deliberate, bounded precision loss — 4 decimal places is
0.1 mm for a metre-valued position, well inside Vicon's own measurement
noise, and far more readable than 17-digit doubles like
`0.070000000000000007`. Consequence for Task 5: round-trip equality
(`==`) holds only for values whose true precision doesn't exceed 4
decimal places — which is true for every fixture value used in this
plan's tests, but would NOT be true of an arbitrary real Vicon recording.
Record this bound in mind for any future consumer that needs finer
precision than 4 decimal places.

- [ ] **Step 1: Write the failing test**

Create `Christian_control/vicon/tests/test_vicon_recorder.cpp`:

```cpp
#include "SnapshotBuilder.h"
#include "ViconRecorder.h"

#include <cassert>
#include <sstream>

namespace {

MarkerData MakeMarker() {
    MarkerData m;
    m.name = "M1";
    m.x = 1000.0;
    m.y = 2000.0;
    m.z = 3000.0;
    m.occluded = false;
    return m;
}

SegmentData MakeSegment() {
    SegmentData s;
    s.subject_name = "Dr Octopus Christian";
    s.segment_name = "Mount";
    s.x = 10.0;
    s.y = 20.0;
    s.z = 30.0;
    s.qx = 0.0;
    s.qy = 0.0;
    s.qz = 0.0;
    s.qw = 1.0;
    s.occluded = false;
    return s;
}

}  // namespace

int main() {
    std::ostringstream frames_out, entities_out;
    ViconRecorder recorder(frames_out, entities_out, "192.168.128.206:801",
                            "Dr Octopus Christian");
    recorder.WriteHeader();

    const auto snapshot = BuildSnapshot(7, 0.07, 100.0, 0.015, {MakeMarker()},
                                         {MakeSegment()});
    recorder.Write(snapshot);

    const std::string frames_text = frames_out.str();
    assert(frames_text.find("# vicon_format = 1") != std::string::npos);
    assert(frames_text.find("frame_number,host_time_s,frame_rate_hz,latency_total_s") !=
           std::string::npos);
    assert(frames_text.find("7,0.0700,100.0000,0.0150") != std::string::npos);

    const std::string entities_text = entities_out.str();
    assert(entities_text.find(
               "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
               "invalid_reason") != std::string::npos);
    assert(entities_text.find("7,marker,,M1,1.0000,2.0000,3.0000,,,,,1,") !=
           std::string::npos);
    assert(entities_text.find(
               "7,segment,Dr Octopus Christian,Mount,0.0100,0.0200,0.0300") !=
           std::string::npos);

    return 0;
}
```

- [ ] **Step 2: Wire the CMake target, run to confirm it fails**

Edit `Christian_control/vicon/CMakeLists.txt`. Change the `vicon_snapshot`
library's source list (added in Task 3) to:

```cmake
add_library(vicon_snapshot STATIC
        src/SnapshotBuilder.cpp
        src/ViconRecorder.cpp
)
```

Inside `if (BUILD_TESTING)`, after the `test_snapshot_builder`
registration, add:

```cmake
    add_executable(test_vicon_recorder tests/test_vicon_recorder.cpp)
    target_link_libraries(test_vicon_recorder PRIVATE vicon_snapshot)
    enable_assertions_for(test_vicon_recorder)
    add_test(NAME vicon_recorder COMMAND test_vicon_recorder)
```

(`enable_assertions_for` — see the note in Task 3.)

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_vicon_recorder`
Expected: FAIL — `ViconRecorder.h` does not exist yet.

- [ ] **Step 3: Write the header**

Create `Christian_control/vicon/src/ViconRecorder.h`:

```cpp
#pragma once

#include "ViconSnapshot.h"

#include <ostream>
#include <string>

// Writes a stream of ViconSnapshots into the two-file house run-log
// layout: frames.csv holds one row per frame (frame-level scalars, with a
// '#'-prefixed preamble parsers skip -- see basic_control's run logs for
// the same convention); entities.csv holds one row per marker/segment per
// frame in long format, joined to frames.csv by frame_number, so a
// variable entity count needs no schema change.
//
// Construct with the two already-open output streams (caller owns them --
// this class never opens or closes a file). Call WriteHeader() once
// before the first Write().
class ViconRecorder {
public:
    ViconRecorder(std::ostream& frames_csv, std::ostream& entities_csv,
                  std::string host, std::string subject);

    void WriteHeader();
    void Write(const ViconSnapshot& snapshot);

private:
    std::ostream& frames_csv_;
    std::ostream& entities_csv_;
    std::string host_;
    std::string subject_;
};
```

- [ ] **Step 4: Write the implementation**

Create `Christian_control/vicon/src/ViconRecorder.cpp`:

```cpp
#include "ViconRecorder.h"

#include <iomanip>
#include <sstream>

namespace {

constexpr int kViconFormat = 1;
// Fixed-point, not exact round-trip: 4 decimal places is 0.1 mm for a
// metre-valued position, well inside Vicon's own measurement noise, and
// far more readable than a 17-significant-digit double. See the
// "Precision policy" note above Task 4 Step 1 for the full rationale.
constexpr int kDecimalPlaces = 4;

std::string QuaternionField(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(kDecimalPlaces) << value;
    return oss.str();
}

void WriteEntityRow(std::ostream& out, unsigned int frame_number,
                     const std::string& kind, const std::string& subject,
                     const std::string& name, double x, double y, double z,
                     const std::string& qx, const std::string& qy,
                     const std::string& qz, const std::string& qw, bool valid,
                     const std::string& invalid_reason) {
    out << frame_number << "," << kind << "," << subject << "," << name << ","
        << x << "," << y << "," << z << "," << qx << "," << qy << "," << qz
        << "," << qw << "," << (valid ? 1 : 0) << "," << invalid_reason << "\n";
}

}  // namespace

ViconRecorder::ViconRecorder(std::ostream& frames_csv, std::ostream& entities_csv,
                              std::string host, std::string subject)
    : frames_csv_(frames_csv), entities_csv_(entities_csv),
      host_(std::move(host)), subject_(std::move(subject)) {
    frames_csv_ << std::fixed << std::setprecision(kDecimalPlaces);
    entities_csv_ << std::fixed << std::setprecision(kDecimalPlaces);
}

void ViconRecorder::WriteHeader() {
    frames_csv_ << "# vicon controller/planner recording -- parsers skip '#' lines\n";
    frames_csv_ << "# vicon_format = " << kViconFormat << "\n";
    frames_csv_ << "# host = " << host_ << "\n";
    frames_csv_ << "# subject = " << subject_ << "\n";
    frames_csv_ << "frame_number,host_time_s,frame_rate_hz,latency_total_s\n";

    entities_csv_ << "# vicon_format = " << kViconFormat << "\n";
    entities_csv_
        << "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,invalid_reason\n";
}

void ViconRecorder::Write(const ViconSnapshot& snapshot) {
    frames_csv_ << snapshot.frame_number << "," << snapshot.host_time_s << ","
                << snapshot.frame_rate_hz << "," << snapshot.latency_total_s
                << "\n";

    for (const auto& marker : snapshot.markers) {
        WriteEntityRow(entities_csv_, snapshot.frame_number, "marker", "",
                       marker.name, marker.position_m.x(), marker.position_m.y(),
                       marker.position_m.z(), "", "", "", "", marker.valid,
                       marker.invalid_reason);
    }
    for (const auto& segment : snapshot.segments) {
        WriteEntityRow(
            entities_csv_, snapshot.frame_number, "segment", segment.subject_name,
            segment.segment_name, segment.position_m.x(), segment.position_m.y(),
            segment.position_m.z(), QuaternionField(segment.orientation.x()),
            QuaternionField(segment.orientation.y()),
            QuaternionField(segment.orientation.z()),
            QuaternionField(segment.orientation.w()), segment.valid,
            segment.invalid_reason);
    }
}
```

- [ ] **Step 5: Run to confirm it passes**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_vicon_recorder && /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build/test_vicon_recorder`
Expected: builds and exits 0.

- [ ] **Step 6: Commit**

```bash
git add Christian_control/vicon/src/ViconRecorder.h Christian_control/vicon/src/ViconRecorder.cpp Christian_control/vicon/tests/test_vicon_recorder.cpp Christian_control/vicon/CMakeLists.txt
git commit -m "vicon: add ViconRecorder (two-CSV house run-log layout)"
```

---

### Task 5: `ViconReplaySource` — read the recording back

**Files:**
- Create: `Christian_control/vicon/src/ViconReplaySource.h`
- Create: `Christian_control/vicon/src/ViconReplaySource.cpp`
- Test: `Christian_control/vicon/tests/test_vicon_replay.cpp`
- Modify: `Christian_control/vicon/CMakeLists.txt`

**Interfaces:**
- Consumes: `ViconSnapshot`, `MarkerSample`, `SegmentSample` (Task 1); the exact CSV layout written by `ViconRecorder` (Task 4).
- Produces: `class ViconReplaySource { ViconReplaySource(std::istream& frames_csv, std::istream& entities_csv); std::optional<ViconSnapshot> Next(); const std::string& LastError() const; }`. `Next()` returns `std::nullopt` both at ordinary end-of-recording (`LastError()` empty) and on a parse failure (`LastError()` non-empty) — callers distinguish the two by checking `LastError()`.

**Scope note on "staleness":** the approved design's Stage 1 acceptance
line groups "occlusion, staleness, truncated-file and unknown-format"
together as cases that must degrade with a reason. Occlusion, truncation
and unknown-format are properties of a single recorded frame and are
covered by the tests below. Staleness is not — it means "how old is this
sample relative to *now*", which only exists for a live consumer, not for
a value read from a historical recording (replaying an old file is not
itself "stale"; every replayed frame is exactly as old as its
`host_time_s` says). This plan implements staleness nowhere; it belongs
to a live consumer with a clock of its own — first the frame-number-repeat
skip already in `record_vicon` (Task 6), later Stage 3's controller
freshness gate. Flagged here rather than silently dropped.

- [ ] **Step 1: Write the failing test**

Create `Christian_control/vicon/tests/test_vicon_replay.cpp`:

```cpp
#include "SnapshotBuilder.h"
#include "ViconRecorder.h"
#include "ViconReplaySource.h"

#include <cassert>
#include <sstream>

namespace {

ViconSnapshot MakeSnapshot(unsigned int frame_number) {
    MarkerData marker;
    marker.name = "M1";
    marker.x = 1000.0;
    marker.y = 2000.0;
    marker.z = 3000.0;
    marker.occluded = false;

    SegmentData segment;
    segment.subject_name = "Dr Octopus Christian";
    segment.segment_name = "Mount";
    segment.x = 10.0;
    segment.y = 20.0;
    segment.z = 30.0;
    segment.qx = 0.0;
    segment.qy = 0.0;
    segment.qz = 0.0;
    segment.qw = 1.0;
    segment.occluded = false;

    return BuildSnapshot(frame_number, frame_number * 0.01, 100.0, 0.015,
                          {marker}, {segment});
}

}  // namespace

int main() {
    // Round trip: every field of two written snapshots survives read-back.
    {
        std::ostringstream frames_out, entities_out;
        ViconRecorder recorder(frames_out, entities_out, "192.168.128.206:801",
                                "Dr Octopus Christian");
        recorder.WriteHeader();
        const auto snap0 = MakeSnapshot(0);
        const auto snap1 = MakeSnapshot(1);
        recorder.Write(snap0);
        recorder.Write(snap1);

        std::istringstream frames_in(frames_out.str());
        std::istringstream entities_in(entities_out.str());
        ViconReplaySource replay(frames_in, entities_in);
        assert(replay.LastError().empty());

        const auto read0 = replay.Next();
        assert(read0.has_value());
        assert(read0->frame_number == snap0.frame_number);
        assert(read0->host_time_s == snap0.host_time_s);
        assert(read0->frame_rate_hz == snap0.frame_rate_hz);
        assert(read0->latency_total_s == snap0.latency_total_s);
        assert(read0->markers.size() == 1);
        assert(read0->markers[0].name == snap0.markers[0].name);
        assert(read0->markers[0].position_m == snap0.markers[0].position_m);
        assert(read0->markers[0].valid == snap0.markers[0].valid);
        assert(read0->segments.size() == 1);
        assert(read0->segments[0].subject_name == snap0.segments[0].subject_name);
        assert(read0->segments[0].segment_name == snap0.segments[0].segment_name);
        assert(read0->segments[0].position_m == snap0.segments[0].position_m);
        assert(read0->segments[0].orientation.coeffs() ==
               snap0.segments[0].orientation.coeffs());
        assert(read0->segments[0].valid == snap0.segments[0].valid);

        const auto read1 = replay.Next();
        assert(read1.has_value());
        assert(read1->frame_number == 1);

        const auto read2 = replay.Next();
        assert(!read2.has_value());
        assert(replay.LastError().empty());  // ordinary end, not an error
    }

    // Empty recording: zero frames after the header degrades with a reason.
    {
        std::ostringstream frames_out, entities_out;
        ViconRecorder recorder(frames_out, entities_out, "host", "subject");
        recorder.WriteHeader();

        std::istringstream frames_in(frames_out.str());
        std::istringstream entities_in(entities_out.str());
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
        assert(!replay.Next().has_value());
    }

    // Unknown vicon_format: degrades with a reason, does not crash.
    {
        std::istringstream frames_in(
            "# vicon_format = 2\n"
            "frame_number,host_time_s,frame_rate_hz,latency_total_s\n"
            "0,0,0,0\n");
        std::istringstream entities_in(
            "# vicon_format = 2\n"
            "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
            "invalid_reason\n");
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
        assert(!replay.Next().has_value());
    }

    // Truncated/malformed row: degrades with a reason, does not crash.
    {
        std::istringstream frames_in(
            "# vicon_format = 1\n"
            "frame_number,host_time_s,frame_rate_hz,latency_total_s\n"
            "0,0.0,100.0\n");  // one column short
        std::istringstream entities_in(
            "# vicon_format = 1\n"
            "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
            "invalid_reason\n");
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
        assert(!replay.Next().has_value());
    }

    // Missing column in the header itself: degrades with a reason.
    {
        std::istringstream frames_in(
            "# vicon_format = 1\n"
            "frame_number,host_time_s,frame_rate_hz\n"  // latency_total_s missing
            "0,0.0,100.0\n");
        std::istringstream entities_in(
            "# vicon_format = 1\n"
            "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
            "invalid_reason\n");
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
    }

    return 0;
}
```

- [ ] **Step 2: Wire the CMake target, run to confirm it fails**

Edit `Christian_control/vicon/CMakeLists.txt`. Change the `vicon_snapshot`
library's source list (Task 4) to:

```cmake
add_library(vicon_snapshot STATIC
        src/SnapshotBuilder.cpp
        src/ViconRecorder.cpp
        src/ViconReplaySource.cpp
)
```

Inside `if (BUILD_TESTING)`, after the `test_vicon_recorder` registration,
add:

```cmake
    add_executable(test_vicon_replay tests/test_vicon_replay.cpp)
    target_link_libraries(test_vicon_replay PRIVATE vicon_snapshot)
    enable_assertions_for(test_vicon_replay)
    add_test(NAME vicon_replay COMMAND test_vicon_replay)
```

(`enable_assertions_for` — see the note in Task 3.)

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_vicon_replay`
Expected: FAIL — `ViconReplaySource.h` does not exist yet.

- [ ] **Step 3: Write the header**

Create `Christian_control/vicon/src/ViconReplaySource.h`:

```cpp
#pragma once

#include "ViconSnapshot.h"

#include <istream>
#include <map>
#include <optional>
#include <string>
#include <vector>

// Reads back a recording written by ViconRecorder. Next() returns one
// ViconSnapshot per call, in the frame order stored in frames.csv, or
// std::nullopt when the stream is exhausted or the file could not be
// parsed -- LastError() distinguishes the two (empty string means an
// ordinary end of a valid recording).
class ViconReplaySource {
public:
    ViconReplaySource(std::istream& frames_csv, std::istream& entities_csv);

    std::optional<ViconSnapshot> Next();
    const std::string& LastError() const { return last_error_; }

private:
    struct FrameRow {
        unsigned int frame_number = 0;
        double host_time_s = 0.0;
        double frame_rate_hz = 0.0;
        double latency_total_s = 0.0;
    };

    bool ParseFrames(std::istream& frames_csv);
    bool ParseEntities(std::istream& entities_csv);

    std::vector<FrameRow> frames_;
    std::map<unsigned int, std::vector<MarkerSample>> markers_by_frame_;
    std::map<unsigned int, std::vector<SegmentSample>> segments_by_frame_;
    std::size_t next_index_ = 0;
    std::string last_error_;
};
```

- [ ] **Step 4: Write the implementation**

Create `Christian_control/vicon/src/ViconReplaySource.cpp`:

```cpp
#include "ViconReplaySource.h"

#include <sstream>
#include <stdexcept>

namespace {

std::vector<std::string> SplitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    std::istringstream stream(line);
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    // getline stops yielding fields once the stream hits EOF having
    // consumed zero characters -- which is exactly the state reached
    // right after the trailing comma on a line ending in ",". Every
    // ViconRecorder entity row ends this way whenever invalid_reason is
    // empty (i.e. every valid marker/segment), so without this, a normal
    // valid row parses one field short. One trailing comma can only ever
    // eat the single rightmost field (interior commas each still end on a
    // delimiter, so getline reads them as empty fields correctly), so
    // appending exactly one empty field is both necessary and sufficient.
    if (!line.empty() && line.back() == ',') {
        fields.push_back("");
    }
    return fields;
}

bool ParseUInt(const std::string& text, unsigned int& out) {
    if (text.empty()) return false;
    try {
        std::size_t consumed = 0;
        const unsigned long value = std::stoul(text, &consumed);
        if (consumed != text.size()) return false;
        out = static_cast<unsigned int>(value);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool ParseDouble(const std::string& text, double& out) {
    if (text.empty()) return false;
    try {
        std::size_t consumed = 0;
        out = std::stod(text, &consumed);
        return consumed == text.size();
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

ViconReplaySource::ViconReplaySource(std::istream& frames_csv,
                                      std::istream& entities_csv) {
    if (!ParseFrames(frames_csv)) return;
    if (!ParseEntities(entities_csv)) return;
    if (frames_.empty()) {
        last_error_ = "recording contains zero frames";
    }
}

bool ViconReplaySource::ParseFrames(std::istream& frames_csv) {
    std::string line;
    bool saw_format = false;
    bool have_header = false;
    while (std::getline(frames_csv, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') {
            if (line.find("vicon_format = 1") != std::string::npos) {
                saw_format = true;
            }
            continue;
        }
        have_header = true;
        break;
    }
    if (!have_header) {
        last_error_ = "frames.csv has no header row";
        return false;
    }
    if (!saw_format) {
        last_error_ = "unknown or missing vicon_format in frames.csv";
        return false;
    }

    const auto header = SplitCsvLine(line);
    if (header.size() != 4 || header[0] != "frame_number") {
        last_error_ = "frames.csv header missing expected columns";
        return false;
    }

    while (std::getline(frames_csv, line)) {
        if (line.empty()) continue;
        const auto fields = SplitCsvLine(line);
        if (fields.size() != 4) {
            last_error_ = "frames.csv row has the wrong number of columns";
            return false;
        }
        FrameRow row;
        double host_time = 0, rate = 0, latency = 0;
        if (!ParseUInt(fields[0], row.frame_number) ||
            !ParseDouble(fields[1], host_time) ||
            !ParseDouble(fields[2], rate) || !ParseDouble(fields[3], latency)) {
            last_error_ = "frames.csv row could not be parsed";
            return false;
        }
        row.host_time_s = host_time;
        row.frame_rate_hz = rate;
        row.latency_total_s = latency;
        frames_.push_back(row);
    }
    return true;
}

bool ViconReplaySource::ParseEntities(std::istream& entities_csv) {
    std::string line;
    bool have_header = false;
    while (std::getline(entities_csv, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') continue;
        have_header = true;
        break;
    }
    if (!have_header) {
        last_error_ = "entities.csv has no header row";
        return false;
    }

    const auto header = SplitCsvLine(line);
    if (header.size() != 13 || header[0] != "frame_number") {
        last_error_ = "entities.csv header missing expected columns";
        return false;
    }

    while (std::getline(entities_csv, line)) {
        if (line.empty()) continue;
        const auto fields = SplitCsvLine(line);
        if (fields.size() != 13) {
            last_error_ = "entities.csv row has the wrong number of columns";
            return false;
        }
        unsigned int frame_number = 0;
        if (!ParseUInt(fields[0], frame_number)) {
            last_error_ = "entities.csv row could not be parsed";
            return false;
        }
        const std::string& kind = fields[1];
        double x = 0, y = 0, z = 0;
        if (!ParseDouble(fields[4], x) || !ParseDouble(fields[5], y) ||
            !ParseDouble(fields[6], z)) {
            last_error_ = "entities.csv row could not be parsed";
            return false;
        }
        const bool valid = (fields[11] == "1");
        const std::string& invalid_reason = fields[12];

        if (kind == "marker") {
            MarkerSample sample;
            sample.name = fields[3];
            sample.position_m = Eigen::Vector3d(x, y, z);
            sample.valid = valid;
            sample.invalid_reason = invalid_reason;
            markers_by_frame_[frame_number].push_back(sample);
        } else if (kind == "segment") {
            double qx = 0, qy = 0, qz = 0, qw = 0;
            if (!ParseDouble(fields[7], qx) || !ParseDouble(fields[8], qy) ||
                !ParseDouble(fields[9], qz) || !ParseDouble(fields[10], qw)) {
                last_error_ = "entities.csv row could not be parsed";
                return false;
            }
            SegmentSample sample;
            sample.subject_name = fields[2];
            sample.segment_name = fields[3];
            sample.position_m = Eigen::Vector3d(x, y, z);
            sample.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
            sample.valid = valid;
            sample.invalid_reason = invalid_reason;
            segments_by_frame_[frame_number].push_back(sample);
        } else {
            last_error_ = "entities.csv row has an unknown kind";
            return false;
        }
    }
    return true;
}

std::optional<ViconSnapshot> ViconReplaySource::Next() {
    if (next_index_ >= frames_.size()) return std::nullopt;
    if (!last_error_.empty()) return std::nullopt;

    const auto& row = frames_[next_index_];
    ViconSnapshot snapshot;
    snapshot.frame_number = row.frame_number;
    snapshot.host_time_s = row.host_time_s;
    snapshot.frame_rate_hz = row.frame_rate_hz;
    snapshot.latency_total_s = row.latency_total_s;

    const auto markers_it = markers_by_frame_.find(row.frame_number);
    if (markers_it != markers_by_frame_.end()) snapshot.markers = markers_it->second;
    const auto segments_it = segments_by_frame_.find(row.frame_number);
    if (segments_it != segments_by_frame_.end())
        snapshot.segments = segments_it->second;

    ++next_index_;
    return snapshot;
}
```

Note: `ParseFrames`/`ParseEntities` explicitly track `have_header` rather
than testing whether the preceding `while` loop's last line looked like a
header. A file that is empty, or contains only `#` lines, exits that loop
with `std::getline` having returned `false` and `line` still holding
whatever `#` line was last read — without the flag, that stale value would
be misparsed as a header instead of reported as truncation.

- [ ] **Step 5: Run to confirm it passes**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target test_vicon_replay && /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build/test_vicon_replay`
Expected: builds and exits 0.

- [ ] **Step 6: Run the full vicon CTest suite**

Run: `cd /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build && ctest --output-on-failure`
Expected: all tests pass (`vicon_segment_data`, `vicon_snapshot`,
`vicon_snapshot_builder`, `vicon_recorder`, `vicon_replay`).

- [ ] **Step 7: Commit**

```bash
git add Christian_control/vicon/src/ViconReplaySource.h Christian_control/vicon/src/ViconReplaySource.cpp Christian_control/vicon/tests/test_vicon_replay.cpp Christian_control/vicon/CMakeLists.txt
git commit -m "vicon: add ViconReplaySource (round-trip + degrade on malformed input)"
```

---

### Task 6: `record_vicon` — live capture tool for the lab

**Files:**
- Create: `Christian_control/vicon/tools/record_vicon.cpp`
- Modify: `Christian_control/vicon/CMakeLists.txt`

**Interfaces:**
- Consumes: `ViconInterface` (existing, plus `getLatencyTotal()` from Task 2), `BuildSnapshot` (Task 3), `ViconRecorder` (Task 4).
- Produces: an executable `record_vicon <host:port> <label> [duration_s] [subject]` that writes `runs/<YYYY-MM-DD>/vicon_<label>/{frames,entities}.csv`. This is the tool Stage 0 (Task 7) uses in the lab; it has no unit test (it is a live-I/O CLI) — verified by building and by manual use against the real server.

- [ ] **Step 1: Write the tool**

Create `Christian_control/vicon/tools/record_vicon.cpp`:

```cpp
// Sensing-only Vicon recorder: connects, converts every frame through
// SnapshotBuilder, and writes it via ViconRecorder into
// runs/YYYY-MM-DD/vicon_<label>/{frames,entities}.csv. No Kortex, no
// robot -- this only reads Vicon and writes two CSVs.
//
// Usage: record_vicon <host:port> <label> [duration_s] [subject]
//   duration_s defaults to 10; subject defaults to "Dr Octopus Christian".
//
// Stage 0 uses this twice: once for a static recording (arms and wearer
// both still) and once with the arms still while the wearer moves, to
// settle whether the Mount segment markers are on the rigid backpack
// plate or on the wearer's body.

#include "SnapshotBuilder.h"
#include "ViconInterface.h"
#include "ViconRecorder.h"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {

std::string TodayFolderName() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&now_c, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d");
    return oss.str();
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: record_vicon <host:port> <label> [duration_s] "
                     "[subject]\n";
        return 1;
    }
    const std::string host = argv[1];
    const std::string label = argv[2];
    const double duration_s = (argc > 3) ? std::stod(argv[3]) : 10.0;
    const std::string subject = (argc > 4) ? argv[4] : "Dr Octopus Christian";

    ViconInterface vicon;
    if (!vicon.connect(host)) {
        return 1;
    }

    const std::filesystem::path dir =
        std::filesystem::path("runs") / TodayFolderName() / ("vicon_" + label);
    std::filesystem::create_directories(dir);

    std::ofstream frames_csv(dir / "frames.csv");
    std::ofstream entities_csv(dir / "entities.csv");
    if (!frames_csv || !entities_csv) {
        std::cerr << "Could not open output files under " << dir << "\n";
        return 1;
    }

    ViconRecorder recorder(frames_csv, entities_csv, host, subject);
    recorder.WriteHeader();

    const auto start = std::chrono::steady_clock::now();
    const auto deadline =
        start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(duration_s));

    unsigned int frames_written = 0;
    int prev_frame_number = -1;
    while (std::chrono::steady_clock::now() < deadline) {
        const int frame_number = vicon.getFrameNumber();
        if (frame_number == prev_frame_number) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (!vicon.getFrame()) break;
        prev_frame_number = frame_number;

        const double host_time_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count();
        const auto markers = vicon.getMarkerPositions(std::string(""));
        const auto segments = vicon.getSegmentPoses();
        const auto snapshot = BuildSnapshot(
            static_cast<unsigned int>(frame_number), host_time_s,
            vicon.getFrameRate(), vicon.getLatencyTotal(), markers, segments);
        recorder.Write(snapshot);
        ++frames_written;
    }

    vicon.disconnect();
    std::cout << "Wrote " << frames_written << " frames to " << dir.string()
              << "\n";
    return 0;
}
```

- [ ] **Step 2: Wire the CMake target**

Edit `Christian_control/vicon/CMakeLists.txt`. After the existing
`connect_vicon` block (after its `set_target_properties(connect_vicon
PROPERTIES BUILD_RPATH ...)`), add:

```cmake

add_executable(record_vicon tools/record_vicon.cpp)
target_link_libraries(record_vicon PRIVATE vicon_snapshot vicon_interface)
set_target_properties(record_vicon PROPERTIES
        BUILD_RPATH "${VICON_SDK}/lib"
)
```

- [ ] **Step 3: Build and confirm**

Run: `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build --target record_vicon`
Expected: builds cleanly with no first-party warnings.

This cannot be run against the live server in this session — port 801 is
currently closed on both lab hosts (`.206`, `.210`), confirmed earlier via
`connect_vicon` returning `ClientConnectionFailed`. Live verification is
Task 7, run by Christian in the lab.

- [ ] **Step 4: Commit**

```bash
git add Christian_control/vicon/tools/record_vicon.cpp Christian_control/vicon/CMakeLists.txt
git commit -m "vicon: add record_vicon capture tool"
```

---

### Task 7: Stage 0 — lab verification and frame-contract decision record

**Files:**
- Create: `Christian_control/docs/decisions/vicon-frame-contract.md`

**Interfaces:**
- Consumes: `connect_vicon` (existing), `record_vicon` (Task 6).
- Produces: a decision record documenting the axis convention, segment template notes, and the two captured recordings' paths — the input Stage 2's calibration tool (a future plan) reads from.

This task is not code — it is the lab procedure itself, executed by
Christian (physical presence, Nexus GUI access, and the marker rig are
required; this cannot be done by an autonomous worker). Do not mark this
task's checklist items complete until Christian has actually performed
them in the lab.

- [ ] **Step 1: Create the decision record template**

Create `Christian_control/docs/decisions/vicon-frame-contract.md`:

```markdown
# Vicon frame contract — Stage 0 lab record

Status: template — fill in during the lab session, then commit.

## Streaming restored

- [ ] Nexus/Tracker is running and publishing (port 801 reachable from
      this machine — Windows firewall blocked it on 2026-08-10).
- [ ] `./build/connect_vicon <host:port>` exits 0 and reports 10 frames.
- Host used: _____________ (`.206` is the code default; `.210` was
  observed live on 2026-08-10 — record whichever actually answers).

## Segment verification

- [ ] All five expected segments (`Mount`, `LeftBase`, `RightBase`,
      `LeftEE`, `RightEE`) appear in `connect_vicon`'s segment listing,
      `occluded = false`, and move plausibly when nudged.
- Segments actually observed: _____________
- Any segment still occluded/absent: _____________ (if any, Stage 2 needs
  a fallback for that segment — note it here, do not proceed silently)

## Axis convention

- [ ] Nexus's configured axis mapping recorded here (Vicon defaults to
      Z-up; confirm rather than assume — neither `ViconInterface` project
      has ever called `SetAxisMapping`).
- Convention: _____________

## Segment template notes

For each of the five segments, as built in Nexus:
- `Mount`: origin/axis choice — _____________
- `LeftBase`: — _____________
- `RightBase`: — _____________
- `LeftEE`: — _____________
- `RightEE`: — _____________

## Recordings captured

Run from `Christian_control/vicon/build/`:

```
./record_vicon <host:port> static 15
./record_vicon <host:port> wearer_moving 20
```

- [ ] `static`: arms and wearer both stationary. Written to:
      `runs/<date>/vicon_static/{frames,entities}.csv`
- [ ] `wearer_moving`: arms stationary, wearer leans/twists/breathes/steps
      (the Mount-rigidity experiment — settles whether the `Mount`
      segment markers are on the rigid backpack plate or the wearer's
      body). Written to: `runs/<date>/vicon_wearer_moving/{frames,entities}.csv`

## Acceptance (from the approved design)

- [ ] Five valid segments observed live.
- [ ] Axis convention and segment template notes recorded above.
- [ ] Both recordings captured and confirmed non-empty (`record_vicon`
      reported a nonzero frame count for each).

## Open question this settles

`Christian_control/docs/superpowers/specs/2026-08-08-vicon-integration-design.md`
§4.4 and
`docs/superpowers/specs/2026-08-12-vicon-controller-planner-integration-design.md`
open question 1: are the `Mount` segment markers on the rigid backpack
plate? Stage 2's calibration tool will compute
`^ref T_leftbase` per frame from the `wearer_moving` recording and plot
its translation norm / rotation angle over time — constant to within
noise means rigid (proceed); systematic variation with posture means the
markers are on the body (the calibration approach in Stage 2 needs
revisiting before it is built). Record the outcome here once Stage 2 runs
this recording through that check.
```

- [ ] **Step 2: Commit the template**

```bash
git add Christian_control/docs/decisions/vicon-frame-contract.md
git commit -m "vicon: add Stage 0 lab checklist and decision-record template"
```

- [ ] **Step 3: Execute the lab session (Christian, not an autonomous worker)**

Follow the template above end to end with Christian present at the lab
machine. Fill in every field, check off every box, and commit the
completed record as a separate commit once done:

```bash
git add Christian_control/docs/decisions/vicon-frame-contract.md
git commit -m "vicon: record Stage 0 lab session results"
```

---

## Plan-level verification

After Task 6, before Task 7's lab session:

- [ ] `cmake --build /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build` — full project builds clean, zero first-party warnings.
- [ ] `cd /home/christian/Desktop/HumanSL_MAIN/Christian_control/vicon/build && ctest --output-on-failure` — all tests pass: `vicon_segment_data`, `vicon_snapshot`, `vicon_snapshot_builder`, `vicon_recorder`, `vicon_replay`.
- [ ] `ldd build/record_vicon` and `ldd build/connect_vicon` — confirm neither links Kortex (grep for `kortex`, expect no match), matching the standing hardware constraint.
- [ ] No robot-facing binary was built or executed by this plan.

## What Stage 2 needs from this plan, stated so the boundary is explicit

Stage 2 (a separate future plan, not part of this one) will read the two
recordings via `ViconReplaySource`, compute `mountseg_T_mount` by least
squares against the URDF's known `mount_T_leftbase` /
`mount_T_rightbase`, and run the `wearer_moving` recording through the
Mount-rigidity check named in Task 7. Nothing in this plan performs that
computation — it only produces the recordings and the tool that made them.
