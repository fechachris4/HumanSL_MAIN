# Vicon Segment Reception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the standalone Vicon wrapper receive and expose every subject segment pose in the current DataStream frame.

**Architecture:** Add a plain `SegmentData` value type and `getSegmentPoses()` to the existing `ViconInterface`; do not introduce a client abstraction. Enable the SDK's segment stream during connection, enumerate subjects and their segments from the latest frame, and expose Vicon-global position and quaternion unchanged. Keep integration with the robot controller out of scope.

**Tech Stack:** C++17, CMake/CTest, vendored Vicon DataStream C++ SDK.

## Global Constraints

- Preserve marker, unlabeled-marker, and device behaviour.
- `SegmentData` position is Vicon global-frame millimetres.
- `SegmentData` quaternion order is Vicon SDK `(x, y, z, w)`; `w` is real.
- `getSegmentPoses()` reads the most recently acquired frame and does not perform I/O itself.
- An occluded segment remains in the result with `occluded=true`; consumers must reject its numeric pose.
- No Kinova, control-loop, actuator, timing, or controller changes.
- Do not commit, push, or operate the robot.

---

### Task 1: Add an offline Vicon interface contract test

**Files:**
- Create: `vicon/tests/test_segment_data.cpp`
- Modify: `vicon/CMakeLists.txt`

**Interfaces:**
- Consumes: `ViconInterface` declared in `vicon/src/ViconInterface.h`.
- Produces: CTest test `vicon_segment_data` that never connects to a Vicon server.

- [ ] **Step 1: Write the failing test**

```cpp
#include "ViconInterface.h"

#include <cassert>

int main() {
    SegmentData empty_segment;
    assert(empty_segment.subject_name.empty());
    assert(empty_segment.segment_name.empty());
    assert(empty_segment.occluded);

    ViconInterface vicon;
    assert(vicon.getSegmentPoses().empty());
    return 0;
}
```

Register it in `vicon/CMakeLists.txt` with:

```cmake
include(CTest)
if (BUILD_TESTING)
    add_executable(test_segment_data tests/test_segment_data.cpp)
    target_link_libraries(test_segment_data PRIVATE vicon_interface)
    add_test(NAME vicon_segment_data COMMAND test_segment_data)
endif ()
```

- [ ] **Step 2: Run the test to verify it fails for the missing API**

Run:

```bash
cmake -S vicon -B vicon/build
cmake --build vicon/build --target test_segment_data --parallel 2
```

Expected: compilation fails because `SegmentData` and `getSegmentPoses()` are not declared.

- [ ] **Step 3: Implement the minimal data contract and offline failure behaviour**

In `vicon/src/ViconInterface.h`, add:

```cpp
struct SegmentData {
    std::string subject_name;
    std::string segment_name;
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    bool occluded = true;
};
```

and declare:

```cpp
std::vector<SegmentData> getSegmentPoses();
```

In `vicon/src/ViconInterface.cpp`, begin the method with:

```cpp
std::vector<SegmentData> ViconInterface::getSegmentPoses() {
    std::vector<SegmentData> segments;
    if (!connected_) return segments;
    // Subject/segment enumeration is added in Task 2.
    return segments;
}
```

- [ ] **Step 4: Run the offline test to verify it passes**

Run:

```bash
cmake --build vicon/build --target test_segment_data --parallel 2
ctest --test-dir vicon/build --output-on-failure -R '^vicon_segment_data$'
```

Expected: `vicon_segment_data` passes without contacting Vicon.

### Task 2: Receive and display the current frame's segment poses

**Files:**
- Modify: `vicon/src/ViconInterface.cpp`
- Modify: `vicon/tools/connect_vicon.cpp`
- Test: `vicon/tests/test_segment_data.cpp`

**Interfaces:**
- Consumes: `SegmentData` and `ViconInterface::getSegmentPoses()` from Task 1.
- Produces: all current-frame Vicon segments for callers and diagnostic output from `connect_vicon`.

- [ ] **Step 1: Extend the test with the stable no-connection guarantee**

Keep the assertion below in `test_segment_data.cpp` after any test refactor:

```cpp
ViconInterface vicon;
assert(vicon.getSegmentPoses().empty());
```

This test must retain its no-network property and fails if the new method attempts a Vicon connection implicitly.

- [ ] **Step 2: Run the test before changing SDK integration**

Run:

```bash
ctest --test-dir vicon/build --output-on-failure -R '^vicon_segment_data$'
```

Expected: pass; it establishes the existing offline baseline before the SDK integration change.

- [ ] **Step 3: Enable and enumerate SDK segment data**

In `ViconInterface::connect()`, call `client_->EnableSegmentData()` after a successful `Connect()` and before the first-frame wait.

Implement `getSegmentPoses()` with this structure:

```cpp
const unsigned int subject_count = client_->GetSubjectCount().SubjectCount;
for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index) {
    const std::string subject_name = client_->GetSubjectName(subject_index).SubjectName;
    const unsigned int segment_count = client_->GetSegmentCount(subject_name).SegmentCount;
    for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index) {
        const std::string segment_name = client_->GetSegmentName(subject_name, segment_index).SegmentName;
        const auto translation = client_->GetSegmentGlobalTranslation(subject_name, segment_name);
        const auto rotation = client_->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
        if (translation.Result != Result::Success || rotation.Result != Result::Success) continue;

        SegmentData segment;
        segment.subject_name = subject_name;
        segment.segment_name = segment_name;
        segment.x = translation.Translation[0];
        segment.y = translation.Translation[1];
        segment.z = translation.Translation[2];
        segment.qx = rotation.Rotation[0];
        segment.qy = rotation.Rotation[1];
        segment.qz = rotation.Rotation[2];
        segment.qw = rotation.Rotation[3];
        segment.occluded = translation.Occluded || rotation.Occluded;
        segments.push_back(segment);
    }
}
```

In `connect_vicon.cpp`, retrieve `getSegmentPoses()` after the marker lists and print each entry's subject, segment, translation in millimetres, quaternion in `(x,y,z,w)` order, and occlusion state.

- [ ] **Step 4: Rebuild and run the full offline Vicon test suite**

Run:

```bash
cmake --build vicon/build --target connect_vicon test_segment_data --parallel 2
ctest --test-dir vicon/build --output-on-failure
```

Expected: both targets build and `vicon_segment_data` passes.

- [ ] **Step 5: Run the sensing-only smoke test when Vicon is live**

Run:

```bash
timeout 15s vicon/build/connect_vicon
```

Expected: ten frames print the existing markers plus each available Vicon segment. This command reads Vicon data only and is not a robot command.

- [ ] **Step 6: Review scope and diff**

Run:

```bash
git diff --check
git diff -- vicon docs/superpowers
git status --short
```

Expected: only the Vicon source, Vicon test/CMake registration, existing default-host edit, and the design/plan documents have changed. Do not commit.
