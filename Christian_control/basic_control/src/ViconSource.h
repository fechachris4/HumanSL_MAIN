//
// ViconSource — the acquisition/filter thread that feeds the controller's
// coherent BasePoseSlot.
//
// The seam that keeps the standalone build true: this header names no SDK
// type, and CMake picks one of two implementations —
//
//   ViconSource.cpp      built when third_party/vicon_api is present;
//                        owns a thread that connects, reads frames, and
//                        publishes BasePoseSamples into the slot.
//   ViconSourceStub.cpp  built otherwise; StartViconSource returns nullptr
//                        and the run proceeds with no world-pose data —
//                        every vicon_* log column stays NaN/0, which the
//                        log format documents as honest absence.
//
// Threading: the returned object owns exactly one thread. It never touches
// the robot, Kortex, or the control loop's data; its only shared state is
// the wait-free slot. Its destructor stops and joins the thread — destroy
// it after RunControlLoop returns (Main owns it), never from the loop.
// Connection failures are retried inside the thread (the controller must
// start, take over, and run whether or not Vicon is reachable); lifecycle
// events are printed from this thread, never from the control loop.
//

#pragma once

#include <memory>

class BasePoseSlot;

// "sdk" or "absent" — which implementation this binary was built with.
// Written into the run log's preamble so a log with all-NaN vicon columns
// says WHY: source absent versus present-but-never-connected.
extern const char* const kViconSourceBuildMode;

// The DataStream endpoint this build connects to. A compile-time constant
// (same policy as connect_vicon's default): 192.168.128.206:801, verified
// live 2026-08-13; .210 pings but serves nothing. In the stub build this
// still names the endpoint the SDK build would use, for the preamble.
extern const char* const kViconSourceHost;

// Starts acquisition into `slot`, which must outlive the returned object.
// nullptr means this binary has no Vicon source (stub build) — callers
// print the reason and continue in zero-error awaiting-world hold. No world
// trajectory can activate until a fresh sample exists.
std::unique_ptr<class ViconSource> StartViconSource(BasePoseSlot& slot);

class ViconSource
{
public:
    virtual ~ViconSource() = default;
};
