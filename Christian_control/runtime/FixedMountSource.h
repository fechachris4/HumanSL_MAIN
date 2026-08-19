//
// FixedMountSource — the fixed-mount counterpart of ViconSource: a small
// thread that fills the controller's BasePoseSlot with a CONSTANT
// world_T_mount instead of a measured one.
//
// Why a producer thread rather than a "fixed mode" flag inside the core:
// the core's world-freshness classification, hold capture and replan edge
// all consume one BasePoseSample contract (BasePose.h). Publishing a
// constant pose through that same contract means there is exactly one
// world-input path in the controller, and the run log records honestly
// where the pose came from (mount_source = fixed in the preamble; the
// vicon_* columns carry this source's samples like any other's).
//
// What a sample contains: the Mount segment at the given pose, valid, with
// an identically-zero mount twist (a rigid bench mount does not move — zero
// is the measurement, not an absence, so mount_twist_valid is true). The
// four other segments stay NaN/invalid: nothing measures them. The
// vicon_frame_number stays 0 — there is no Vicon frame — and latency NaN.
//
// Threading: same shape as ViconSource. One owned thread publishing at
// kPublishRateHz; its only shared state is the wait-free slot; the
// destructor stops and joins. Destroy it after RunControlLoop returns
// (Main owns it), never from the loop.
//

#pragma once

#include <atomic>
#include <thread>

#include <Eigen/Geometry>

class BasePoseSlot;

class FixedMountSource
{
public:
    // ~Vicon's frame rate, so the loop's zero-order-hold cadence and the
    // age the freshness classifier sees match a live-Vicon run.
    static constexpr double kPublishRateHz = 100.0;

    // Starts publishing `world_T_mount` into `slot`, which must outlive
    // this object. The rotation part must be a rotation (callers build it
    // from a unit quaternion — ParseMainArgs enforces that).
    FixedMountSource(BasePoseSlot& slot,
                     const Eigen::Isometry3d& world_T_mount);
    ~FixedMountSource();

    FixedMountSource(const FixedMountSource&) = delete;
    FixedMountSource& operator=(const FixedMountSource&) = delete;

private:
    void Run();

    BasePoseSlot& slot_;
    const Eigen::Vector3d position_m_;
    const Eigen::Quaterniond rotation_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};
