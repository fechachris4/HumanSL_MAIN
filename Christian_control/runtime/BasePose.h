//
// BasePose — the coherent world pose/twist sample consumed and logged by the
// controller.
//
// Three pieces, in reading order:
//
//   1. BasePoseSample — the sample contract from
//      docs/thesis/world-frame-hold-derivation.md §5: Vicon's frame
//      number, our own sequence number, the steady_clock receive time,
//      the SDK-reported latency, and the five named segments' poses in
//      Vicon-world metres. sequence == 0 means "no sample has ever
//      arrived", and every pose field of that state is NaN with
//      valid=false — absence must be unmistakable, never a plausible
//      zero (the same policy the run log already uses for NaN).
//   2. ToBasePoseSample — pure mapping from a validated ViconSnapshot
//      (../tracking/src, SDK-free value type; SnapshotBuilder has already
//      done mm→m and quaternion checks) into the fixed-size sample.
//      Segments are matched by name, subject-agnostic; anything absent
//      or invalid stays NaN/invalid.
//   3. BasePoseSlot — the handoff between the acquisition thread (writes
//      at the Vicon rate, ~100 Hz) and the 500 Hz control thread (reads
//      every cycle, for measurement and logging). Single producer, single
//      consumer, wait-free triple buffer: the three buffers are
//      partitioned between the two threads by atomic index exchanges, so
//      no buffer is ever touched by both threads at once — a reader can
//      never observe a torn sample, and neither side ever blocks.
//
// Timing note: t_receive_s is steady_clock seconds (time_since_epoch), the
// same clock RunControlLoop paces with, so age = now − t_receive_s is
// exact. Between Vicon frames the reader keeps seeing the same sequence —
// zero-order hold, with age growing. NOTHING may finite-difference a
// reused sample (derivation doc §5). Differentiation belongs to the Vicon
// producer and only occurs when source frame numbers advance.
//

#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

#include "MountTwistEstimator.h"
#include "ViconSnapshot.h"

// The five segments streamed by the lab's Nexus subject, in log-column
// order. Matched by segment name only — the subject name changed once
// already ("Dr Octopus Christian" → "Christian Test") and must not matter.
inline constexpr int kBasePoseSegmentCount = 5;
inline constexpr const char* kBasePoseSegmentNames[kBasePoseSegmentCount] = {
    "Mount", "LeftBase", "RightBase", "LeftEE", "RightEE",
};
inline constexpr int kBasePoseMount = 0;
inline constexpr int kBasePoseLeftBase = 1;
inline constexpr int kBasePoseRightBase = 2;
inline constexpr int kBasePoseLeftEE = 3;
inline constexpr int kBasePoseRightEE = 4;

// One segment's pose in Vicon-world: metres, quaternion stored x,y,z,w
// (the run log's tool_quat_xyzw order). valid=false ⇒ every number is NaN.
struct BasePoseSegmentPose {
    double position_m[3] = {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN()};
    double quat_xyzw[4] = {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN()};
    bool valid = false;
};

// The full sample contract. Plain fixed-size value — copied whole through
// the slot and into the log, no allocation anywhere.
struct BasePoseSample {
    std::uint32_t vicon_frame_number = 0; // Vicon's own frame counter
    std::uint64_t sequence = 0; // ours; ++ per NEW sample; 0 = never any
    double t_receive_s = // steady_clock seconds at the SDK read
        std::numeric_limits<double>::quiet_NaN();
    double frame_rate_hz = // SDK GetFrameRate at that read
        std::numeric_limits<double>::quiet_NaN();
    double latency_reported_s = // SDK GetLatencyTotal at that read
        std::numeric_limits<double>::quiet_NaN();
    // Filtered Mount spatial twist in Vicon-world axes, estimated only from
    // advancing valid Vicon frames. It shares this sample's frame/sequence
    // provenance and triple-buffer publication with the Mount pose, so a
    // reader cannot observe a twist from another frame. Invalid means all six
    // numbers remain NaN rather than resembling a stationary measurement.
    double mount_linear_world_m_s[3] = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    double mount_angular_world_rad_s[3] = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    bool mount_twist_valid = false;
    BasePoseSegmentPose segments[kBasePoseSegmentCount];
};

// Age of a sample against the same steady clock it was stamped with.
// NaN while no sample has ever arrived — never a plausible zero.
inline double BasePoseAgeS(const BasePoseSample& sample, double now_steady_s)
{
    if (sample.sequence == 0)
        return std::numeric_limits<double>::quiet_NaN();
    return now_steady_s - sample.t_receive_s;
}

// Pure mapping: validated snapshot → fixed-size sample. The snapshot's
// segments are already metres with checked quaternions (SnapshotBuilder);
// this only reshapes them into the five named slots. Unknown segments are
// ignored; missing or invalid ones keep the NaN/invalid default.
inline BasePoseSample ToBasePoseSample(const ViconSnapshot& snapshot,
                                       std::uint64_t sequence,
                                       const MountTwistEstimate& mount_twist = {})
{
    BasePoseSample sample;
    sample.vicon_frame_number = snapshot.frame_number;
    sample.sequence = sequence;
    sample.t_receive_s = snapshot.host_time_s;
    sample.frame_rate_hz = snapshot.frame_rate_hz;
    sample.latency_reported_s = snapshot.latency_total_s;
    if (mount_twist.valid &&
        mount_twist.source_frame_number == snapshot.frame_number &&
        mount_twist.linear_m_s.allFinite() &&
        mount_twist.angular_rad_s.allFinite()) {
        for (int i = 0; i < 3; ++i) {
            sample.mount_linear_world_m_s[i] = mount_twist.linear_m_s[i];
            sample.mount_angular_world_rad_s[i] =
                mount_twist.angular_rad_s[i];
        }
        sample.mount_twist_valid = true;
    }
    for (const SegmentSample& segment : snapshot.segments) {
        for (int slot = 0; slot < kBasePoseSegmentCount; ++slot) {
            if (segment.segment_name != kBasePoseSegmentNames[slot])
                continue;
            if (!segment.valid)
                break; // stays NaN/invalid — occlusion is not a pose
            BasePoseSegmentPose& out = sample.segments[slot];
            for (int i = 0; i < 3; ++i)
                out.position_m[i] = segment.position_m[i];
            out.quat_xyzw[0] = segment.orientation.x();
            out.quat_xyzw[1] = segment.orientation.y();
            out.quat_xyzw[2] = segment.orientation.z();
            out.quat_xyzw[3] = segment.orientation.w();
            out.valid = true;
            break;
        }
    }
    return sample;
}

// Single-producer / single-consumer latest-value slot. Wait-free on both
// sides: three buffers, one owned by the writer, one owned by the reader,
// one in the middle handed over by atomic exchange. Ownership transfers
// wholesale with each exchange, so a buffer is never read and written at
// the same time. The middle index carries a "new" bit so the reader can
// tell a fresh publish from its own returned buffer.
//
// Thread contract (matches LoopLog's style): exactly one thread calls
// Publish (the acquisition thread), exactly one calls ReadLatest (the
// control loop). ReadLatest never blocks, never allocates, and copies one
// fixed-size sample — the only cost the 500 Hz path pays.
class BasePoseSlot
{
public:
    // Writer side. Copies `sample` into the writer's buffer, then swaps
    // that buffer into the middle (release: the copy happens-before any
    // reader that acquires the index).
    void Publish(const BasePoseSample& sample)
    {
        buffers_[write_index_] = sample;
        const int previous = middle_.exchange(write_index_ | kNewBit,
                                              std::memory_order_acq_rel);
        write_index_ = previous & kIndexMask;
    }

    // Reader side. Takes the middle buffer if it holds a new publish,
    // otherwise keeps the buffer it already owns (zero-order hold; the
    // caller sees the same sequence again and the age keeps growing).
    // Returns false while no sample has ever been published.
    bool ReadLatest(BasePoseSample& out)
    {
        if (middle_.load(std::memory_order_relaxed) & kNewBit) {
            const int previous = middle_.exchange(
                read_index_, std::memory_order_acq_rel);
            read_index_ = previous & kIndexMask;
        }
        out = buffers_[read_index_];
        return out.sequence != 0;
    }

private:
    static constexpr int kIndexMask = 0x3;
    static constexpr int kNewBit = 0x4;

    BasePoseSample buffers_[3];
    std::atomic<int> middle_{1}; // no kNewBit: nothing published yet
    int write_index_ = 0;        // writer thread only
    int read_index_ = 2;         // reader thread only
};
