//
// BasePose contract tests — the world-pose sample the controller LOGS in
// slice 1 (observe-only; no control law reads it). Three things are pinned:
//
//   1. The sample contract: sequence 0 means "no Vicon sample has ever
//      arrived", and every pose field of that state is NaN with valid=false
//      — absence must be unmistakable, never a plausible zero.
//   2. The snapshot→sample mapping: the five named segments are matched by
//      name (subject-agnostic), metres and quaternions copied through, a
//      missing or invalid segment stays NaN/invalid.
//   3. The slot: single-writer/single-reader triple buffer. The reader
//      always gets a complete, untorn sample; sequence never runs
//      backwards; reading before any publish reports absence.
//
// Hardware-free: no SDK, no Kortex — ViconSnapshot.h is a plain value type.
//

#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include "BasePose.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    SegmentSample MakeSegment(const std::string& name, double x, bool valid)
    {
        SegmentSample segment;
        segment.subject_name = "Christian Test";
        segment.segment_name = name;
        segment.position_m = Eigen::Vector3d(x, x + 0.1, x + 0.2);
        segment.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        segment.valid = valid;
        return segment;
    }
} // namespace

int main()
{
    // --- 1. Default sample: absence is NaN + invalid + sequence 0
    {
        BasePoseSample sample;
        Check(sample.sequence == 0, "default sample has sequence 0");
        Check(std::isnan(sample.latency_reported_s),
              "default latency is NaN");
        Check(std::isnan(sample.t_receive_s), "default receive time is NaN");
        for (int seg = 0; seg < kBasePoseSegmentCount; ++seg) {
            Check(!sample.segments[seg].valid,
                  "default segment invalid: " +
                      std::string(kBasePoseSegmentNames[seg]));
            for (int i = 0; i < 3; ++i)
                Check(std::isnan(sample.segments[seg].position_m[i]),
                      "default position NaN");
            for (int i = 0; i < 4; ++i)
                Check(std::isnan(sample.segments[seg].quat_xyzw[i]),
                      "default quaternion NaN");
        }
        Check(std::isnan(BasePoseAgeS(sample, 12.0)),
              "age of the never-a-sample state is NaN");
    }

    // --- 2. Snapshot→sample mapping
    {
        ViconSnapshot snapshot;
        snapshot.frame_number = 668410;
        snapshot.host_time_s = 41.5;
        snapshot.latency_total_s = 0.012;
        snapshot.segments.push_back(MakeSegment("Mount", 1.0, true));
        snapshot.segments.push_back(MakeSegment("LeftBase", 2.0, true));
        snapshot.segments.push_back(MakeSegment("LeftEE", 3.0, false));
        snapshot.segments.push_back(MakeSegment("SomethingElse", 9.0, true));
        // RightBase and RightEE deliberately absent.

        const BasePoseSample sample = ToBasePoseSample(snapshot, 7);
        Check(sample.sequence == 7, "sequence carried through");
        Check(sample.vicon_frame_number == 668410, "frame number carried");
        Check(sample.t_receive_s == 41.5, "receive time carried");
        Check(sample.latency_reported_s == 0.012, "latency carried");

        Check(sample.segments[kBasePoseMount].valid, "Mount valid");
        Check(sample.segments[kBasePoseMount].position_m[0] == 1.0 &&
                  sample.segments[kBasePoseMount].position_m[2] == 1.2,
              "Mount position copied in metres");
        Check(sample.segments[kBasePoseMount].quat_xyzw[3] == 1.0 &&
                  sample.segments[kBasePoseMount].quat_xyzw[0] == 0.0,
              "Mount quaternion stored xyzw (w last)");

        Check(sample.segments[kBasePoseLeftBase].valid, "LeftBase valid");
        Check(!sample.segments[kBasePoseLeftEE].valid,
              "occluded LeftEE stays invalid");
        Check(std::isnan(sample.segments[kBasePoseLeftEE].position_m[0]),
              "occluded LeftEE position stays NaN");
        Check(!sample.segments[kBasePoseRightBase].valid,
              "absent RightBase stays invalid");
        Check(!sample.segments[kBasePoseRightEE].valid,
              "absent RightEE stays invalid");

        Check(BasePoseAgeS(sample, 41.6) > 0.099 &&
                  BasePoseAgeS(sample, 41.6) < 0.101,
              "age = now - receive time");
    }

    // --- 3. Slot: publish/read round-trip and absence before first publish
    {
        BasePoseSlot slot;
        BasePoseSample out;
        Check(!slot.ReadLatest(out),
              "reading before any publish reports absence");

        ViconSnapshot snapshot;
        snapshot.frame_number = 1;
        snapshot.host_time_s = 1.0;
        snapshot.segments.push_back(MakeSegment("Mount", 5.0, true));
        slot.Publish(ToBasePoseSample(snapshot, 1));

        Check(slot.ReadLatest(out), "read after publish succeeds");
        Check(out.sequence == 1, "published sequence visible");
        Check(out.segments[kBasePoseMount].position_m[0] == 5.0,
              "published pose visible");

        // Re-read without a new publish: same sample again (ZOH semantics —
        // the caller detects reuse by the unchanged sequence).
        BasePoseSample again;
        Check(slot.ReadLatest(again) && again.sequence == 1,
              "re-read without new publish returns the same sequence");
    }

    // --- 4. Slot under concurrent writes: never torn, never backwards.
    // The writer stamps every position field of every segment with the
    // sequence number, so any mixed-generation read is detectable.
    {
        BasePoseSlot slot;
        std::atomic<bool> stop{false};
        std::thread writer([&] {
            for (std::uint64_t seq = 1; seq <= 200000 && !stop; ++seq) {
                BasePoseSample sample;
                sample.sequence = seq;
                sample.vicon_frame_number =
                    static_cast<std::uint32_t>(seq);
                sample.t_receive_s = static_cast<double>(seq);
                sample.latency_reported_s = 0.0;
                for (int seg = 0; seg < kBasePoseSegmentCount; ++seg) {
                    sample.segments[seg].valid = true;
                    for (int i = 0; i < 3; ++i)
                        sample.segments[seg].position_m[i] =
                            static_cast<double>(seq);
                    for (int i = 0; i < 4; ++i)
                        sample.segments[seg].quat_xyzw[i] =
                            static_cast<double>(seq);
                }
                slot.Publish(sample);
            }
        });

        std::uint64_t last_seq = 0;
        bool torn = false;
        bool backwards = false;
        for (int reads = 0; reads < 200000; ++reads) {
            BasePoseSample sample;
            if (!slot.ReadLatest(sample))
                continue;
            if (sample.sequence < last_seq)
                backwards = true;
            last_seq = sample.sequence;
            const double expected = static_cast<double>(sample.sequence);
            for (int seg = 0; seg < kBasePoseSegmentCount && !torn; ++seg)
                for (int i = 0; i < 3; ++i)
                    if (sample.segments[seg].position_m[i] != expected)
                        torn = true;
        }
        stop = true;
        writer.join();
        Check(!torn, "concurrent reads are never torn");
        Check(!backwards, "sequence never runs backwards");
        Check(last_seq > 0, "reader observed published samples");
    }

    if (failures == 0) {
        std::cout << "test_base_pose: all checks passed\n";
        return 0;
    }
    std::cout << "test_base_pose: " << failures << " check(s) failed\n";
    return 1;
}
