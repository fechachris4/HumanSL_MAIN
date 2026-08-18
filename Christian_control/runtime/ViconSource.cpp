//
// ViconSource, SDK implementation — built only when third_party/vicon_api
// exists (see the VICON_SOURCE_FILES block in CMakeLists.txt).
//
// One thread, one job: keep the freshest validated world-pose sample in
// the slot. Reuses the vicon project's pieces rather than duplicating
// them: ViconInterface (SDK wrapper, millimetres) and BuildSnapshot
// (validation + mm→m + quaternion checks — the same code path the vicon
// tests already pin down). The thread tolerates every failure mode by
// retrying: no Vicon on the network, Nexus stopped, connection lost
// mid-run. The controller never waits on any of it.
//

#include "ViconSource.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "BasePose.h"
#include "Config.h"
#include "MountTwistEstimator.h"
#include "SnapshotBuilder.h"   // ../tracking/src — validation + mm→m
#include "ViconInterface.h"    // ../tracking/src — SDK wrapper

const char* const kViconSourceBuildMode = "sdk";
const char* const kViconSourceHost = "192.168.128.206:801";

namespace
{

double SteadyNowS()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class SdkViconSource final : public ViconSource
{
public:
    explicit SdkViconSource(BasePoseSlot& slot)
        : slot_(slot), thread_([this] { Run(); })
    {
    }

    ~SdkViconSource() override
    {
        stop_ = true;
        // Worst joins: one blocking getFrame (~one Vicon frame) or one
        // retry sleep slice (50 ms). Nothing here can wedge the shutdown.
        thread_.join();
    }

private:
    void Run()
    {
        ViconInterface vicon;
        MountTwistEstimator mount_twist_estimator(
            config::kViconMountTwistFilterTauS,
            config::kViconMountTwistResetGapS);
        std::uint64_t sequence = 0;
        int last_frame_number = -1;
        bool was_connected = false;

        while (!stop_) {
            if (!vicon.isConnected()) {
                if (was_connected) {
                    std::cout << "[vicon] connection lost — retrying "
                              << kViconSourceHost << "\n";
                    was_connected = false;
                }
                if (!vicon.connect(kViconSourceHost)) {
                    // Unreachable is normal (lab network absent): retry
                    // every second, in slices so stop stays responsive.
                    for (int i = 0; i < 20 && !stop_; ++i)
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(50));
                    continue;
                }
                std::cout << "[vicon] connected to " << kViconSourceHost
                          << " (" << vicon.getFrameRate() << " Hz server)\n";
                was_connected = true;
                last_frame_number = -1;
            }

            if (!vicon.getFrame()) {
                vicon.disconnect(); // reconnect path prints and retries
                continue;
            }

            const int frame_number = vicon.getFrameNumber();
            if (frame_number == last_frame_number) {
                // Same frame served again (ClientPull semantics): not a
                // new sample — publishing it would inflate the sequence
                // and fake freshness. Wait out the gap.
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            last_frame_number = frame_number;

            // Markers deliberately not read: BasePoseSample carries only
            // the five segment poses; skipping ~22 marker reads keeps the
            // per-frame work small. BuildSnapshot accepts the empty list.
            const std::vector<MarkerData> no_markers;
            const ViconSnapshot snapshot = BuildSnapshot(
                static_cast<unsigned int>(frame_number), SteadyNowS(),
                vicon.getFrameRate(), vicon.getLatencyTotal(), no_markers,
                vicon.getSegmentPoses());
            const MountTwistEstimate mount_twist =
                mount_twist_estimator.Update(snapshot);
            slot_.Publish(
                ToBasePoseSample(snapshot, ++sequence, mount_twist));
        }
        vicon.disconnect();
    }

    BasePoseSlot& slot_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

} // namespace

std::unique_ptr<ViconSource> StartViconSource(BasePoseSlot& slot)
{
    return std::make_unique<SdkViconSource>(slot);
}
