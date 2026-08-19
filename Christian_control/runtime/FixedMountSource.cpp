#include "FixedMountSource.h"

#include <chrono>

#include "BasePose.h"

FixedMountSource::FixedMountSource(BasePoseSlot& slot,
                                   const Eigen::Isometry3d& world_T_mount)
    : slot_(slot),
      position_m_(world_T_mount.translation()),
      rotation_(Eigen::Quaterniond(world_T_mount.linear()).normalized()),
      thread_(&FixedMountSource::Run, this)
{
}

FixedMountSource::~FixedMountSource()
{
    stop_.store(true, std::memory_order_relaxed);
    if (thread_.joinable())
        thread_.join();
}

void FixedMountSource::Run()
{
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / kPublishRateHz));
    std::uint64_t sequence = 0;
    auto next_publish = clock::now();
    while (!stop_.load(std::memory_order_relaxed)) {
        BasePoseSample sample; // NaN/invalid everywhere not set below
        sample.sequence = ++sequence;
        // Same clock convention as ViconSource: steady seconds, so
        // BasePoseAgeS() against the loop's steady clock is exact.
        sample.t_receive_s = std::chrono::duration<double>(
                                 clock::now().time_since_epoch())
                                 .count();
        sample.frame_rate_hz = kPublishRateHz;
        // vicon_frame_number stays 0 and latency stays NaN: no Vicon frame
        // exists. A fixed mount does not move, so its twist is measured
        // zero, not unknown.
        for (int i = 0; i < 3; ++i) {
            sample.mount_linear_world_m_s[i] = 0.0;
            sample.mount_angular_world_rad_s[i] = 0.0;
        }
        sample.mount_twist_valid = true;
        BasePoseSegmentPose& mount = sample.segments[kBasePoseMount];
        for (int i = 0; i < 3; ++i)
            mount.position_m[i] = position_m_[i];
        mount.quat_xyzw[0] = rotation_.x();
        mount.quat_xyzw[1] = rotation_.y();
        mount.quat_xyzw[2] = rotation_.z();
        mount.quat_xyzw[3] = rotation_.w();
        mount.valid = true;
        slot_.Publish(sample);

        next_publish += period;
        std::this_thread::sleep_until(next_publish);
    }
}
