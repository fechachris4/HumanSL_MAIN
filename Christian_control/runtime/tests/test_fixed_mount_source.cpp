//
// Hardware-free test for FixedMountSource: it must fill the same
// BasePoseSample contract ViconSource fills, with a constant Mount pose,
// fresh timestamps, advancing sequence, an identically-zero valid twist,
// and honest absence (NaN/invalid) everywhere it measures nothing.
// No Kortex, no Vicon SDK, no robot.
//

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include "BasePose.h"
#include "FixedMountSource.h"

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

    double NowSteadyS()
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
} // namespace

int main()
{
    BasePoseSlot slot;
    Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
    world_T_mount.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
    // 90 degrees about z.
    world_T_mount.linear() =
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

    {
        FixedMountSource source(slot, world_T_mount);

        // A sample arrives within a publish period or two.
        BasePoseSample sample;
        bool got = false;
        for (int i = 0; i < 200 && !got; ++i) {
            got = slot.ReadLatest(sample) && sample.sequence > 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        Check(got, "a sample is published");

        const BasePoseSegmentPose& mount = sample.segments[kBasePoseMount];
        Check(mount.valid, "the Mount segment is valid");
        Check(std::abs(mount.position_m[0] - 0.1) < 1e-12 &&
                  std::abs(mount.position_m[1] + 0.2) < 1e-12 &&
                  std::abs(mount.position_m[2] - 0.3) < 1e-12,
              "the Mount position is the given translation");
        const Eigen::Quaterniond expected(world_T_mount.linear());
        const Eigen::Quaterniond published(
            mount.quat_xyzw[3], mount.quat_xyzw[0], mount.quat_xyzw[1],
            mount.quat_xyzw[2]);
        Check(std::abs(std::abs(expected.dot(published)) - 1.0) < 1e-9,
              "the Mount quaternion is the given rotation");

        // The freshness contract the controller classifies on: age is small
        // and finite (world_fresh needs age <= kWorldFreshMaxAgeS).
        const double age_s = BasePoseAgeS(sample, NowSteadyS());
        Check(std::isfinite(age_s) && age_s >= 0.0 && age_s < 1.0,
              "the sample's steady-clock age is fresh");

        // A fixed mount's twist is measured zero, not unknown.
        Check(sample.mount_twist_valid, "the mount twist is valid");
        for (int i = 0; i < 3; ++i)
            Check(sample.mount_linear_world_m_s[i] == 0.0 &&
                      sample.mount_angular_world_rad_s[i] == 0.0,
                  "the mount twist is identically zero");

        // Honest absence: no Vicon frame exists, and the four other
        // segments measure nothing.
        Check(sample.vicon_frame_number == 0, "vicon_frame_number stays 0");
        Check(std::isnan(sample.latency_reported_s), "latency stays NaN");
        for (int seg = 0; seg < kBasePoseSegmentCount; ++seg) {
            if (seg == kBasePoseMount)
                continue;
            Check(!sample.segments[seg].valid &&
                      std::isnan(sample.segments[seg].position_m[0]),
                  std::string("segment ") + kBasePoseSegmentNames[seg] +
                      " stays NaN/invalid");
        }

        // The sequence advances: a later read sees a newer publish, so the
        // controller's zero-order-hold age never grows without bound.
        const std::uint64_t first_sequence = sample.sequence;
        bool advanced = false;
        for (int i = 0; i < 200 && !advanced; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            BasePoseSample later;
            advanced = slot.ReadLatest(later) &&
                       later.sequence > first_sequence;
        }
        Check(advanced, "the sequence advances across publishes");
    } // destructor stops and joins the thread — reaching the next line IS the test

    if (failures == 0) {
        std::cout << "all FixedMountSource tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
