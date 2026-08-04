//
// Targets — desired positions, their SPSC mailbox, and the reference source.
// Never talks to the robot; does no control math.
//

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "State.h"

// One base_link target. Every live target is position-only, preserving the
// controller's takeover orientation.
struct PoseTarget {
    Eigen::Vector3d p_desired;               // metres, right-arm base frame
    std::optional<Eigen::Matrix3d> rotation; // always nullopt at runtime
};

// Rotation matrix from roll/pitch/yaw RADIANS, composed R = Rz·Ry·Rx.
Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw);

// Parse public runtime input: exactly three finite x y z coordinates in
// metres, in base_link. The conservative sphere is 0.852 m about the base
// origin. The compiled fixed target remains valid within 1e-12.
std::optional<PoseTarget> ParsePoseTarget(const std::string& line,
                                          std::string& error);

// Exactly eight entries, with the stdin thread as sole producer and the
// 500 Hz source as sole consumer. Release/acquire publication means the
// source never locks, allocates, or performs I/O in Get.
class PoseTargetMailbox
{
public:
    static constexpr std::size_t kCapacity = 8;

    bool Enqueue(const PoseTarget& target);
    std::optional<PoseTarget> TryDequeue();

private:
    std::array<PoseTarget, kCapacity> entries_{};
    alignas(64) std::atomic<std::uint64_t> read_index_{0};
    alignas(64) std::atomic<std::uint64_t> write_index_{0};
};

// Thread body: reads stdin lines, validates and enqueues them. Terminal I/O
// remains outside the control loop; polling lets teardown join promptly.
void RunPoseTargetInput(PoseTargetMailbox& mailbox,
                        const std::atomic<bool>& stop);

// Begins with the compiled fixed target (sequence 0). It holds the active
// target until Runner reports an arrival edge; the next Get consumes exactly
// one queued target, if any, and increments the sequence. If empty, it stays
// ready for a target later enqueued by the producer.
class PoseTargetSource
{
public:
    PoseTargetSource(PoseTarget fixed_target, PoseTargetMailbox& mailbox);

    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status);
    void OnArrived();

private:
    PoseTargetMailbox& mailbox_;
    PoseTarget active_target_;
    std::uint64_t sequence_ = 0;
    bool ready_for_next_ = false;
};
