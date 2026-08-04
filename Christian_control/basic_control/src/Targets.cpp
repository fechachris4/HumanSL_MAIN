//
// Targets — implementations for Targets.h.
//

#include <cmath>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <utility>

#include <unistd.h>

#include "Config.h"
#include "Targets.h"

Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw)
{
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

std::optional<PoseTarget> ParsePoseTarget(const std::string& line, std::string& error)
{
    std::istringstream input(line);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!(input >> x >> y >> z)) {
        error = "expected exactly three x y z coordinates in metres";
        return std::nullopt;
    }
    std::string trailing;
    if (input >> trailing) {
        error = "expected exactly three x y z coordinates in metres";
        return std::nullopt;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        error = "all target coordinates must be finite";
        return std::nullopt;
    }

    const Eigen::Vector3d position(x, y, z);
    const Eigen::Vector3d fixed_target(config::kFixedTargetM[0],
                                       config::kFixedTargetM[1],
                                       config::kFixedTargetM[2]);
    const double conservative_reach_m =
        config::kReachRadiusM - config::kReachMarginM;
    if ((position - fixed_target).norm() > 1e-12 &&
        position.norm() > conservative_reach_m) {
        error = "target is outside the 0.852 m conservative reach sphere";
        return std::nullopt;
    }

    PoseTarget target;
    target.p_desired = position;
    return target;
}

bool PoseTargetMailbox::Enqueue(const PoseTarget& target)
{
    const std::uint64_t write = write_index_.load(std::memory_order_relaxed);
    const std::uint64_t read = read_index_.load(std::memory_order_acquire);
    if (write - read >= kCapacity)
        return false;

    entries_[write % kCapacity] = target;
    write_index_.store(write + 1, std::memory_order_release);
    return true;
}

std::optional<PoseTarget> PoseTargetMailbox::TryDequeue()
{
    const std::uint64_t read = read_index_.load(std::memory_order_relaxed);
    const std::uint64_t write = write_index_.load(std::memory_order_acquire);
    if (read == write)
        return std::nullopt;

    PoseTarget target = entries_[read % kCapacity];
    read_index_.store(read + 1, std::memory_order_release);
    return target;
}

void RunPoseTargetInput(PoseTargetMailbox& mailbox, const std::atomic<bool>& stop)
{
    std::string line;
    while (!stop.load(std::memory_order_relaxed)) {
        pollfd stdin_fd{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&stdin_fd, 1, 100);
        if (ready <= 0)
            continue;
        if (!std::getline(std::cin, line))
            break;
        if (line.empty())
            continue;

        std::string error;
        const std::optional<PoseTarget> target = ParsePoseTarget(line, error);
        if (!target) {
            std::cout << "target rejected: " << error << "\n";
        } else if (!mailbox.Enqueue(*target)) {
            std::cout << "target rejected: queue is full (8 pending targets)\n";
        } else {
            std::cout << "target queued: " << target->p_desired.x() << " "
                      << target->p_desired.y() << " " << target->p_desired.z()
                      << " m (base_link, position-only)\n";
        }
    }
}

PoseTargetSource::PoseTargetSource(PoseTarget fixed_target,
                                   PoseTargetMailbox& mailbox)
    : mailbox_(mailbox), active_target_(std::move(fixed_target))
{
    active_target_.rotation.reset();
}

Reference PoseTargetSource::Get(const RobotState& /*state*/, double /*dt_s*/,
                                ControllerStatus& /*status*/)
{
    if (ready_for_next_) {
        if (const std::optional<PoseTarget> next = mailbox_.TryDequeue()) {
            active_target_ = *next;
            active_target_.rotation.reset();
            ++sequence_;
            ready_for_next_ = false;
        }
    }

    Reference reference;
    reference.pose = PoseReference{active_target_.p_desired, std::nullopt,
                                   Twist{}, sequence_};
    return reference;
}

void PoseTargetSource::OnArrived()
{
    ready_for_next_ = true;
}
