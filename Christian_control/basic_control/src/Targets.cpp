//
// Targets — implementations for Targets.h.
//

#include <array>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <utility>

#include <unistd.h>

#include "Config.h"
#include "Targets.h"

namespace
{
    constexpr int kInputPollTimeoutMs = 50;
    constexpr std::size_t kInputReadBytes = 256;
    constexpr std::size_t kMaxInputLineBytes = 512;

    void ProcessPoseTargetLine(PoseTargetMailbox& mailbox, const std::string& line)
    {
        if (line.empty())
            return;

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

    void ConsumeInputBytes(PoseTargetMailbox& mailbox, const char* bytes,
                           std::size_t count, std::string& pending_line,
                           bool& discarding_overlong_line)
    {
        for (std::size_t i = 0; i < count; ++i) {
            const char character = bytes[i];
            if (character == '\n') {
                if (discarding_overlong_line) {
                    std::cout << "target rejected: input line exceeds 512 bytes\n";
                    discarding_overlong_line = false;
                } else {
                    if (!pending_line.empty() && pending_line.back() == '\r')
                        pending_line.pop_back();
                    ProcessPoseTargetLine(mailbox, pending_line);
                }
                pending_line.clear();
            } else if (!discarding_overlong_line) {
                if (pending_line.size() == kMaxInputLineBytes) {
                    pending_line.clear();
                    discarding_overlong_line = true;
                } else {
                    pending_line.push_back(character);
                }
            }
        }
    }
} // namespace

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
    RunPoseTargetInputFromFd(mailbox, stop, STDIN_FILENO);
}

void RunPoseTargetInputFromFd(PoseTargetMailbox& mailbox,
                              const std::atomic<bool>& stop, int input_fd)
{
    std::array<char, kInputReadBytes> bytes{};
    std::string pending_line;
    bool discarding_overlong_line = false;

    while (!stop.load(std::memory_order_relaxed)) {
        pollfd input{input_fd, POLLIN, 0};
        const int ready = poll(&input, 1, kInputPollTimeoutMs);
        if (ready == 0)
            continue;
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (stop.load(std::memory_order_relaxed))
            break;
        if (input.revents & POLLNVAL)
            break;
        if (!(input.revents & (POLLIN | POLLHUP))) {
            if (input.revents & POLLERR)
                break;
            continue;
        }

        const ssize_t read_count = read(input_fd, bytes.data(), bytes.size());
        if (read_count > 0) {
            ConsumeInputBytes(mailbox, bytes.data(),
                              static_cast<std::size_t>(read_count), pending_line,
                              discarding_overlong_line);
            continue;
        }
        if (read_count < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            break;
        }
        // EOF terminates a final line even without '\n'; a stop deliberately
        // drops an incomplete line instead of enqueueing during teardown.
        if (!stop.load(std::memory_order_relaxed)) {
            if (discarding_overlong_line)
                std::cout << "target rejected: input line exceeds 512 bytes\n";
            else
                ProcessPoseTargetLine(mailbox, pending_line);
        }
        break;
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

void NotifyPoseTargetSourceOnArrivalEdge(PoseTargetSource& source,
                                         const ControllerStatus& status)
{
    if (status.arrived_edge)
        source.OnArrived();
}
