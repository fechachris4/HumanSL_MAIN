//
// Targets — implementations for Targets.h.
//

#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/stat.h>
#include <thread>
#include <tuple>
#include <unistd.h>

#include "Targets.h"

Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw)
{
    // R = Rz(yaw) · Ry(pitch) · Rx(roll) — the simulation's convention
    // (msc_project controller/transforms.py rotation_from_rpy).
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

std::optional<PoseTarget> ParsePoseTarget(const std::string& line, std::string& error)
{
    std::istringstream in(line);
    std::array<double, 6> values{};
    int count = 0;
    while (count < 6 && in >> values[count]) {
        if (!std::isfinite(values[count])) {
            error = "number " + std::to_string(count + 1) + " is not finite";
            return std::nullopt;
        }
        ++count;
    }
    in.clear(); // a failed 4th/7th extraction must not hide trailing text
    std::string trailing;
    if (in >> trailing) {
        error = "expected 3 (x y z) or 6 (x y z roll pitch yaw) numbers";
        return std::nullopt;
    }
    if (count != 3 && count != 6) {
        error = "expected 3 numbers (x y z, meters, right-arm base frame) or 6 "
                "(x y z meters + roll pitch yaw, radians)";
        return std::nullopt;
    }

    PoseTarget target;
    target.p_desired = Eigen::Vector3d(values[0], values[1], values[2]);
    if (count == 6)
        target.rotation = RotationFromRpy(values[3], values[4], values[5]);
    return target;
}

void PoseTargetStore::Store(const Eigen::Vector3d& p_desired,
                            const Eigen::Matrix3d& rotation)
{
    std::lock_guard<std::mutex> lock(mutex_);
    p_desired_ = p_desired;
    rotation_ = rotation;
    ++sequence_;
}

void PoseTargetStore::StorePosition(const Eigen::Vector3d& p_desired)
{
    std::lock_guard<std::mutex> lock(mutex_);
    p_desired_ = p_desired;
    ++sequence_;
}

PoseTargetStore::Snapshot PoseTargetStore::Get() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return Snapshot{p_desired_, rotation_, sequence_};
}

void RunPoseTargetInput(PoseTargetStore& store, const std::atomic<bool>& stop)
{
    std::string line;
    while (!stop) {
        // Wait up to 100 ms for stdin to become readable, then re-check
        // `stop`. getline only runs when a line is actually available.
        struct pollfd stdin_fd = {STDIN_FILENO, POLLIN, 0};
        int ready = poll(&stdin_fd, 1, 100);
        if (ready <= 0)
            continue;
        if (!std::getline(std::cin, line))
            break; // stdin closed (EOF) — no more targets can arrive
        if (line.empty())
            continue;

        std::string error;
        std::optional<PoseTarget> target = ParsePoseTarget(line, error);
        if (!target) {
            std::cout << "target rejected: " << error << "\n";
            continue;
        }
        if (target->rotation) {
            store.Store(target->p_desired, *target->rotation);
            std::cout << "desired pose accepted: " << target->p_desired.x() << " "
                      << target->p_desired.y() << " " << target->p_desired.z()
                      << " (m) + orientation (right-arm base frame)\n";
        } else {
            store.StorePosition(target->p_desired);
            std::cout << "desired position accepted: " << target->p_desired.x() << " "
                      << target->p_desired.y() << " " << target->p_desired.z()
                      << " (m, right-arm base frame; "
                         "orientation target unchanged)\n";
        }
    }
}

std::optional<std::string> FirstTargetLine(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        return std::nullopt;
    std::string line;
    while (std::getline(file, line)) {
        const std::size_t first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#')
            continue;
        return line;
    }
    return std::nullopt;
}

namespace
{
    // What "the file changed" means: a different inode (editors that save
    // by atomic rename), size, or mtime. Nullopt while the file is absent.
    using FileSignature = std::tuple<ino_t, off_t, time_t>;

    std::optional<FileSignature> SignatureOf(const std::string& path)
    {
        struct stat status {};
        if (::stat(path.c_str(), &status) != 0)
            return std::nullopt;
        return FileSignature{status.st_ino, status.st_size, status.st_mtime};
    }
} // namespace

void RunPoseTargetFileInput(PoseTargetStore& store, const std::string& path,
                            const std::atomic<bool>& stop)
{
    // Whatever the file says NOW is ignored — only a change made during
    // the session becomes a target (see the header comment).
    std::optional<FileSignature> last_seen = SignatureOf(path);

    while (!stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const std::optional<FileSignature> current = SignatureOf(path);
        if (current == last_seen)
            continue;
        last_seen = current;
        if (!current)
            continue; // file removed; nothing to apply

        const std::optional<std::string> line = FirstTargetLine(path);
        if (!line) {
            std::cout << "target file " << path
                      << ": no target line (empty or comments only)\n";
            continue;
        }
        std::string error;
        const std::optional<PoseTarget> target = ParsePoseTarget(*line, error);
        if (!target) {
            std::cout << "target file " << path << " rejected: " << error << "\n";
            continue;
        }
        if (target->rotation) {
            store.Store(target->p_desired, *target->rotation);
            std::cout << "target file " << path << ": pose accepted "
                      << target->p_desired.x() << " " << target->p_desired.y() << " "
                      << target->p_desired.z()
                      << " (m) + orientation (right-arm base frame)\n";
        } else {
            store.StorePosition(target->p_desired);
            std::cout << "target file " << path << ": position accepted "
                      << target->p_desired.x() << " " << target->p_desired.y() << " "
                      << target->p_desired.z()
                      << " (m, right-arm base frame; "
                         "orientation target unchanged)\n";
        }
    }
}

// ---------------------------------------------------------------
// PoseTargetSource
// ---------------------------------------------------------------

PoseTargetSource::PoseTargetSource(
    const PoseTargetStore& store,
    std::optional<PoseTarget> initial_target)
    : store_(store), initial_target_(std::move(initial_target))
{}

void PoseTargetSource::Reset(const RobotState& /*state*/)
{
    // Anything stored before takeover is discarded (except the deliberate
    // initial target, which Get serves below).
    baseline_sequence_ = store_.Get().sequence;
}

Reference PoseTargetSource::Get(const RobotState& /*state*/, double /*dt_s*/,
                                ControllerStatus& /*status*/)
{
    const PoseTargetStore::Snapshot snapshot = store_.Get();
    Reference reference;
    // Operator targets are STATIONARY: a typed or compiled pose is a place
    // to be, not a motion, so the reference twist stays zero and the Kd
    // term stays pure damping. A source that moves its target (an
    // orientation policy, a Cartesian path) fills the twist instead.
    if (snapshot.sequence != baseline_sequence_)
        reference.pose = PoseReference{snapshot.p_desired, snapshot.rotation,
                                       Twist{}, snapshot.sequence};
    else if (initial_target_)
        // Sequence 0 never collides with a live store sequence (those
        // continue from the takeover baseline), so the arrival notice
        // arms exactly once for the initial target too. An empty rotation
        // here means the controller keeps the takeover orientation.
        reference.pose = PoseReference{initial_target_->p_desired,
                                       initial_target_->rotation, Twist{}, 0};
    return reference;
}

