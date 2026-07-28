//
// Target: the operator's desired end-effector position — parsing,
// latest-value store, and the stdin input thread.
//

#include "control/Target.h"

#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <tuple>

#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

std::optional<Eigen::Vector3d> ParseCartesianTarget(const std::string& line,
                                                    std::string& error)
{
    std::istringstream in(line);
    Eigen::Vector3d p_desired;
    for (int i = 0; i < 3; ++i) {
        if (!(in >> p_desired[i])) {
            error = "expected 3 numbers (desired x y z, meters, "
                    "dual-model world/common mount frame)";
            return std::nullopt;
        }
        if (!std::isfinite(p_desired[i])) {
            error = "coordinate " + std::to_string(i + 1) + " is not finite";
            return std::nullopt;
        }
    }
    std::string trailing;
    if (in >> trailing) {
        error = "expected exactly 3 numbers, got more";
        return std::nullopt;
    }
    return p_desired;
}

void TargetStore::Store(const Eigen::Vector3d& p_desired)
{
    std::lock_guard<std::mutex> lock(mutex_);
    p_desired_ = p_desired;
    ++sequence_;
}

TargetStore::Snapshot TargetStore::Get() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return Snapshot{p_desired_, sequence_};
}

void RunTargetInput(TargetStore& store, const std::atomic<bool>& stop)
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
        std::optional<Eigen::Vector3d> p_desired = ParseCartesianTarget(line, error);
        if (!p_desired) {
            std::cout << "target rejected: " << error << "\n";
            continue;
        }
        store.Store(*p_desired);
        std::cout << "desired position accepted: " << p_desired->x() << " " << p_desired->y()
                  << " " << p_desired->z()
                  << " (m, dual-model world/common mount frame)\n";
    }
}

// --- Pose targets (the reactive-pose controller) ---------------------------

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
        error = "expected 3 numbers (x y z, meters, dual-model world/common "
                "mount frame) or 6 "
                "(x y z meters + roll pitch yaw, radians)";
        return std::nullopt;
    }

    PoseTarget target;
    target.p_desired = Eigen::Vector3d(values[0], values[1], values[2]);
    if (count == 6) {
        // R = Rz(yaw) · Ry(pitch) · Rx(roll) — the simulation's convention
        // (msc_project controller/transforms.py rotation_from_rpy).
        const double roll = values[3];
        const double pitch = values[4];
        const double yaw = values[5];
        target.rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                              .toRotationMatrix();
    }
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
                      << " (m) + orientation (dual-model world/common mount frame)\n";
        } else {
            store.StorePosition(target->p_desired);
            std::cout << "desired position accepted: " << target->p_desired.x() << " "
                      << target->p_desired.y() << " " << target->p_desired.z()
                      << " (m, dual-model world/common mount frame; "
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
                      << " (m) + orientation (dual-model world/common mount frame)\n";
        } else {
            store.StorePosition(target->p_desired);
            std::cout << "target file " << path << ": position accepted "
                      << target->p_desired.x() << " " << target->p_desired.y() << " "
                      << target->p_desired.z()
                      << " (m, dual-model world/common mount frame; "
                         "orientation target unchanged)\n";
        }
    }
}
