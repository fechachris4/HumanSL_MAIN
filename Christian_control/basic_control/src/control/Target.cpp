//
// Target: the operator's desired end-effector position — parsing,
// latest-value store, and the stdin input thread.
//

#include "control/Target.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include <poll.h>
#include <unistd.h>

std::optional<Eigen::Vector3d> ParseCartesianTarget(const std::string& line,
                                                    std::string& error)
{
    std::istringstream in(line);
    Eigen::Vector3d p_desired;
    for (int i = 0; i < 3; ++i) {
        if (!(in >> p_desired[i])) {
            error = "expected 3 numbers (desired x y z, meters, base frame)";
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
                  << " " << p_desired->z() << " (m, base frame)\n";
    }
}
