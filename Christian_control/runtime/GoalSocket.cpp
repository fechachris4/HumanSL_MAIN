#include "GoalSocket.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

bool NoTrailingToken(std::istringstream& input)
{
    std::string extra;
    return !(input >> extra);
}

} // namespace

std::optional<std::string> ParseGoalCommandLine(
    const std::string& line, std::uint64_t command_id, GoalCommand& command)
{
    std::istringstream input(line);
    std::string kind;
    if (!(input >> kind))
        return "empty goal command";

    GoalCommand parsed;
    parsed.command_id = command_id;
    std::string orientation;
    if (kind == "POINT") {
        parsed.kind = GoalKind::kPoint;
        if (!(input >> parsed.point_m.x() >> parsed.point_m.y() >>
              parsed.point_m.z() >> orientation))
            return "POINT expects x y z and INHERIT or FIXED r p y";
    } else if (kind == "CIRCLE") {
        parsed.kind = GoalKind::kCircle;
        if (!(input >> parsed.circle_centre_m.x() >>
              parsed.circle_centre_m.y() >> parsed.circle_centre_m.z() >>
              parsed.circle_radius_m >> parsed.circle_normal.x() >>
              parsed.circle_normal.y() >> parsed.circle_normal.z() >>
              parsed.circle_duration_s >> orientation))
            return "CIRCLE expects centre, radius, normal, duration and orientation";
    } else {
        return "goal command must start with POINT or CIRCLE";
    }

    if (orientation == "INHERIT") {
        parsed.orientation = GoalOrientation::kInherit;
    } else if (orientation == "RADIAL") {
        parsed.orientation = GoalOrientation::kRadialInward;
    } else if (orientation == "FIXED") {
        parsed.orientation = GoalOrientation::kFixedRpy;
        if (!(input >> parsed.fixed_rpy_rad.x() >> parsed.fixed_rpy_rad.y() >>
              parsed.fixed_rpy_rad.z()))
            return "FIXED orientation expects roll pitch yaw in radians";
    } else {
        return "orientation must be INHERIT, FIXED or RADIAL";
    }
    if (!NoTrailingToken(input))
        return "unexpected trailing goal command data";
    if (const std::optional<std::string> error = ValidateGoalCommand(parsed))
        return *error;
    command = parsed;
    return std::nullopt;
}

GoalSocket::GoalSocket(GoalCommandSlot& slot, const std::string& path,
                       const std::string& log_prefix)
    : slot_(slot), path_(path), log_prefix_(log_prefix)
{
    socket_fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (socket_fd_ < 0)
        throw std::runtime_error("goal socket: socket(): " +
                                 std::string(std::strerror(errno)));

    ::unlink(path_.c_str());
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(address.sun_path)) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("goal socket path is too long: " + path_);
    }
    std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1);
    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) != 0) {
        const std::string error = std::strerror(errno);
        ::close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("goal socket: cannot bind " + path_ + ": " +
                                 error);
    }
    thread_ = std::thread(&GoalSocket::Run, this);
}

GoalSocket::~GoalSocket()
{
    stop_.store(true, std::memory_order_relaxed);
    if (thread_.joinable())
        thread_.join();
    if (socket_fd_ >= 0)
        ::close(socket_fd_);
    ::unlink(path_.c_str());
}

void GoalSocket::Run()
{
    std::uint64_t next_id = 1;
    while (!stop_.load(std::memory_order_relaxed)) {
        pollfd descriptor{socket_fd_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, 100);
        if (ready <= 0)
            continue;
        char buffer[512];
        const ssize_t count = ::recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
        if (count <= 0)
            continue;
        buffer[count] = '\0';
        GoalCommand command;
        if (const std::optional<std::string> error =
                ParseGoalCommandLine(buffer, next_id, command)) {
            std::cerr << log_prefix_ << "live goal rejected: " << *error << "\n";
            continue;
        }
        ++next_id;
        slot_.Publish(command);
        std::cout << log_prefix_ << "live goal " << command.command_id
                  << " received: " << buffer << "\n";
    }
}
