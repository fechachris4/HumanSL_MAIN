#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>

#include "GoalCommand.h"
#include "LatestValueSlot.h"

inline constexpr const char* kRightGoalSocketPath =
    "/tmp/humansl-goal-right.sock";
inline constexpr const char* kLeftGoalSocketPath =
    "/tmp/humansl-goal-left.sock";

using GoalCommandSlot = LatestValueSlot<GoalCommand>;

std::optional<std::string> ParseGoalCommandLine(
    const std::string& line, std::uint64_t command_id, GoalCommand& command);

class GoalSocket {
public:
    GoalSocket(GoalCommandSlot& slot, const std::string& path,
               const std::string& log_prefix);
    ~GoalSocket();

    GoalSocket(const GoalSocket&) = delete;
    GoalSocket& operator=(const GoalSocket&) = delete;

private:
    void Run();

    GoalCommandSlot& slot_;
    std::string path_;
    std::string log_prefix_;
    int socket_fd_ = -1;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};
