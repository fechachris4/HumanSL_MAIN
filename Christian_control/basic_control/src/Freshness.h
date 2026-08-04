//
// Freshness — pure per-actuator cyclic acknowledgement tracking.
//
// Acknowledgement IDs come from completed cyclic feedback replies. They are
// evidence that the downstream cyclic path advances, not evidence of motion
// or physical setpoint acceptance.
//

#pragma once

#include <array>
#include <cstdint>
#include <optional>

class FeedbackFreshnessMonitor
{
public:
    void Update(const std::array<std::uint32_t, 7>& command_ack)
    {
        // The first frame has nothing to compare against: seed and report
        // zero unchanged cycles for every Kortex actuator (joints 1..7).
        if (!initialized_)
        {
            previous_ = command_ack;
            initialized_ = true;
            unchanged_cycles_.fill(0);
            return;
        }
        for (int i = 0; i < 7; ++i)
        {
            if (command_ack[i] == previous_[i])
                ++unchanged_cycles_[i];
            else
                unchanged_cycles_[i] = 0;
            previous_[i] = command_ack[i];
        }
    }

    const std::array<int, 7>& unchanged_cycles() const { return unchanged_cycles_; }

private:
    bool initialized_ = false;
    std::array<std::uint32_t, 7> previous_{};
    std::array<int, 7> unchanged_cycles_{};
};

// Returns the first Kortex actuator index whose acknowledgement has remained
// unchanged for the configured number of completed cyclic replies. A
// non-positive threshold disables this guard.
inline std::optional<int> StaleAcknowledgementJoint(
    const std::array<int, 7>& unchanged_cycles, int stop_cycles)
{
    if (stop_cycles <= 0)
        return std::nullopt;
    for (int i = 0; i < 7; ++i)
        if (unchanged_cycles[i] >= stop_cycles)
            return i;
    return std::nullopt;
}
