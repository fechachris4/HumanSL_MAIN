#pragma once
// Thread-safe setpoint queue: the laptop mirror of the real JointTrajectory
// deque + replan seam. A controller replace()s the active horizon; the backend
// next()s one setpoint per tick. When only one remains it is held (not popped),
// matching the real execution loop's "hold last waypoint" behaviour.
#include <deque>
#include <mutex>
#include <optional>
#include "joint_types.h"

namespace jmpc {

class SetpointQueue {
public:
    // Atomically swap the active horizon (receding-horizon replan).
    void replace(const std::deque<JointSetpoint>& horizon) {
        std::lock_guard<std::mutex> lk(m_);
        q_ = horizon;
    }

    // Pop the next setpoint; hold (do not remove) the last one. nullopt if never populated.
    std::optional<JointSetpoint> next() {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.empty()) return std::nullopt;
        JointSetpoint front = q_.front();
        if (q_.size() > 1) q_.pop_front();
        return front;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }

private:
    mutable std::mutex m_;
    std::deque<JointSetpoint> q_;
};

}  // namespace jmpc
