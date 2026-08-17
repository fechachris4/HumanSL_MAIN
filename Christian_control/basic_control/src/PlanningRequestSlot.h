#pragma once

#include <atomic>

#include "PlanningRequest.h"

// SPSC latest-value triple buffer. The 500 Hz producer and non-real-time
// writer each own a buffer; atomic exchange transfers the middle buffer.
class PlanningRequestSlot {
public:
    void Publish(const PlanningRequest& request) {
        buffers_[write_index_] = request;
        const int previous = middle_.exchange(write_index_ | kNewBit,
                                              std::memory_order_acq_rel);
        write_index_ = previous & kIndexMask;
    }

    bool TakeLatest(PlanningRequest& request) {
        if (!(middle_.load(std::memory_order_relaxed) & kNewBit)) return false;
        const int previous = middle_.exchange(read_index_,
                                              std::memory_order_acq_rel);
        read_index_ = previous & kIndexMask;
        request = buffers_[read_index_];
        return true;
    }

private:
    static constexpr int kIndexMask = 0x3;
    static constexpr int kNewBit = 0x4;
    PlanningRequest buffers_[3];
    std::atomic<int> middle_{1};
    int write_index_ = 0;
    int read_index_ = 2;
};
