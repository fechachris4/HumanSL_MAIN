#pragma once

#include <atomic>

// SPSC latest-value triple buffer. Publishing replaces any unread older
// value; the producer and consumer never touch the same buffer.
template <typename T>
class LatestValueSlot {
public:
    void Publish(const T& value)
    {
        buffers_[write_index_] = value;
        const int previous = middle_.exchange(write_index_ | kNewBit,
                                              std::memory_order_acq_rel);
        write_index_ = previous & kIndexMask;
    }

    bool TakeLatest(T& value)
    {
        if (!(middle_.load(std::memory_order_relaxed) & kNewBit))
            return false;
        const int previous = middle_.exchange(read_index_,
                                              std::memory_order_acq_rel);
        read_index_ = previous & kIndexMask;
        value = buffers_[read_index_];
        return true;
    }

private:
    static constexpr int kIndexMask = 0x3;
    static constexpr int kNewBit = 0x4;
    T buffers_[3]{};
    std::atomic<int> middle_{1};
    int write_index_ = 0;
    int read_index_ = 2;
};
