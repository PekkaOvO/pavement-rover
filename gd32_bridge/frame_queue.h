#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>

namespace gd32_bridge {

// Thread-safe bounded queue of complete frame blobs.
// Producer (UartReader): push() after CRC validation.
// Consumer (TcpSender): pop() blocks until data available.
class FrameQueue {
public:
    explicit FrameQueue(size_t max_frames = 32)
        : max_frames_(max_frames) {}

    // Push a complete frame. When full, drops the oldest frame to make room
    // so the TCP sender always has the latest data.
    bool push(const uint8_t *data, size_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_frames_)
            queue_.pop_front();  // drop oldest to keep pipeline fresh
        queue_.emplace_back(data, data + len);
        cv_.notify_one();
        return true;
    }

    // Blocking pop. Returns false on shutdown signal (empty + shutdown).
    bool pop(std::vector<uint8_t> &frame) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || shutdown_; });
        if (queue_.empty())
            return false;
        frame = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    // Non-blocking pop. Returns false if empty.
    bool try_pop(std::vector<uint8_t> &frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return false;
        frame = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    // Signal shutdown to unblock any waiting pop().
    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        cv_.notify_all();
    }

private:
    std::deque<std::vector<uint8_t>> queue_;
    size_t max_frames_;
    bool shutdown_ = false;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
};

} // namespace gd32_bridge
