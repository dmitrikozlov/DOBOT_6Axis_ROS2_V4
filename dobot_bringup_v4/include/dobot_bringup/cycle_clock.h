/**
 * Cycle tracking: ServoJ paced on the controller's 4 ms motion cycle.
 *
 * The CR5 takes a ServoJ on its 4 ms motion cycle and divides the step by
 * min(t, the gap since the previous command in whole cycles), so commands must
 * arrive exactly N cycles apart for the target speed to stay even (2026-11-03).
 * The realtime feed (30004, ~8.2 ms) does not show that cycle. The motion
 * module's log port (65511) does: one "[ch8]" text block per cycle, whose
 * field 84 is the queue number being executed.
 *
 * 65511 is an undocumented debug port, not a Dobot interface. Everything that
 * uses it is in this file and its .cpp; TrajectoryExecutor falls back to
 * pacing on the realtime feed without it. Remove both, and the executor's
 * cycle branches, once Dobot provides a supported way.
 */

#ifndef CYCLE_CLOCK_H
#define CYCLE_CLOCK_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Splits the 65511 byte stream into blocks. Block k (1-based) starts at the
// k-th "[ch8]" mark; it is complete when the next mark arrives.
class CycleBlockParser
{
public:
    // Appends data. Returns how many blocks started in it; adds the field 84
    // of each block completed in it to field84 (0 where a block has none).
    size_t feed(const char *data, size_t n, std::vector<uint64_t> &field84);

private:
    std::string text_;      // from the latest mark on; before the first, a split mark's bytes
    bool started_ = false;
};

// Reads 65511 on its own thread, reconnecting while the port is away. Times
// are the host's steady clock at a block's arrival.
class CycleClock
{
public:
    using Clock = std::chrono::steady_clock;
    static constexpr int kPort = 65511;
    static constexpr double kCycleS = 0.004;

    explicit CycleClock(const std::string &ip, int port = kPort);
    ~CycleClock();

    void start();
    void stop();

    // A block arrived within the last 100 ms.
    bool alive() const;
    // Blocks so far.
    uint64_t count() const;
    // Waits until count() >= n, at most timeout: the count then, or 0.
    uint64_t waitBlock(uint64_t n, std::chrono::milliseconds timeout) const;
    // When block k arrived (k <= count(), among the latest few thousand).
    Clock::time_point arrival(uint64_t k) const;
    // The first block whose field 84 showed queue number `queue`.
    std::optional<uint64_t> takenIn(uint64_t queue) const;

    // The reader's handling of bytes received at t; public for tests.
    void onData(const char *data, size_t n, Clock::time_point t);

private:
    void run();

    std::string ip_;
    int port_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<int> fd_{-1};

    mutable std::mutex mutex_;
    mutable std::condition_variable cv_;
    CycleBlockParser parser_;
    uint64_t count_ = 0;
    uint64_t completed_ = 0;
    std::deque<Clock::time_point> times_;   // the latest arrivals; times_.back() is block count_
    std::unordered_map<uint64_t, uint64_t> taken_;
    uint64_t newest_queue_ = 0;
    std::vector<uint64_t> scratch_;
};

// Keeps commands away from the edge of the cycle. Per command: the take-up
// delay, in blocks from the block it was sent after to the block where field
// 84 first shows it. It is constant while the commands sit well inside a
// cycle; one taken a cycle later than usual means they are near the late edge
// and move 1 ms earlier, one taken early, 1 ms later.
class CyclePhaseTracker
{
public:
    static constexpr double kStepMs = 1.0;
    static constexpr size_t kBaseline = 3;  // take-up delays before the usual one counts

    // A command with queue number `queue`, sent after block `block`.
    void sent(uint64_t queue, uint64_t block);
    // Reads the take-up delays seen so far; returns the delay (ms) after its
    // block at which to send the next command.
    double update(const std::function<std::optional<uint64_t>(uint64_t)> &taken_in);
    double delayMs() const { return delay_ms_; }
    int shifts() const { return shifts_; }

private:
    struct Sent
    {
        uint64_t queue;
        uint64_t block;
        std::optional<int64_t> lag;
    };
    std::deque<Sent> sent_;                 // since the latest step
    std::optional<int64_t> usual_;
    double delay_ms_ = 0.0;
    int shifts_ = 0;
};

#endif  // CYCLE_CLOCK_H
