/**
 * Cycle tracking: ServoJ paced on the controller's 4 ms motion cycle.
 *
 * The CR5 takes a ServoJ on its 4 ms motion cycle and divides the step by
 * min(t, the gap since the previous command in whole cycles), so commands must
 * arrive exactly N cycles apart for the target speed to stay even (2026-11-03).
 * The realtime feed (30004, ~8.2 ms) does not show that cycle. The motion
 * module's log port (65511) does: one text block per cycle, a "<field>:<value>"
 * line per field from field 1, whose field 84 is the queue number being executed.
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
// k-th line "1:"; it is complete when the next one arrives.
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
    static constexpr size_t kMinBlocks = 250;      // 1 s

    explicit CycleClock(const std::string &ip, int port = kPort);
    ~CycleClock();

    void start();
    void stop();

    // A block arrived within the last 100 ms, and blocks have come without a
    // silence for long enough (kMinBlocks) to predict the cycle from.
    bool alive() const;
    // Blocks so far.
    uint64_t count() const;
    // Waits until count() >= n, at most timeout: the count then, or 0.
    uint64_t waitBlock(uint64_t n, std::chrono::milliseconds timeout) const;
    // When block k arrived (k <= count(), among the latest few thousand).
    Clock::time_point arrival(uint64_t k) const;
    // When block k is due, past or future: k cycles from the earliest arrival
    // among the latest kBaseBlocks, so blocks that arrive late or bunched (as
    // they do from firmware 2026-11) do not move it.
    Clock::time_point due(uint64_t k) const;
    // The latest block due at or before t.
    uint64_t dueAt(Clock::time_point t) const;
    // The first block whose field 84 showed queue number `queue`.
    std::optional<uint64_t> takenIn(uint64_t queue) const;

    // The reader's handling of bytes received at t; public for tests.
    void onData(const char *data, size_t n, Clock::time_point t);

private:
    // 10 s: the two clocks drift apart ~0.03 ms in it (2.7 ppm, 2026-11-20).
    static constexpr size_t kBaseBlocks = 2500;
    void run();
    Clock::time_point base() const;     // when block 0 was due; under mutex_

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
    // The latest arrivals since the latest silence; times_.back() is block count_.
    // The controller drops blocks in a silence (321 of them on 2026-11-20), so
    // the count stops matching its cycle there.
    std::deque<Clock::time_point> times_;
    std::unordered_map<uint64_t, uint64_t> taken_;
    uint64_t newest_queue_ = 0;
    std::vector<uint64_t> scratch_;
};

// Measures where in the 4 ms cycle the controller's take-up boundary falls, to
// check the send phase of moving commands (the servo_send_ms parameter) against.
// Near the boundary, send and controller jitter split a 6-cycle gap into 5 and
// 7: a 20 % speed step at full speed.
//
// Held commands (the target not moving, so their gaps do not matter) probe:
// their send phase sweeps the cycle, and the take-up delay of each, in blocks
// from the block it was sent after to the block where field 84 first shows
// it, is one more past the boundary. The controller takes a command late now
// and then, by over 2 ms, but almost never early (7 late, 1 early in ~4100,
// 2026-11-20): moving commands belong just after the latest probe still taken
// before the boundary, leaving the rest of the cycle as margin for a late
// take-up. That was 1.95 ms after the block before and after a controller
// power cycle (2026-11-20, 2026-11-23).
class CyclePhaseTracker
{
public:
    static constexpr int kProbePhases = 16; // 0.25 ms apart
    static constexpr size_t kProbesNeeded = 32;
    static constexpr size_t kProbesKept = 64;
    // After the edge: probe spacing, the base's wobble (to ~0.3 ms) and the boundary's.
    static constexpr double kEdgeMarginMs = 0.75;

    // A probe with queue number `queue`, sent phase_ms after block `block` was due.
    void probed(uint64_t queue, uint64_t block, double phase_ms);
    // Reads the probes' take-up delays, measuring the boundary once enough have one.
    void update(const std::function<std::optional<uint64_t>(uint64_t)> &taken_in);
    // The delay (ms) after its block for the next probe.
    double nextProbeMs();

    struct Boundary
    {
        double phase_ms;        // a send this long after a block is taken a block later than one before it
        int64_t lag_before;     // the take-up delay of a send just before it
        double edge_ms;         // the latest probe still taken with lag_before, -2..+2 ms from phase_ms
        double send_ms;         // where moving commands belong: kEdgeMarginMs after the edge, in [0, 4)
        size_t probes;
        size_t misfits;         // probes on the wrong side of it: the jitter around it
    };
    // The boundary that best splits (send phase in ms, take-up delay) samples.
    static std::optional<Boundary> fit(const std::vector<std::pair<double, int64_t>> &samples);
    // The latest measurement, and how many there have been.
    const std::optional<Boundary> &boundary() const { return boundary_; }
    int measurements() const { return measurements_; }

private:
    struct Probe
    {
        uint64_t queue;
        uint64_t block;
        double phase_ms;
        std::optional<int64_t> lag;
    };
    std::deque<Probe> probes_;
    size_t probes_new_ = 0;                 // probes with a take-up delay since the latest measurement
    int probe_next_ = 0;
    std::optional<Boundary> boundary_;
    int measurements_ = 0;
};

#endif  // CYCLE_CLOCK_H
