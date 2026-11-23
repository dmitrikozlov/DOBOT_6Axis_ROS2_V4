#include <dobot_bringup/cycle_clock.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
const char kMark[] = "\n1:";          // a block's first line is field 1
constexpr size_t kMarkLen = sizeof(kMark) - 1;
const char kField84[] = "\n84:";
constexpr size_t kMaxText = 1 << 20;        // a block is a few kB; past this the stream is not blocks
constexpr size_t kKeepTimes = 4096;         // ~16 s of arrivals
constexpr size_t kKeepTaken = 4096;         // queue numbers
constexpr auto kAlive = std::chrono::milliseconds(100);

uint64_t field84(const std::string &text, size_t from, size_t to)
{
    size_t i = text.find(kField84, from);
    if (i == std::string::npos || i >= to)
        return 0;
    return std::strtoull(text.c_str() + i + sizeof(kField84) - 1, nullptr, 10);
}

// A TCP connection to ip:port within timeout_ms, or -1.
int connectTo(const std::string &ip, int port, int timeout_ms)
{
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
        return -1;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int r = connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    if (r < 0 && errno == EINPROGRESS)
    {
        pollfd p{fd, POLLOUT, 0};
        int err = 0;
        socklen_t len = sizeof(err);
        if (poll(&p, 1, timeout_ms) == 1 &&
            getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0)
            r = 0;
    }
    if (r < 0)
    {
        close(fd);
        return -1;
    }
    fcntl(fd, F_SETFL, flags);
    timeval tv{0, 200000};      // recv wakes to see stop()
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return fd;
}
}  // namespace

size_t CycleBlockParser::feed(const char *data, size_t n, std::vector<uint64_t> &field84_out)
{
    // A mark split across calls is found from its first byte; the one the
    // text starts with is not found again.
    size_t from = text_.size() >= kMarkLen - 1 ? text_.size() - (kMarkLen - 1) : 0;
    if (started_)
        from = std::max<size_t>(from, 1);
    text_.append(data, n);
    size_t marks = 0, start = 0;
    for (size_t i = text_.find(kMark, from); i != std::string::npos; i = text_.find(kMark, i + 1))
    {
        if (started_)
            field84_out.push_back(field84(text_, start, i));
        started_ = true;
        start = i;
        marks++;
    }
    if (started_)
        text_.erase(0, start);
    else if (text_.size() > kMarkLen - 1)
        text_.erase(0, text_.size() - (kMarkLen - 1));
    if (text_.size() > kMaxText)
    {
        text_.clear();
        started_ = false;
    }
    return marks;
}

CycleClock::CycleClock(const std::string &ip, int port) : ip_(ip), port_(port) {}

CycleClock::~CycleClock()
{
    stop();
}

void CycleClock::start()
{
    if (running_.exchange(true))
        return;
    thread_ = std::thread(&CycleClock::run, this);
}

void CycleClock::stop()
{
    running_ = false;
    int fd = fd_.exchange(-1);
    if (fd >= 0)
        shutdown(fd, SHUT_RDWR);
    if (thread_.joinable())
        thread_.join();
    if (fd >= 0)
        close(fd);
}

void CycleClock::run()
{
    char buf[65536];
    while (running_)
    {
        int fd = connectTo(ip_, port_, 1000);
        if (fd < 0)
        {
            for (int i = 0; i < 20 && running_; i++)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        fd_ = fd;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            parser_ = CycleBlockParser{};   // a new connection starts mid-block
        }
        while (running_)
        {
            ssize_t got = recv(fd, buf, sizeof(buf), 0);
            if (got > 0)
                onData(buf, static_cast<size_t>(got), Clock::now());
            else if (got == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
                break;
        }
        if (fd_.exchange(-1) == fd)
            close(fd);
    }
}

void CycleClock::onData(const char *data, size_t n, Clock::time_point t)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        scratch_.clear();
        size_t marks = parser_.feed(data, n, scratch_);
        for (uint64_t q : scratch_)
        {
            completed_++;
            if (q == 0 || taken_.count(q))
                continue;
            taken_[q] = completed_;
            newest_queue_ = std::max(newest_queue_, q);
            if (taken_.size() > 2 * kKeepTaken)
            {
                for (auto it = taken_.begin(); it != taken_.end();)
                    it = it->first + kKeepTaken < newest_queue_ ? taken_.erase(it) : std::next(it);
            }
        }
        if (marks && !times_.empty() && t - times_.back() > kAlive)
            times_.clear();
        for (size_t i = 0; i < marks; i++)
            times_.push_back(t);
        while (times_.size() > kKeepTimes)
            times_.pop_front();
        count_ += marks;
    }
    cv_.notify_all();
}

bool CycleClock::alive() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return times_.size() >= kMinBlocks && Clock::now() - times_.back() < kAlive;
}

uint64_t CycleClock::count() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
}

uint64_t CycleClock::waitBlock(uint64_t n, std::chrono::milliseconds timeout) const
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [&] { return count_ >= n; }))
        return 0;
    return count_;
}

CycleClock::Clock::time_point CycleClock::arrival(uint64_t k) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (times_.empty())
        return Clock::now();
    uint64_t back = count_ - k;
    if (k > count_ || back >= times_.size())
        return times_.back();
    return times_[times_.size() - 1 - back];
}

CycleClock::Clock::time_point CycleClock::base() const
{
    const auto cycle = std::chrono::nanoseconds(static_cast<int64_t>(kCycleS * 1e9));
    const size_t n = std::min(times_.size(), kBaseBlocks);
    Clock::time_point earliest = Clock::time_point::max();
    for (size_t i = times_.size() - n; i < times_.size(); i++)
    {
        uint64_t k = count_ - (times_.size() - 1 - i);
        earliest = std::min(earliest, times_[i] - cycle * static_cast<int64_t>(k));
    }
    return earliest;
}

CycleClock::Clock::time_point CycleClock::due(uint64_t k) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (times_.empty())
        return Clock::now();
    return base() + std::chrono::nanoseconds(static_cast<int64_t>(kCycleS * 1e9)) *
                        static_cast<int64_t>(k);
}

uint64_t CycleClock::dueAt(Clock::time_point t) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (times_.empty())
        return count_;
    auto since = t - base();
    if (since.count() < 0)
        return 0;
    return static_cast<uint64_t>(since / std::chrono::nanoseconds(static_cast<int64_t>(kCycleS * 1e9)));
}

std::optional<uint64_t> CycleClock::takenIn(uint64_t queue) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = taken_.find(queue);
    if (it == taken_.end())
        return std::nullopt;
    return it->second;
}

void CyclePhaseTracker::probed(uint64_t queue, uint64_t block, double phase_ms)
{
    probes_.push_back({queue, block, phase_ms, std::nullopt});
    while (probes_.size() > kProbesKept)
        probes_.pop_front();
}

double CyclePhaseTracker::nextProbeMs()
{
    const double cycle_ms = CycleClock::kCycleS * 1000.0;
    double ms = (probe_next_ + 0.5) * cycle_ms / kProbePhases;
    probe_next_ = (probe_next_ + 1) % kProbePhases;
    return ms;
}

std::optional<CyclePhaseTracker::Boundary>
CyclePhaseTracker::fit(const std::vector<std::pair<double, int64_t>> &samples)
{
    const double cycle_ms = CycleClock::kCycleS * 1000.0;
    if (samples.empty())
        return std::nullopt;
    // Phases into [0, cycle): a send a cycle later is one block later.
    std::vector<std::pair<double, int64_t>> s;
    for (auto [phase, lag] : samples)
    {
        int64_t whole = static_cast<int64_t>(std::floor(phase / cycle_ms));
        s.emplace_back(phase - whole * cycle_ms, lag - whole);
    }
    // Per candidate boundary b: lag_before for phases below it, one more above.
    constexpr int kSteps = 80;
    std::vector<size_t> misfits(kSteps);
    std::vector<int64_t> before(kSteps);
    for (int i = 0; i < kSteps; i++)
    {
        double b = i * cycle_ms / kSteps;
        std::map<int64_t, size_t> votes;   // lag_before each sample implies
        for (auto [phase, lag] : s)
            votes[phase < b ? lag : lag - 1]++;
        auto best = std::max_element(votes.begin(), votes.end(),
                                     [](const auto &x, const auto &y) { return x.second < y.second; });
        before[i] = best->first;
        misfits[i] = s.size() - best->second;
    }
    // The middle of the longest run of best candidates, around the cycle.
    size_t fewest = *std::min_element(misfits.begin(), misfits.end());
    int run_start = 0, run_len = 0;
    for (int i = 0; i < kSteps; i++)
    {
        int len = 0;
        while (len < kSteps && misfits[(i + len) % kSteps] == fewest)
            len++;
        if (len > run_len)
        {
            run_len = len;
            run_start = i;
        }
    }
    int mid = (run_start + run_len / 2) % kSteps;
    double b = mid * cycle_ms / kSteps;
    // Late take-ups move probes past the boundary, never before it: the latest
    // probe taken with lag_before was sent before it. Within half a cycle of
    // b; a probe more than that after b is measured against the next block.
    double edge = b;
    bool found = false;
    for (auto [phase, lag] : s)
    {
        double rel = phase - b;
        int64_t l = lag;
        if (rel >= cycle_ms / 2.0)
        {
            rel -= cycle_ms;
            l -= 1;
        }
        else if (rel < -cycle_ms / 2.0)
        {
            rel += cycle_ms;
            l += 1;
        }
        if (l == before[mid] && (!found || b + rel > edge))
        {
            edge = b + rel;
            found = true;
        }
    }
    double send = std::fmod(edge + kEdgeMarginMs, cycle_ms);
    if (send < 0.0)
        send += cycle_ms;
    return Boundary{b, before[mid], edge, send, s.size(), fewest};
}

void CyclePhaseTracker::update(const std::function<std::optional<uint64_t>(uint64_t)> &taken_in)
{
    for (Probe &p : probes_)
    {
        if (!p.lag)
        {
            if (auto b = taken_in(p.queue))
            {
                p.lag = static_cast<int64_t>(*b) - static_cast<int64_t>(p.block);
                probes_new_++;
            }
        }
    }
    if (probes_new_ < kProbesNeeded)
        return;
    std::vector<std::pair<double, int64_t>> samples;
    for (const Probe &p : probes_)
        if (p.lag)
            samples.emplace_back(p.phase_ms, *p.lag);
    probes_new_ = 0;
    if (auto b = fit(samples))
    {
        boundary_ = b;
        measurements_++;
    }
}
