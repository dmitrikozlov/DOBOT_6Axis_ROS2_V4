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
const char kMark[] = "[ch8]";
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
    return !times_.empty() && Clock::now() - times_.back() < kAlive;
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

std::optional<uint64_t> CycleClock::takenIn(uint64_t queue) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = taken_.find(queue);
    if (it == taken_.end())
        return std::nullopt;
    return it->second;
}

void CyclePhaseTracker::sent(uint64_t queue, uint64_t block)
{
    sent_.push_back({queue, block, std::nullopt});
    while (sent_.size() > 64)
        sent_.pop_front();
}

double CyclePhaseTracker::update(const std::function<std::optional<uint64_t>(uint64_t)> &taken_in)
{
    std::map<int64_t, size_t> seen;
    for (Sent &s : sent_)
    {
        if (!s.lag)
        {
            if (auto b = taken_in(s.queue))
                s.lag = static_cast<int64_t>(*b) - static_cast<int64_t>(s.block);
        }
        if (s.lag)
            seen[*s.lag]++;
    }
    size_t n = 0;
    for (const auto &kv : seen)
        n += kv.second;
    if (n >= kBaseline)
        usual_ = std::max_element(seen.begin(), seen.end(),
                                  [](const auto &a, const auto &b) { return a.second < b.second; })
                     ->first;
    if (!usual_)
        return delay_ms_;
    for (const Sent &s : sent_)
    {
        if (!s.lag || *s.lag == *usual_)
            continue;
        double unwrapped = delay_ms_ + (*s.lag > *usual_ ? -kStepMs : kStepMs);
        const double cycle_ms = CycleClock::kCycleS * 1000.0;
        delay_ms_ = std::fmod(unwrapped, cycle_ms);
        if (delay_ms_ < 0.0)
            delay_ms_ += cycle_ms;
        // Past 0 or a whole cycle the send moves to the neighbouring block's
        // cycle, and every take-up delay with it.
        *usual_ += std::lround((delay_ms_ - unwrapped) / cycle_ms);
        shifts_++;
        sent_.clear();
        break;
    }
    return delay_ms_;
}
