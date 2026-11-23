// Unit tests for cycle tracking (cycle_clock.h): no controller needed.

#include <gtest/gtest.h>
#include <dobot_bringup/cycle_clock.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <map>
#include <string>
#include <thread>

static std::string block(uint64_t queue)
{
    return "\r\n1:0.1\r\n11:0\r\n84:" + std::to_string(queue) + "\r\n165:10\r\nS1:-1,2,3\r\nS2:-1,2,3";
}

TEST(CycleBlockParser, CountsBlocksAndReadsField84)
{
    CycleBlockParser p;
    std::vector<uint64_t> f84;
    std::string text = "junk" + block(10) + block(10) + block(11);
    EXPECT_EQ(p.feed(text.data(), text.size(), f84), 3u);
    // the third block completes with the next mark
    ASSERT_EQ(f84.size(), 2u);
    EXPECT_EQ(f84[0], 10u);
    EXPECT_EQ(f84[1], 10u);
    std::string next = block(12);
    EXPECT_EQ(p.feed(next.data(), next.size(), f84), 1u);
    ASSERT_EQ(f84.size(), 3u);
    EXPECT_EQ(f84[2], 11u);
}

TEST(CycleBlockParser, MarksAndFieldsSplitAcrossChunks)
{
    std::string text = block(5) + block(6) + "\r\n1:0\r\n84:1" + block(8);
    for (size_t chunk = 1; chunk <= 7; chunk++)
    {
        CycleBlockParser p;
        std::vector<uint64_t> f84;
        size_t marks = 0;
        for (size_t i = 0; i < text.size(); i += chunk)
            marks += p.feed(text.data() + i, std::min(chunk, text.size() - i), f84);
        EXPECT_EQ(marks, 4u) << chunk;
        ASSERT_EQ(f84.size(), 3u) << chunk;
        EXPECT_EQ(f84[0], 5u);
        EXPECT_EQ(f84[1], 6u);
        EXPECT_EQ(f84[2], 1u);
    }
}

TEST(CycleBlockParser, BlockWithoutField84)
{
    CycleBlockParser p;
    std::vector<uint64_t> f84;
    std::string text = "\r\n1:2\r\n840:9\r\n1:3";
    EXPECT_EQ(p.feed(text.data(), text.size(), f84), 2u);
    ASSERT_EQ(f84.size(), 1u);
    EXPECT_EQ(f84[0], 0u);
}

TEST(CycleClock, CountsArrivalsAndTakeUp)
{
    CycleClock c("127.0.0.1", 1);
    auto t0 = CycleClock::Clock::now();
    std::string a = block(100) + block(100) + block(101);
    c.onData(a.data(), a.size(), t0);
    std::string b = block(101) + block(102);
    c.onData(b.data(), b.size(), t0 + std::chrono::milliseconds(8));
    EXPECT_EQ(c.count(), 5u);
    EXPECT_EQ(c.arrival(3), t0);
    EXPECT_EQ(c.arrival(4), t0 + std::chrono::milliseconds(8));
    EXPECT_EQ(c.takenIn(100).value(), 1u);
    EXPECT_EQ(c.takenIn(101).value(), 3u);
    EXPECT_FALSE(c.takenIn(102).has_value());   // its block is not complete yet
    EXPECT_EQ(c.waitBlock(5, std::chrono::milliseconds(1)), 5u);
    EXPECT_EQ(c.waitBlock(6, std::chrono::milliseconds(1)), 0u);
}

TEST(CycleClock, NotAliveWithoutThePort)
{
    CycleClock c("127.0.0.1", 1);   // nothing listens on port 1
    c.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_FALSE(c.alive());
    c.stop();
}

TEST(CycleClock, ReadsFromASocket)
{
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
    socklen_t len = sizeof(addr);
    getsockname(srv, reinterpret_cast<sockaddr *>(&addr), &len);
    listen(srv, 1);
    std::thread server([srv] {
        int fd = accept(srv, nullptr, nullptr);
        for (uint64_t k = 0; k < 300; k++)
        {
            std::string b = block(200 + k / 6);
            send(fd, b.data(), b.size(), 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        close(fd);
    });
    CycleClock c("127.0.0.1", ntohs(addr.sin_port));
    c.start();
    EXPECT_NE(c.waitBlock(20, std::chrono::milliseconds(2000)), 0u);
    EXPECT_FALSE(c.alive());        // too few blocks yet to predict the cycle from
    EXPECT_NE(c.waitBlock(CycleClock::kMinBlocks + 10, std::chrono::milliseconds(3000)), 0u);
    EXPECT_TRUE(c.alive());
    server.join();
    c.stop();
    close(srv);
    EXPECT_EQ(c.count(), 300u);
    EXPECT_EQ(c.takenIn(201).value(), 7u);
}

TEST(CycleClock, DueIgnoresLateAndBunchedArrivals)
{
    CycleClock c("127.0.0.1", 1);
    auto t0 = CycleClock::Clock::now();
    auto cycle = std::chrono::milliseconds(4);
    std::string pending;
    for (uint64_t k = 1; k <= 20; k++)
    {
        pending += block(k);
        if (k == 7)
            continue;                               // 7 arrives together with 8
        auto late = k == 5 ? std::chrono::milliseconds(3) : std::chrono::milliseconds(0);
        c.onData(pending.data(), pending.size(), t0 + cycle * k + late);
        pending.clear();
    }
    EXPECT_EQ(c.count(), 20u);
    EXPECT_EQ(c.due(7), t0 + cycle * 7);
    EXPECT_EQ(c.due(26), t0 + cycle * 26);
    EXPECT_EQ(c.dueAt(t0 + cycle * 26 + std::chrono::milliseconds(1)), 26u);
}

// Taken up in the first block that starts at least lambda_ms after the send:
// the take-up delay of a send phase_ms after its block.
static int64_t takeUp(double phase_ms, double lambda_ms)
{
    return static_cast<int64_t>(std::ceil((phase_ms + lambda_ms) / 4.0));
}

TEST(CyclePhaseTracker, FitsTheBoundaryThroughJitter)
{
    std::vector<std::pair<double, int64_t>> s;
    for (int rep = 0; rep < 4; rep++)
        for (int i = 0; i < 16; i++)
        {
            double phase = (i + 0.5) * 0.25;
            double jitter = 0.3 * ((rep * 7 + i * 3) % 5 - 2) / 2.0;   // -0.3..0.3 ms
            s.emplace_back(phase, takeUp(phase, 9.0 + jitter));
        }
    auto b = CyclePhaseTracker::fit(s);
    ASSERT_TRUE(b.has_value());
    EXPECT_NEAR(b->phase_ms, 3.0, 0.3);       // 3 + 9 = 12: a block boundary
    EXPECT_EQ(b->lag_before, 3);
    EXPECT_EQ(b->probes, 64u);
    EXPECT_NEAR(b->edge_ms, 3.0, 0.45);
}

TEST(CyclePhaseTracker, LateTakeUpsDoNotMoveTheEdge)
{
    std::vector<std::pair<double, int64_t>> s;
    for (int rep = 0; rep < 4; rep++)
        for (int i = 0; i < 16; i++)
        {
            double phase = (i + 0.5) * 0.25;
            bool late = (rep * 16 + i) % 5 == 0;          // every fifth, 2.5 ms late
            s.emplace_back(phase, takeUp(phase, late ? 11.5 : 9.0));
        }
    auto b = CyclePhaseTracker::fit(s);
    ASSERT_TRUE(b.has_value());
    EXPECT_DOUBLE_EQ(b->edge_ms, 2.875);      // the latest probe before 3.0 taken on time
}

TEST(CyclePhaseTracker, FitTakesPhasesPastTheCycleIntoTheNextBlock)
{
    // the same sends, some measured against the block before or after
    std::vector<std::pair<double, int64_t>> s;
    for (int i = 0; i < 40; i++)
    {
        double phase = i * 0.1;
        int64_t lag = takeUp(phase, 9.0);
        if (i % 3 == 0)
            s.emplace_back(phase + 4.0, lag + 1);   // after the block before
        else if (i % 3 == 1)
            s.emplace_back(phase - 4.0, lag - 1);   // after the next block
        else
            s.emplace_back(phase, lag);
    }
    auto b = CyclePhaseTracker::fit(s);
    ASSERT_TRUE(b.has_value());
    EXPECT_NEAR(b->phase_ms, 3.0, 0.15);
    EXPECT_EQ(b->lag_before, 3);
    EXPECT_EQ(b->misfits, 0u);
}

TEST(CyclePhaseTracker, SendPastTheCycleWrapsIntoIt)
{
    std::vector<std::pair<double, int64_t>> s;
    for (int i = 0; i < 16; i++)
    {
        double phase = (i + 0.5) * 0.25;
        s.emplace_back(phase, takeUp(phase, 8.5));      // boundary at 3.5 ms
    }
    auto b = CyclePhaseTracker::fit(s);
    ASSERT_TRUE(b.has_value());
    EXPECT_DOUBLE_EQ(b->edge_ms, 3.375);
    EXPECT_DOUBLE_EQ(b->send_ms, 3.375 + CyclePhaseTracker::kEdgeMarginMs - 4.0);
}

TEST(CyclePhaseTracker, ProbesMeasureWhereMovingCommandsBelong)
{
    CyclePhaseTracker t;
    std::map<uint64_t, uint64_t> taken;
    uint64_t queue = 0, block = 0;
    auto taken_in = [&taken](uint64_t q) {
        auto it = taken.find(q);
        return it == taken.end() ? std::nullopt : std::optional<uint64_t>(it->second);
    };
    for (size_t i = 0; i < CyclePhaseTracker::kProbesNeeded; i++)
    {
        EXPECT_EQ(t.measurements(), 0);
        block += 6;
        double phase = t.nextProbeMs();
        t.probed(++queue, block, phase);
        taken[queue] = block + takeUp(phase, 9.0);
        t.update(taken_in);
    }
    ASSERT_EQ(t.measurements(), 1);
    // just after the boundary at 3.0 ms
    EXPECT_NEAR(t.boundary()->send_ms, 2.875 + CyclePhaseTracker::kEdgeMarginMs, 1e-9);
}
