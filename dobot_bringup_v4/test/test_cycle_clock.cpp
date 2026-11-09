// Unit tests for cycle tracking (cycle_clock.h): no controller needed.

#include <gtest/gtest.h>
#include <dobot_bringup/cycle_clock.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <map>
#include <string>
#include <thread>

static std::string block(uint64_t queue)
{
    return "[ch8]\n1:0.1\n2:0.2\n84:" + std::to_string(queue) + "\n85:7\n";
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
    std::string text = block(5) + block(6) + "[ch8]\n84:1\n" + block(8);
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
    std::string text = "[ch8]\n1:2\n840:9\n[ch8]";
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
        for (uint64_t k = 0; k < 50; k++)
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
    EXPECT_TRUE(c.alive());
    server.join();
    c.stop();
    close(srv);
    EXPECT_EQ(c.count(), 50u);
    EXPECT_EQ(c.takenIn(201).value(), 7u);
}

// Commands every 6 blocks, taken up `lag` blocks after the one they followed.
struct FakeController
{
    std::map<uint64_t, uint64_t> taken;
    uint64_t queue = 1000, block = 0;
    void send(CyclePhaseTracker &t, int64_t lag)
    {
        block += 6;
        t.sent(++queue, block);
        taken[queue] = block + lag;
    }
    std::optional<uint64_t> takenIn(uint64_t q) const
    {
        auto it = taken.find(q);
        return it == taken.end() ? std::nullopt : std::optional<uint64_t>(it->second);
    }
};

TEST(CyclePhaseTracker, StepsEarlierAfterALateTakeUp)
{
    CyclePhaseTracker t;
    FakeController c;
    auto taken = [&c](uint64_t q) { return c.takenIn(q); };
    for (int i = 0; i < 5; i++)
    {
        c.send(t, 2);
        EXPECT_DOUBLE_EQ(t.update(taken), 0.0);
    }
    c.send(t, 3);       // a cycle late: near the late edge
    EXPECT_DOUBLE_EQ(t.update(taken), 3.0);   // 1 ms earlier, wrapped into the cycle before
    EXPECT_EQ(t.shifts(), 1);
    // sending in the cycle before, every take-up delay is one more
    for (int i = 0; i < 5; i++)
    {
        c.send(t, 3);
        EXPECT_DOUBLE_EQ(t.update(taken), 3.0);
    }
    c.send(t, 2);       // early: 1 ms later, back across the wrap
    EXPECT_DOUBLE_EQ(t.update(taken), 0.0);
    EXPECT_EQ(t.shifts(), 2);
}

TEST(CyclePhaseTracker, NoStepBeforeTheBaseline)
{
    CyclePhaseTracker t;
    FakeController c;
    auto taken = [&c](uint64_t q) { return c.takenIn(q); };
    c.send(t, 2);
    c.send(t, 3);
    EXPECT_DOUBLE_EQ(t.update(taken), 0.0);
    EXPECT_EQ(t.shifts(), 0);
}
