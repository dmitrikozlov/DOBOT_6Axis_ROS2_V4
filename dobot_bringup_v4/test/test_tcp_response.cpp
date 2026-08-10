/**
 * Regression tests for dashboard response framing.
 *
 * doTcpCmd/doTcpCmd_f/sendServoCommand read until a ';' terminator. The read
 * loop used to check its deadline only when tcpRecv reported a timeout — but
 * tcpRecv returns *true* with has_read == 0 once the buffer is full (its
 * `while (len)` body never runs for len == 0). A reply that overran the buffer
 * without a terminator therefore spun forever, holding dash_mutex_: the node
 * stayed up and kept its topics while every service call blocked behind it.
 *
 * Each test drives a throwaway loopback listener — no robot and no ROS graph.
 */

#include <gtest/gtest.h>
#include <dobot_bringup/command.h>
#include <dobot_bringup/tcp_socket.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// A loopback listener standing in for the controller.
class Listener
{
public:
    explicit Listener(uint16_t port = 0)
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        bound_ = fd_ >= 0 && ::bind(fd_, (sockaddr *)&addr, sizeof(addr)) == 0 &&
                 ::listen(fd_, 4) == 0;
        if (!bound_)
            return;

        socklen_t len = sizeof(addr);
        ::getsockname(fd_, (sockaddr *)&addr, &len);
        port_ = ntohs(addr.sin_port);
    }

    ~Listener()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    bool bound() const { return bound_; }
    uint16_t port() const { return port_; }

    int accept(int timeout_ms)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        timeval tv{ timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
        if (::select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0)
            return -1;
        return ::accept(fd_, nullptr, nullptr);
    }

private:
    int fd_ = -1;
    bool bound_ = false;
    uint16_t port_ = 0;
};

// Wait for the command the commander sent, and discard it.
static bool awaitCommand(int fd, int timeout_ms)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    timeval tv{ timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    if (::select(fd + 1, &fds, nullptr, nullptr, &tv) <= 0)
        return false;
    char scratch[256];
    return ::recv(fd, scratch, sizeof(scratch), 0) > 0;
}

static bool sendAll(int fd, const char *data, size_t len)
{
    while (len > 0)
    {
        ssize_t n = ::send(fd, data, len, 0);
        if (n <= 0)
            return false;
        data += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

// Runs a service call on its own thread so the test can play controller while it
// is in flight, and bounds how long the call is allowed to take.
//
// A hung read loop never returns and never releases dash_mutex_, so without this
// a regression surfaces as a whole-suite timeout with nothing pointing at the
// cause. Everything the worker touches is shared-owned, because a hung worker
// cannot be joined: it is detached and outlives the test body.
class BoundedCall
{
public:
    template <typename Fn>
    explicit BoundedCall(Fn fn) : done_(std::make_shared<std::atomic<bool>>(false))
    {
        auto done = done_;
        worker_ = std::thread([fn, done]() mutable {
            fn();
            done->store(true);
        });
    }

    ~BoundedCall()
    {
        if (worker_.joinable())
            worker_.detach();  // only reachable when it hung; process exit reclaims it
    }

    bool wait(int timeout_ms)
    {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (!done_->load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (!done_->load())
            return false;
        worker_.join();
        return true;
    }

private:
    std::shared_ptr<std::atomic<bool>> done_;
    std::thread worker_;
};

// Fixture: the commander hardcodes 29999 (dashboard) and 30004 (realtime), so
// the fake controller has to own those exact ports.
class DashboardResponse : public ::testing::Test
{
protected:
    void SetUp() override
    {
        realtime_ = std::make_unique<Listener>(30004);
        dashboard_ = std::make_unique<Listener>(29999);
        if (!realtime_->bound() || !dashboard_->bound())
            GTEST_SKIP() << "ports 30004/29999 busy (a robot or another test run?)";

        // shared_ptr, not unique_ptr: a hung worker keeps calling into it.
        commander_ = std::make_shared<CRCommanderRos2>("127.0.0.1");
        commander_->init();

        realtime_peer_ = realtime_->accept(2000);
        ASSERT_GE(realtime_peer_, 0) << "commander never opened the realtime link";
        dash_peer_ = dashboard_->accept(2000);
        ASSERT_GE(dash_peer_, 0) << "commander never opened the dashboard link";
    }

    void TearDown() override
    {
        commander_.reset();  // joins recvTask (unless a detached worker still holds it)
        if (dash_peer_ >= 0)
            ::close(dash_peer_);
        if (realtime_peer_ >= 0)
            ::close(realtime_peer_);
    }

    // Heap-owned so a detached worker never writes into the test body's stack.
    std::shared_ptr<std::atomic<int32_t>> makeErrSlot() const
    {
        return std::make_shared<std::atomic<int32_t>>(-99);
    }

    std::unique_ptr<Listener> realtime_;
    std::unique_ptr<Listener> dashboard_;
    std::shared_ptr<CRCommanderRos2> commander_;
    int realtime_peer_ = -1;
    int dash_peer_ = -1;
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// The reported hang: a reply that fills the receive buffer without ever sending
// ';'. The loop must give up and drop the link rather than spin on a zero-length
// read. Dropping it is what lets recvTask resynchronise the stream — a reply left
// unread would otherwise be picked up as the *next* command's response.
TEST_F(DashboardResponse, OversizedReplyWithoutTerminatorDoesNotHang)
{
    auto commander = commander_;
    auto err = makeErrSlot();
    BoundedCall call([commander, err]() {
        int32_t e = 0;
        commander->callRosService("RobotMode()", e);
        err->store(e);
    });

    ASSERT_TRUE(awaitCommand(dash_peer_, 2000)) << "commander never sent the command";
    std::string blob = "0,{";
    blob.append(2048, 'x');  // > kRecvBufSize, and no ';' anywhere
    ASSERT_TRUE(sendAll(dash_peer_, blob.data(), blob.size()));

    ASSERT_TRUE(call.wait(5000)) << "read loop never returned — the unbounded loop is back";
    EXPECT_EQ(err->load(), -1);

    // Link dropped, so recvTask reconnects: a second dashboard connection lands.
    const int again = dashboard_->accept(5000);
    EXPECT_GE(again, 0) << "dashboard link was not dropped and re-established";
    if (again >= 0)
        ::close(again);
}

// A silent controller must still bound the wait. This path the loop always
// handled; it is here so the rewrite cannot regress it unnoticed.
TEST_F(DashboardResponse, SilentControllerTimesOutAndDropsTheLink)
{
    auto commander = commander_;
    auto err = makeErrSlot();
    BoundedCall call([commander, err]() {
        int32_t e = 0;
        commander->callRosService("RobotMode()", e);
        err->store(e);
    });

    EXPECT_TRUE(awaitCommand(dash_peer_, 2000));
    // Deliberately send nothing.

    ASSERT_TRUE(call.wait(5000)) << "read loop never returned on a silent peer";
    EXPECT_EQ(err->load(), -1);
}

// Replies arrive in whatever chunks TCP delivers, so the loop must accumulate
// across reads and stop only at the terminator. The gap here exceeds the 100 ms
// poll, so tcpRecv returns false having *already* read the first chunk — the
// partial read must still advance the write pointer, or the second read
// overwrites it from the start and "0,{5},Robot" + "Mode();" lands as
// "Mode();obot".
//
// The ErrorID is deliberately non-zero: doTcpCmd pre-sets err_id to 0 and only
// overwrites it after finding '{', so a garbled buffer (no '{') also yields 0.
// Asserting 0 here would pass on total corruption.
TEST_F(DashboardResponse, ReplySplitAcrossPacketsIsReassembled)
{
    auto commander = commander_;
    auto err = makeErrSlot();
    BoundedCall call([commander, err]() {
        int32_t e = -99;
        commander->callRosService("RobotMode()", e);
        err->store(e);
    });

    ASSERT_TRUE(awaitCommand(dash_peer_, 2000));
    const std::string head = "-3,{5},Robot";
    const std::string tail = "Mode();";
    ASSERT_TRUE(sendAll(dash_peer_, head.data(), head.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));  // force a second read
    ASSERT_TRUE(sendAll(dash_peer_, tail.data(), tail.size()));

    ASSERT_TRUE(call.wait(5000)) << "read loop never returned on a split reply";
    EXPECT_EQ(err->load(), -3) << "reply was not reassembled across the two reads";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
