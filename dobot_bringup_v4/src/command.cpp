#include <dobot_bringup/command.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

static const rclcpp::Logger kLogger = rclcpp::get_logger("dobot_tcp");
static constexpr uint32_t kRecvBufSize = 1024;
// The controller pushes a realtime packet every ~8 ms, so a second of silence is
// already abnormal and three consecutive misses mean the link is gone.
static constexpr uint32_t kRealtimeRecvTimeoutMs = 1000;
static constexpr int kRealtimeMaxTimeouts = 3;
// The controller stamps every realtime packet with its own length and a fixed
// test pattern (see RealTimeData in command.h). Both are framing markers: if
// either is wrong, the read did not start on a packet boundary.
static constexpr uint64_t kRealtimeTestValue = 0x0123456789ABCDEFULL;

CRCommanderRos2::CRCommanderRos2(const std::string &ip)
    : is_running_(false)
{
    real_time_data_ = std::make_shared<RealTimeData>();
    real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
    dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
}

CRCommanderRos2::~CRCommanderRos2()
{
    is_running_ = false;
    if (thread_ && thread_->joinable())
        thread_->join();
}

void CRCommanderRos2::getCurrentJointStatus(double *joint)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < 6; i++)
        joint[i] = deg2Rad(real_time_data_->q_actual[i]);
}

void CRCommanderRos2::getToolVectorActual(double *val)
{
    std::lock_guard<std::mutex> lock(mutex_);
    memcpy(val, real_time_data_->tool_vector_actual, sizeof(double) * 6);
}

void CRCommanderRos2::recvTask()
{
    // Edge-triggered logging for the realtime link: when the controller is off,
    // the recv times out (or the reconnect fails) on every loop iteration, which
    // otherwise floods the log. Log the transition to unhealthy once, and the
    // recovery once, so each failure/recovery cycle is at most two lines.
    bool realtime_healthy = true;
    bool dashboard_healthy = true;
    int realtime_timeouts = 0;
    while (is_running_)
    {
        if (real_time_tcp_->isConnect())
        {
            try
            {
                RealTimeData packet;
                if (real_time_tcp_->tcpRecvExact(&packet, sizeof(RealTimeData), kRealtimeRecvTimeoutMs))
                {
                    realtime_timeouts = 0;

                    // Reject out-of-frame packets rather than publishing them.
                    // A desynchronised stream reads as plausible-looking
                    // garbage: robot_mode lands on some unimplemented field and
                    // comes out 0, q_actual comes out zeros, and everything
                    // downstream believes it. The FJT executor gates on
                    // robot_mode, so it then sits out its whole ENABLE timeout
                    // and the arm refuses to move until the node is restarted.
                    // Dropping the link makes that self-healing: the reconnect
                    // below starts reading on a packet boundary again.
                    if (packet.len != sizeof(RealTimeData) ||
                        packet.test_value != kRealtimeTestValue)
                    {
                        if (realtime_healthy)
                        {
                            RCLCPP_ERROR(kLogger,
                                "realtime packet out of frame (len=%u, expected %zu; "
                                "test_value=0x%016llx, expected 0x%016llx) — dropping "
                                "link to resynchronise; suppressing until recovery",
                                packet.len, sizeof(RealTimeData),
                                static_cast<unsigned long long>(packet.test_value),
                                static_cast<unsigned long long>(kRealtimeTestValue));
                            realtime_healthy = false;
                        }
                        real_time_tcp_->disConnect();
                        continue;
                    }

                    if (!realtime_healthy)
                    {
                        RCLCPP_INFO(kLogger, "tcp recv recovered");
                        realtime_healthy = true;
                    }

                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        *real_time_data_ = packet;
                        real_time_stamp_ = std::chrono::steady_clock::now();
                    }
                }
                else if (++realtime_timeouts >= kRealtimeMaxTimeouts)
                {
                    // A restarting controller usually leaves this socket half-open
                    // (it never gets to send a FIN), so recv only ever times out and
                    // the connection stays nominally "up" forever. Drop it so the
                    // reconnect branch below can re-establish it — without this the
                    // node had to be restarted by hand. This also resynchronises the
                    // packet framing, which a partial read leaves misaligned.
                    if (realtime_healthy)
                    {
                        RCLCPP_WARN(kLogger,
                            "tcp recv timeout (controller restarted?) — dropping realtime "
                            "link and reconnecting; suppressing until recovery");
                        realtime_healthy = false;
                    }
                    real_time_tcp_->disConnect();
                    realtime_timeouts = 0;
                }
            }
            catch (const TcpClientException &err)
            {
                real_time_tcp_->disConnect();
                realtime_timeouts = 0;
                if (realtime_healthy)
                {
                    RCLCPP_ERROR(kLogger, "tcp recv error: %s", err.what());
                    realtime_healthy = false;
                }
            }
        }
        else
        {
            try
            {
                real_time_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {
                if (realtime_healthy)
                {
                    RCLCPP_ERROR(kLogger, "realtime tcp connect failed: %s", err.what());
                    realtime_healthy = false;
                }
                sleep(3);
            }
        }

        if (!dash_board_tcp_->isConnect())
        {
            // Reconnect under dash_mutex_: a concurrent service call or servo command
            // would otherwise send on the fd while it is being swapped underneath it.
            bool connect_failed = false;
            {
                std::lock_guard<std::mutex> lock(dash_mutex_);
                try
                {
                    dash_board_tcp_->connect();
                    // Edge-triggered, like the failure below: the operator has to
                    // learn the link came back, or the single "connect failed"
                    // line stands unanswered forever and every later log is read
                    // as if the arm were still unreachable.
                    if (!dashboard_healthy)
                    {
                        RCLCPP_INFO(kLogger, "dashboard tcp connection restored");
                        dashboard_healthy = true;
                    }
                }
                catch (const TcpClientException &err)
                {
                    if (dashboard_healthy)
                    {
                        RCLCPP_ERROR(kLogger, "dashboard tcp connect failed: %s (suppressing until recovery)",
                                     err.what());
                        dashboard_healthy = false;
                    }
                    connect_failed = true;
                }
            }
            if (connect_failed)
                sleep(3);  // outside the lock: don't stall service calls on a doomed link
        }
    }
}

void CRCommanderRos2::init()
{
    try
    {
        is_running_ = true;
        thread_ = std::unique_ptr<std::thread>(new std::thread(&CRCommanderRos2::recvTask, this));
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(kLogger, "Commander init failed: %s", err.what());
    }
}
int stringToInt(const std::string& str) {
    return std::atoi(str.c_str());
}

namespace
{
// How long a dashboard command waits for its ';'-terminated reply, and how long
// each individual recv blocks while waiting.
constexpr auto kResponseTimeout = std::chrono::seconds(2);
constexpr uint32_t kResponsePollMs = 100;

/// Read one ';'-terminated dashboard response into `buf`, which is left NUL
/// terminated. Returns false — after dropping the link — when no complete
/// response arrives in time, or when one overruns the buffer with no terminator.
///
/// Both failures leave the byte stream unusable: a reply that shows up late is
/// read as the NEXT command's response, and every result after that is offset by
/// one. Dropping the socket is what forces recvTask to reconnect and resynchronise.
///
/// The deadline is checked at the TOP of every iteration, not only when tcpRecv
/// reports a timeout. That distinction is the whole point of this helper:
/// tcpRecv returns *true* with has_read == 0 once the buffer is full (its
/// `while (len)` loop never runs), so a deadline confined to the !ok branch spins
/// forever — while holding dash_mutex_. The node stays alive, keeps its topics,
/// and silently stops answering every service call.
bool recvResponse(std::shared_ptr<TcpClient> &tcp, char *buf, uint32_t buf_size,
                  const char *fn, const char *cmd)
{
    char *recv_ptr = buf;
    const auto deadline = std::chrono::steady_clock::now() + kResponseTimeout;

    while (true)
    {
        if (std::chrono::steady_clock::now() > deadline)
        {
            RCLCPP_ERROR(kLogger, "%s: response timeout for: %s — dropping dashboard link", fn, cmd);
            tcp->disConnect();
            return false;
        }

        // Reserve the last byte so buf stays NUL terminated for strchr()/"%s".
        const uint32_t space = buf_size - 1 - static_cast<uint32_t>(recv_ptr - buf);
        if (space == 0)
        {
            RCLCPP_ERROR(kLogger, "%s: response exceeded %u bytes with no ';' for: %s — dropping dashboard link",
                         fn, buf_size - 1, cmd);
            tcp->disConnect();
            return false;
        }

        // tcpRecv returns false when select() times out — but it may already have
        // read part of the reply, and has_read reports how much. Advance by
        // has_read regardless: treating a partial read as "no progress" leaves the
        // bytes in place and lets the next read overwrite them from the start, so a
        // reply split across TCP segments ("0,{5},Robot" then "Mode();") assembles
        // as "Mode();obot". Genuine socket errors throw, so has_read is enough to
        // drive the loop and the bool carries no information we need.
        uint32_t has_read = 0;
        (void)tcp->tcpRecv(recv_ptr, space, has_read, kResponsePollMs);
        if (has_read == 0)
            continue;  // nothing arrived this round; the deadline above bounds the wait

        recv_ptr += has_read;
        if (*(recv_ptr - 1) == ';')
            return true;
    }
}
}  // namespace
void CRCommanderRos2::doTcpCmd(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    err_id = 0;
    try
    {
        char buf[kRecvBufSize];
        memset(buf, 0, sizeof(buf));
        auto currentTime = std::chrono::system_clock::now();
        auto currentTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTime);
        auto valueMS = currentTime_ms.time_since_epoch().count();
        RCLCPP_INFO(kLogger, "time: %ld  tcp send cmd: %s", valueMS, cmd);

        tcp->tcpSend(cmd, strlen(cmd));
        // Either the link is half-open (controller restarted) or the command/
        // response stream has lost sync — a late reply would be read as the *next*
        // command's. Both are only fixable by reconnecting, which recvTask does
        // once recvResponse has taken the socket down.
        if (!recvResponse(tcp, buf, kRecvBufSize, "doTcpCmd", cmd))
        {
            err_id = -1;
            return;
        }

        const char* p = std::strchr(buf, '{');
        if (p)
        {
            std::string num_str(buf, p - buf);
            while (!num_str.empty() && (num_str.back() == ',' || num_str.back() == ' '))
                num_str.pop_back();
            err_id = stringToInt(num_str);
            RCLCPP_INFO(kLogger, "ErrorID: %s", num_str.c_str());
        }

        RCLCPP_INFO(kLogger, "tcp recv feedback: %s", buf); // FIXME parse the buf may be better
    }
    catch (const std::logic_error &err)
    {
        // A command that never left the host must not be reported as executed.
        // err_id is pre-set to 0 above, and leaving it there on a send failure
        // ("tcp is disconnected", after the controller was power-cycled) told
        // every ROS caller res=0 = success. Callers then built on a command that
        // did nothing — e.g. the tool-485 init (SetToolPower/SetToolMode/
        // SetTool485) "succeeded" while the dashboard link was down, so the
        // gripper's bridge socket was opened onto an unconfigured tool port and
        // every Modbus transaction timed out from then on.
        RCLCPP_ERROR(kLogger, "tcpDoCmd failed: %s", err.what());
        err_id = -1;
    }
}


void CRCommanderRos2::doTcpCmd_f(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,std::string &mode_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    err_id = 0;
    try
    {
        char buf[kRecvBufSize];
        memset(buf, 0, sizeof(buf));
        auto currentTime = std::chrono::system_clock::now();
        auto currentTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTime);
        auto valueMS = currentTime_ms.time_since_epoch().count();
        RCLCPP_INFO(kLogger, "time: %ld  tcp send cmd: %s", valueMS, cmd);
        tcp->tcpSend(cmd, strlen(cmd));
        if (!recvResponse(tcp, buf, kRecvBufSize, "doTcpCmd_f", cmd))
        {
            err_id = -1;
            return;
        }

        const char* brace_start = std::strchr(buf, '{');
        if (brace_start != nullptr)
        {
            std::string num_str(buf, brace_start - buf);

            while (!num_str.empty() &&
                   (num_str.back() == ',' || num_str.back() == ' '))
            {
                num_str.pop_back();
            }

            err_id = stringToInt(num_str);
            RCLCPP_INFO(kLogger, "ErrorID: %d", err_id);

            const char* brace_end = std::strchr(brace_start, '}');

            if (brace_end != nullptr)
                mode_id = std::string(brace_start, brace_end - brace_start + 1);
        }

        RCLCPP_INFO(kLogger, "tcp recv feedback: %s", buf); // FIXME parse the buf may be better
    }
    catch (const std::logic_error &err)
    {
        // Report the failure instead of the pre-set err_id = 0 — see doTcpCmd.
        RCLCPP_ERROR(kLogger, "tcpDoCmd failed: %s", err.what());
        err_id = -1;
    }
}

bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id)
{
    std::lock_guard<std::mutex> lock(dash_mutex_);
    try
    {
        std::vector<std::string> result_;
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(kLogger, "callRosService failed: %s", err.what());
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService_f(const std::string cmd, int32_t &err_id,std::string &mode_id)
{
    std::lock_guard<std::mutex> lock(dash_mutex_);
    try
    {
        std::vector<std::string> result_;
        doTcpCmd_f(this->dash_board_tcp_, cmd.c_str(), err_id,mode_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(kLogger, "callRosService_f failed: %s", err.what());
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id, std::vector<std::string> &result_)
{
    std::lock_guard<std::mutex> lock(dash_mutex_);
    try
    {
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(kLogger, "callRosService failed: %s", err.what());
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::sendServoCommand(const std::string &cmd, int32_t &err_id)
{
    std::lock_guard<std::mutex> lock(dash_mutex_);
    try
    {
        char buf[kRecvBufSize];
        memset(buf, 0, sizeof(buf));

        dash_board_tcp_->tcpSend(cmd.c_str(), cmd.length());

        // Read response until ';' terminator (bounded by kResponseTimeout).
        if (!recvResponse(dash_board_tcp_, buf, kRecvBufSize, "sendServoCommand", cmd.c_str()))
        {
            err_id = -1;
            return false;
        }

        // Parse error ID (number before first '{').
        err_id = 0;
        for (int i = 0; buf[i] != '\0'; i++)
        {
            if (buf[i] == '{')
            {
                std::string num_str(buf, i);
                // Trim trailing comma / whitespace
                while (!num_str.empty() &&
                       (num_str.back() == ',' || num_str.back() == ' '))
                    num_str.pop_back();
                err_id = std::atoi(num_str.c_str());
                break;
            }
        }

        return true;
    }
    catch (const std::logic_error &err)
    {
        RCLCPP_ERROR(kLogger, "sendServoCommand failed: %s", err.what());
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::isEnable() const
{
    return real_time_data_->robot_mode == 5;
}

bool CRCommanderRos2::isConnected() const
{
    bool dash_connected = dash_board_tcp_->isConnect();
    bool real_time_connected = real_time_tcp_->isConnect();
    
    // 仅在连接断开时打印日志，帮助分析问题
    static bool last_connected = true;
    bool current_connected = dash_connected && real_time_connected;
    
    if (!current_connected && last_connected) {
        RCLCPP_WARN(rclcpp::get_logger("CRCommanderRos2"), 
                    "Robot disconnected - Dashboard: %s, Real-time: %s", 
                    dash_connected ? "connected" : "disconnected", 
                    real_time_connected ? "connected" : "disconnected");
    }
    last_connected = current_connected;
    
    return current_connected;
}

uint64_t CRCommanderRos2::getRobotMode() const
{
    return real_time_data_->robot_mode;
}

RealTimeData CRCommanderRos2::getRealData() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return *real_time_data_;
}

int64_t CRCommanderRos2::realtimeAgeMs() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (real_time_stamp_.time_since_epoch().count() == 0)
        return -1;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - real_time_stamp_)
        .count();
}
