#include <dobot_bringup/trajectory_executor.h>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

#include <pthread.h>
#include <sched.h>

static const rclcpp::Logger kLogger = rclcpp::get_logger("trajectory_executor");
// The controller pushes a realtime packet every ~8 ms. Half a second is 60
// missed packets — comfortably past any jitter, and short of the ~3 s a
// reconnect takes, so a genuine blip still gets the full 10 s wait below.
static constexpr int64_t kRealtimeStaleMs = 500;
// While streaming, a feed this old means the arm is moving unwatched.
static constexpr int64_t kStreamStaleMs = 100;

namespace
{
// One sent ServoJ, kept so a controller rejection can be traced to the
// commands just before it.
struct SentServo
{
    double elapsed;                 // trajectory time of the target (s)
    double gap_ms;                  // wall time since the previous send
    std::array<double, 6> target;   // deg
    std::array<double, 6> actual;   // deg, controller feedback at send time
    std::array<double, 6> ctrl_target;  // deg, the controller's own target then
    double tcp_target;              // mm/s, the controller's target tool speed
    double tcp_actual;              // mm/s, its measured tool speed
};

// The linear part of the feed's TCP speed (x, y, z; mm/s).
double linearSpeed(const double *v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

constexpr size_t kRecentSends = 8;
// The controller's target further than this from the arm before a stream is
// worth a warning.
constexpr double kTargetOffsetWarnDeg = 0.5;

std::string joints(const std::array<double, 6> &v, const char *fmt = "%.3f")
{
    std::string out = "[";
    char buf[32];
    for (size_t j = 0; j < 6; j++)
    {
        std::snprintf(buf, sizeof(buf), fmt, v[j]);
        out += buf;
        if (j < 5)
            out += ", ";
    }
    return out + "]";
}

// The commands before a failure: target, actual, and the joint speed the
// controller derives from consecutive targets (step / t).
void logRecentSends(const std::deque<SentServo> &sends, double servo_t)
{
    RCLCPP_ERROR(kLogger, "Last %zu ServoJ commands (deg; speed = step / t=%.3f s):",
                 sends.size(), servo_t);
    for (size_t i = 0; i < sends.size(); i++)
    {
        const SentServo &s = sends[i];
        std::array<double, 6> speed{};
        if (i > 0)
            for (size_t j = 0; j < 6; j++)
                speed[j] = (s.target[j] - sends[i - 1].target[j]) / servo_t;
        RCLCPP_ERROR(kLogger, "  t=%.3f gap=%.1f ms target=%s actual=%s speed=%s deg/s "
                     "controller target=%s tool speed target %.0f actual %.0f mm/s",
                     s.elapsed, s.gap_ms, joints(s.target).c_str(),
                     joints(s.actual).c_str(), joints(speed, "%.1f").c_str(),
                     joints(s.ctrl_target).c_str(), s.tcp_target, s.tcp_actual);
    }
}
}  // namespace

const std::array<std::string, 6> TrajectoryExecutor::kJointNames{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

std::string TrajectoryExecutor::jointOrder(const std::vector<std::string> &joint_names,
                                          std::array<size_t, 6> &from)
{
    if (joint_names.size() != kJointNames.size())
        return "the trajectory names " + std::to_string(joint_names.size()) +
               " joints; the arm has " + std::to_string(kJointNames.size());
    for (size_t j = 0; j < kJointNames.size(); j++)
    {
        auto it = std::find(joint_names.begin(), joint_names.end(), kJointNames[j]);
        if (it == joint_names.end())
            return "the trajectory has no " + kJointNames[j];
        from[j] = static_cast<size_t>(it - joint_names.begin());
    }
    return "";
}

std::string TrajectoryExecutor::toDriverOrder(
    const trajectory_msgs::msg::JointTrajectory &traj,
    trajectory_msgs::msg::JointTrajectory &out)
{
    std::array<size_t, 6> from{};
    std::string why = jointOrder(traj.joint_names, from);
    if (!why.empty())
        return why;
    if (traj.points.empty())
        return "the trajectory has no points";

    out = trajectory_msgs::msg::JointTrajectory{};
    out.header = traj.header;
    out.joint_names.assign(kJointNames.begin(), kJointNames.end());
    double previous = 0.0;
    for (size_t i = 0; i < traj.points.size(); i++)
    {
        const auto &pt = traj.points[i];
        if (pt.positions.size() != kJointNames.size())
            return "point " + std::to_string(i) + " has " + std::to_string(pt.positions.size()) +
                   " positions";
        double t = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9;
        if (t < previous)
            return "point " + std::to_string(i) + " is earlier than the one before it";
        previous = t;
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.time_from_start = pt.time_from_start;
        for (size_t j = 0; j < kJointNames.size(); j++)
        {
            double q = pt.positions[from[j]];
            if (!std::isfinite(q))
                return "point " + std::to_string(i) + " has a non-finite position";
            p.positions.push_back(q);
        }
        out.points.push_back(p);
    }
    return "";
}

std::string TrajectoryExecutor::startFault(const std::array<double, 6> &start,
                                          const std::array<double, 6> &lo,
                                          const std::array<double, 6> &hi,
                                          const std::array<double, 6> &ctrl_target)
{
    size_t worst_j = 0;
    double worst = 0.0;
    for (size_t j = 0; j < 6; j++)
    {
        double out = std::max(lo[j] - ctrl_target[j], ctrl_target[j] - hi[j]);
        if (out > worst)
        {
            worst = out;
            worst_j = j;
        }
    }
    if (worst <= kStartFaultDeg)
        return "";

    // Every joint that is not near 0 moved toward 0 by a fraction of its angle.
    std::vector<double> fractions;
    bool toward_zero = true;
    for (size_t j = 0; j < 6; j++)
    {
        if (std::fabs(start[j]) < 1.0)
            continue;
        double f = (start[j] - ctrl_target[j]) / start[j];
        toward_zero = toward_zero && f > 0.0;
        fractions.push_back(f);
    }
    char buf[200];
    if (toward_zero && fractions.size() >= 3)
    {
        std::sort(fractions.begin(), fractions.end());
        std::snprintf(buf, sizeof(buf),
                      "every joint moved toward 0 deg by %.3f %% of its angle (%.3f-%.3f %%)",
                      fractions[fractions.size() / 2] * 100.0, fractions.front() * 100.0,
                      fractions.back() * 100.0);
    }
    else
    {
        std::snprintf(buf, sizeof(buf),
                      "%s moved to %.3f deg, %.2f deg outside the commanded %.3f..%.3f deg",
                      kJointNames[worst_j].c_str(), ctrl_target[worst_j], worst,
                      lo[worst_j], hi[worst_j]);
    }
    return buf;
}

bool TrajectoryExecutor::ArrivalCheck::update(double t, const std::array<double, 6> &offset_deg)
{
    for (size_t j = 0; j < 6; j++)
    {
        if (std::fabs(offset_deg[j]) > tolerance_[j])
        {
            within_since_ = -1.0;
            return false;
        }
    }
    if (within_since_ < 0.0)
        within_since_ = t;
    return t - within_since_ >= kSettleS;
}

TrajectoryExecutor::TrajectoryExecutor(
    std::shared_ptr<CRCommanderRos2> commander,
    double servo_rate, double servo_t,
    double aheadtime, double gain, int rt_priority)
    : commander_(commander),
      servo_rate_(servo_rate),
      servo_t_(servo_t),
      aheadtime_(aheadtime),
      gain_(gain),
      rt_priority_(rt_priority)
{
}

double TrajectoryExecutor::nextDue(double due, double sent, double period)
{
    return (sent - due > kLateSend ? sent : due) + period;
}

TrajectoryExecutor::~TrajectoryExecutor()
{
    stop();
}

void TrajectoryExecutor::loadTrajectory(const trajectory_msgs::msg::JointTrajectory &traj)
{
    loadTrajectory(traj, GoalTolerance{});
}

void TrajectoryExecutor::loadTrajectory(
    const trajectory_msgs::msg::JointTrajectory &traj,
    const GoalTolerance &tolerance)
{
    stop();

    std::lock_guard<std::mutex> lock(mutex_);
    tolerance_ = tolerance;
    timestamps_.clear();
    joints_deg_.clear();

    for (const auto &pt : traj.points)
    {
        double t = pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9;
        timestamps_.push_back(t);

        std::array<double, 6> joints{};
        for (size_t i = 0; i < 6 && i < pt.positions.size(); i++)
        {
            joints[i] = rad2Deg(pt.positions[i]);
        }
        joints_deg_.push_back(joints);
    }

    state_ = State{};
    if (!timestamps_.empty())
    {
        state_.total_duration = timestamps_.back();
    }
}

void TrajectoryExecutor::start(bool behind_movj)
{
    if (running_)
        return;
    behind_movj_ = behind_movj;
    halt_requested_ = false;

    // Join a previously completed thread before creating a new one.
    // Without this, assigning to thread_ destroys the old joinable
    // std::thread, which calls std::terminate().
    if (thread_.joinable())
        thread_.join();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamps_.empty())
        {
            state_.done = true;
            return;
        }
        state_.done = false;
        state_.error_msg.clear();
        state_.goal_tolerance_violated = false;
        state_.halted = false;
        state_.elapsed = 0.0;
    }

    running_ = true;
    thread_ = std::thread(&TrajectoryExecutor::servoLoop, this);
}

void TrajectoryExecutor::stop()
{
    if (!running_)
        return;

    // Once streamed, the arm is settling on the last point and Stop() has
    // nothing to halt. It would only end the servo session early, which on the
    // CR5 can make a later stream lurch the wrong way (Stop() on an arrived
    // session, then MovL, then MovJ: alarms 225/227; bench, 2026-10-22).
    bool streamed = settling_;
    running_ = false;

    if (thread_.joinable())
        thread_.join();

    if (commander_ && !streamed)
    {
        int32_t err_id = 0;
        commander_->sendServoCommand("Stop()", err_id);
    }
}

double TrajectoryExecutor::haltRateStep(double tool_speed_mm_s, double decel_mm_s2, double period)
{
    double stop_s = decel_mm_s2 > 0.0 ? tool_speed_mm_s / decel_mm_s2 : 0.0;
    return period / std::max(stop_s, period);
}

void TrajectoryExecutor::halt()
{
    // Only the first call acts: once a halt is under way, its settle wait is
    // what tells that the arm has stopped.
    if (!running_ || halt_requested_.exchange(true))
        return;
    // Streamed to the end: the arm is settling and there is nothing to slow.
    if (settling_)
        running_ = false;
}

bool TrajectoryExecutor::isRunning() const
{
    return running_;
}

void TrajectoryExecutor::fail(const std::string &msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.error_msg = msg;
    state_.done = true;
    RCLCPP_ERROR(kLogger, "%s", msg.c_str());
}

TrajectoryExecutor::State TrajectoryExecutor::getState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

std::array<double, 6> TrajectoryExecutor::interpolateAt(double t) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return interpolate(t);
}

// MoveIt trajectories have waypoints at irregular time intervals.
// The servo loop sends commands at a fixed rate (every 20ms at 50Hz).
// Linearly interpolate joint positions (degrees) at time t between
// the two surrounding trajectory waypoints.  Caller must hold mutex_.
std::array<double, 6> TrajectoryExecutor::interpolate(double t) const
{
    // Caller must hold mutex_.
    if (joints_deg_.empty())
        return {};

    if (timestamps_.size() == 1 || t <= timestamps_.front())
        return joints_deg_.front();

    if (t >= timestamps_.back())
        return joints_deg_.back();

    // Find the segment containing t.
    size_t seg = 0;
    for (size_t i = 0; i < timestamps_.size() - 1; i++)
    {
        if (timestamps_[i + 1] >= t)
        {
            seg = i;
            break;
        }
    }

    double t0 = timestamps_[seg];
    double t1 = timestamps_[seg + 1];
    double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 1.0;
    alpha = std::min(1.0, std::max(0.0, alpha));

    std::array<double, 6> result{};
    for (size_t j = 0; j < 6; j++)
    {
        result[j] = joints_deg_[seg][j] +
                     alpha * (joints_deg_[seg + 1][j] - joints_deg_[seg][j]);
    }
    return result;
}

void TrajectoryExecutor::servoLoop()
{
    // A late wake-up delays a ServoJ; the controller has nothing new to aim
    // at meanwhile and the arm dips. Ahead of the camera and planning threads
    // on a busy Pi, the servo thread wakes on time. Needs RLIMIT_RTPRIO
    // (LimitRTPRIO= in the systemd unit) or CAP_SYS_NICE.
    if (rt_priority_ > 0)
    {
        sched_param sp{};
        sp.sched_priority = rt_priority_;
        int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        static std::atomic<bool> warned{false};
        if (err != 0 && !warned.exchange(true))
            RCLCPP_WARN(kLogger, "Servo thread stays at normal priority: SCHED_FIFO %d refused (%s)",
                        rt_priority_, std::strerror(err));
    }

    RCLCPP_INFO(kLogger,
                "Servo loop started: rate=%.1f Hz, t=%.4f s, "
                "aheadtime=%.1f, gain=%.1f, points=%zu",
                servo_rate_, servo_t_, aheadtime_, gain_,
                joints_deg_.size());

    // Wait for robot to be in ENABLE mode (5) before streaming ServoJ.
    // ServoJ is rejected if the robot is still RUNNING (7) from a
    // previous command.
    auto wait_start = std::chrono::steady_clock::now();
    while (running_)
    {
        if (halt_requested_)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.done = true;
            state_.halted = true;
            running_ = false;
            RCLCPP_INFO(kLogger, "Halted before the stream started: nothing sent");
            return;
        }
        // robot_mode comes off the realtime feed (TCP 30004), not the
        // dashboard. When that feed stalls the struct keeps whatever it last
        // held, so "the robot will not enable" and "we have no idea what the
        // robot is doing" arrive here looking identical — and the operator
        // chasing the resulting "current: 0" goes and stares at the arm
        // instead of the link. Tell them apart.
        int64_t age_ms = commander_->realtimeAgeMs();
        bool stale = age_ms < 0 || age_ms > kRealtimeStaleMs;

        uint64_t mode = 0;
        if (!stale)
        {
            RealTimeData rd = commander_->getRealData();
            mode = rd.robot_mode;
            if (mode == 5 || (behind_movj_ && mode == 7))
                break;
            if (mode == 9 || mode == 11)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_.error_msg =
                    "Robot in error mode " + std::to_string(mode) +
                    " before trajectory start";
                state_.done = true;
                RCLCPP_ERROR(kLogger, "Robot mode %lu before start — aborting",
                             static_cast<unsigned long>(mode));
                running_ = false;
                return;
            }
        }
        if (std::chrono::steady_clock::now() - wait_start >
            std::chrono::seconds(10))
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.error_msg =
                stale ? "Realtime feedback stale (" +
                            (age_ms < 0 ? std::string("no packet ever received")
                                        : std::to_string(age_ms) + " ms since the last one") +
                            ") — robot mode unknown; check the TCP 30004 link"
                      : "Timeout waiting for ENABLE mode (current: " +
                            std::to_string(mode) + ")";
            state_.done = true;
            RCLCPP_ERROR(kLogger, "%s", state_.error_msg.c_str());
            running_ = false;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Where the controller is aiming before the first command. A target away
    // from the arm means it still holds an earlier motion's, and streaming
    // may start by driving towards it.
    std::array<double, 6> lo{}, hi{}, first{};  // the start-of-stream check (startFault)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        first = joints_deg_.front();
    }
    {
        RealTimeData rd = commander_->getRealData();
        std::array<double, 6> ctrl{}, actual{};
        double worst = 0.0;
        for (size_t j = 0; j < 6; j++)
        {
            ctrl[j] = rd.q_target[j];
            actual[j] = rd.q_actual[j];
            worst = std::max(worst, std::fabs(ctrl[j] - actual[j]));
        }
        if (worst > kTargetOffsetWarnDeg)
            RCLCPP_WARN(kLogger, "Before streaming: the controller's target %s is %.2f deg "
                        "from the arm %s", joints(ctrl).c_str(), worst, joints(actual).c_str());
        else
            RCLCPP_INFO(kLogger, "Before streaming: the controller's target is on the arm "
                        "(within %.3f deg)", worst);
        lo = hi = ctrl;
    }
    // Why the controller's target left the commanded path, with when and the
    // controller's uptime; empty past kStartWatchS or while it follows.
    auto start_fault = [&](double elapsed, const RealTimeData &rd) -> std::string {
        if (elapsed > kStartWatchS)
            return "";
        std::array<double, 6> ctrl{};
        for (size_t j = 0; j < 6; j++)
            ctrl[j] = rd.q_target[j];
        std::string what = startFault(first, lo, hi, ctrl);
        if (what.empty())
            return "";
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "%.0f ms into the stream (controller up %llu s) the controller's target "
                      "left the commanded path: ",
                      elapsed * 1e3, static_cast<unsigned long long>(rd.run_time));
        return buf + what + " — a controller fault, not the trajectory";
    };

    auto start_time = std::chrono::steady_clock::now();
    const double period = 1.0 / servo_rate_;
    auto since_start = [&start_time](std::chrono::steady_clock::time_point t) {
        return std::chrono::duration<double>(t - start_time).count();
    };

    double total_duration;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        total_duration = state_.total_duration;
    }

    uint64_t iteration = 0;
    std::deque<SentServo> recent;
    auto last_send = start_time;
    double max_gap_ms = 0.0;
    double due = 0.0;               // when this command was due (s from start)
    int late_sends = 0;
    double max_wake_late_ms = 0.0;  // how late the thread woke for a command
    double max_reply_ms = 0.0;      // the controller's slowest ServoJ reply
    bool final_sent = false;        // the last point (or a halt's) has been sent
    double sent_elapsed = 0.0;      // trajectory time of the latest command
    bool halting = false;
    double rate = 1.0;              // trajectory time per wall time; falls in a halt
    double rate_step = 0.0;
    double halt_t = 0.0;

    while (running_)
    {
        // The trajectory point for this command's slot, whenever it is sent.
        // The first slot at or after the end carries the last point itself.
        double elapsed = static_cast<double>(iteration) * period;
        if (!halting && !final_sent && halt_requested_)
        {
            if (iteration == 0)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_.done = true;
                state_.halted = true;
                RCLCPP_INFO(kLogger, "Halted before the first ServoJ: nothing sent");
                break;
            }
            double tool = linearSpeed(commander_->getRealData().TCP_speed_target);
            rate_step = haltRateStep(tool, halt_decel_mm_s2_, period);
            halting = true;
            halt_t = sent_elapsed;
            RCLCPP_INFO(kLogger, "Halting at %.2f s of %.2f s: tool %.0f mm/s, to a standstill "
                        "in %.0f ms at %.0f mm/s^2", sent_elapsed, total_duration, tool,
                        period / rate_step * 1e3, halt_decel_mm_s2_);
        }
        // In a halt the trajectory's time slows to a stop along the same path
        // (the average of the rate over the period: a linear deceleration).
        if (halting)
        {
            double before = rate;
            rate = std::max(0.0, rate - rate_step);
            halt_t += 0.5 * (before + rate) * period;
            elapsed = halt_t;
        }
        bool last = elapsed >= total_duration;
        if (last)
            elapsed = total_duration;
        bool halt_end = halting && rate <= 0.0;

        // Streamed to the end, or halted: wait for the arm to settle on the
        // last command.
        if (final_sent)
        {
            std::array<double, 6> final_target;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                final_target = halting ? state_.desired : joints_deg_.back();
                if (!halting)
                {
                    state_.elapsed = total_duration;
                    state_.desired = final_target;
                }
                state_.desired_vel = {};
                state_.halted = halting;
            }
            RCLCPP_INFO(kLogger, "Trajectory streamed: %.2f s (planned %.2f s), longest gap "
                        "between ServoJ commands %.1f ms (nominal %.1f ms), %d sent more "
                        "than 1 ms late; latest wake-up %.1f ms, slowest reply %.1f ms",
                        since_start(std::chrono::steady_clock::now()), total_duration,
                        max_gap_ms, 1000.0 / servo_rate_, late_sends, max_wake_late_ms,
                        max_reply_ms);
            settle(final_target, halting);
            break;
        }

        // Interpolate target joints at current time.
        std::array<double, 6> target;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target = interpolate(elapsed);
        }

        // The feed is how the stream sees the arm; without it, stop the arm.
        int64_t age_ms = commander_->realtimeAgeMs();
        if (age_ms < 0 || age_ms > kStreamStaleMs)
        {
            int32_t stop_err = 0;
            commander_->sendServoCommand("Stop()", stop_err);
            fail("Realtime feedback stale (" + std::to_string(age_ms) +
                 " ms since the last packet) during the trajectory; the arm was stopped. "
                 "Check the TCP 30004 link");
            logRecentSends(recent, servo_t_);
            break;
        }

        // Check robot mode from real-time data.
        RealTimeData real_data = commander_->getRealData();
        uint64_t mode = real_data.robot_mode;
        std::string fault = start_fault(elapsed, real_data);
        if (mode == 9 || mode == 11)
        {
            fail("Robot entered error mode " + std::to_string(mode) +
                 (fault.empty() ? "" : "; " + fault));
            logRecentSends(recent, servo_t_);
            break;
        }
        // The controller alarms by itself within ~75 ms, and that alarm and the
        // ClearError after it are what reset its state: no Stop(), no retry.
        if (!fault.empty())
        {
            fail(fault);
            logRecentSends(recent, servo_t_);
            break;
        }
        for (size_t j = 0; j < 6; j++)
        {
            lo[j] = std::min(lo[j], target[j]);
            hi[j] = std::max(hi[j], target[j]);
        }

        // Format and send ServoJ command.
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd),
                      "ServoJ(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                      "t=%.4f,aheadtime=%.1f,gain=%.1f)",
                      target[0], target[1], target[2],
                      target[3], target[4], target[5],
                      servo_t_, aheadtime_, gain_);

        int32_t err_id = 0;
        auto send_time = std::chrono::steady_clock::now();
        bool ok = commander_->sendServoCommand(std::string(cmd), err_id);
        auto reply_time = std::chrono::steady_clock::now();
        if (iteration > 0)
            max_wake_late_ms = std::max(max_wake_late_ms, (since_start(send_time) - due) * 1e3);
        max_reply_ms = std::max(
            max_reply_ms, std::chrono::duration<double, std::milli>(reply_time - send_time).count());

        SentServo sent{elapsed,
                       std::chrono::duration<double, std::milli>(send_time - last_send).count(),
                       target, {}, {}, linearSpeed(real_data.TCP_speed_target),
                       linearSpeed(real_data.TCP_speed_actual)};
        for (size_t j = 0; j < 6; j++)
        {
            sent.actual[j] = real_data.q_actual[j];
            sent.ctrl_target[j] = real_data.q_target[j];
        }
        if (iteration > 0)
            max_gap_ms = std::max(max_gap_ms, sent.gap_ms);
        last_send = send_time;
        recent.push_back(sent);
        if (recent.size() > kRecentSends)
            recent.pop_front();

        if (!ok || err_id != 0)
        {
            // A rejection at the start is how the fault usually shows; the
            // feed tells which.
            std::string why = start_fault(elapsed, commander_->getRealData());
            fail("ServoJ failed: err_id=" + std::to_string(err_id) +
                 (why.empty() ? "" : "; " + why));
            logRecentSends(recent, servo_t_);
            break;
        }

        // Update state.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t i = 0; i < 6; i++)
            {
                state_.desired_vel[i] = iteration > 0 ? (target[i] - state_.desired[i]) / period : 0.0;
                state_.actual[i] = real_data.q_actual[i];
                state_.actual_vel[i] = real_data.qd_actual[i];
                state_.error[i] = target[i] - real_data.q_actual[i];
            }
            state_.elapsed = elapsed;
            state_.desired = target;
        }
        sent_elapsed = elapsed;
        final_sent = last || halt_end;

        iteration++;
        double sent_at = since_start(send_time);
        if (sent_at - due > kLateSend)
            late_sends++;
        due = nextDue(due, sent_at, period);
        std::this_thread::sleep_until(
            start_time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(due)));
    }

    running_ = false;
}

void TrajectoryExecutor::settle(const std::array<double, 6> &final_target, bool halted)
{
    GoalTolerance tolerance;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tolerance = tolerance_;
    }
    ArrivalCheck arrival(tolerance.position_deg);
    settling_ = true;
    struct Settled
    {
        std::atomic<bool> &flag;
        ~Settled() { flag = false; }
    } settled{settling_};
    auto start = std::chrono::steady_clock::now();
    std::array<double, 6> offset{};
    while (running_)
    {
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        int64_t age_ms = commander_->realtimeAgeMs();
        if (age_ms < 0 || age_ms > kStreamStaleMs)
        {
            fail("Realtime feedback stale (" + std::to_string(age_ms) +
                 " ms since the last packet) while the arm settled; arrival unknown. "
                 "Check the TCP 30004 link");
            return;
        }
        RealTimeData rd = commander_->getRealData();
        if (rd.robot_mode == 9 || rd.robot_mode == 11)
        {
            fail("Robot entered error mode " + std::to_string(rd.robot_mode) +
                 " while settling on the last point");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t j = 0; j < 6; j++)
            {
                offset[j] = rd.q_actual[j] - final_target[j];
                state_.actual[j] = rd.q_actual[j];
                state_.actual_vel[j] = rd.qd_actual[j];
                state_.error[j] = -offset[j];
            }
        }
        if (arrival.update(t, offset))
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.done = true;
            RCLCPP_INFO(kLogger, "Arrived %.0f ms after the last %s: offset %s deg",
                        t * 1e3, halted ? "command of the halt" : "point", joints(offset).c_str());
            return;
        }
        // A halt has no goal to reach: the arm stops where it stops.
        if (halted && t > tolerance.time_s)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.done = true;
            RCLCPP_INFO(kLogger, "Halted; %.2f s after the last command the arm is %s deg from it",
                        t, joints(offset).c_str());
            return;
        }
        if (t > tolerance.time_s)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_.goal_tolerance_violated = true;
            }
            char limit[32];
            std::snprintf(limit, sizeof(limit), "%.2f", tolerance.time_s);
            fail("The arm did not settle within " + joints(tolerance.position_deg) +
                 " deg of the last point in " + limit + " s: offset " + joints(offset) + " deg");
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
}
