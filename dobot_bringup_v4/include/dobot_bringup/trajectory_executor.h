/**
 * TrajectoryExecutor — streams ServoJ commands from a JointTrajectory.
 *
 * Resamples a MoveIt-planned trajectory to a fixed rate and sends ServoJ
 * commands directly over the dashboard TCP, bypassing the ROS service layer
 * for minimal latency.  Robot mode is monitored from the real-time feedback
 * channel; the loop aborts on ERROR (9) or COLLISION (11), or when the feed
 * stalls.  After the last point it waits for the arm to settle on it.
 */

#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <dobot_bringup/command.h>

class TrajectoryExecutor
{
public:
    // The driver's joints, in the order ServoJ takes them.
    static const std::array<std::string, 6> kJointNames;

    // When a trajectory counts as arrived: every joint within its tolerance
    // of the last point, within time_s of the end of the stream.
    struct GoalTolerance
    {
        std::array<double, 6> position_deg{0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
        double time_s = 1.0;
    };

    TrajectoryExecutor(std::shared_ptr<CRCommanderRos2> commander,
                       double servo_rate = 50.0,
                       double servo_t = 0.02,
                       double aheadtime = 50.0,
                       double gain = 500.0,
                       int rt_priority = 0);

    ~TrajectoryExecutor();

    // Load a new trajectory.  Stops current execution if running.
    // Joint positions must be in radians (ROS convention) and in the order of
    // kJointNames (see toDriverOrder); they are converted to degrees for the
    // Dobot controller internally.
    void loadTrajectory(const trajectory_msgs::msg::JointTrajectory &traj,
                        const GoalTolerance &tolerance);
    void loadTrajectory(const trajectory_msgs::msg::JointTrajectory &traj);

    // Why joint_names cannot be mapped onto kJointNames, or an empty string;
    // from[j] is then the index of kJointNames[j] in joint_names.
    static std::string jointOrder(const std::vector<std::string> &joint_names,
                                  std::array<size_t, 6> &from);

    // Copy traj into out with its points in kJointNames order. Returns an
    // empty string, or why the trajectory cannot be executed.
    static std::string toDriverOrder(const trajectory_msgs::msg::JointTrajectory &traj,
                                     trajectory_msgs::msg::JointTrajectory &out);

    // Start streaming ServoJ commands. behind_movj: a MovJ was just sent, and
    // the stream may start while the controller runs it (ServoJ queues behind
    // it); otherwise the stream waits for ENABLE.
    void start(bool behind_movj = false);

    // Stop streaming and send Dobot Stop(): the arm halts where it is. After
    // the last point has been sent, only the wait for arrival is abandoned.
    void stop();

    // Decelerate along the path to a standstill and end the stream, without
    // Stop(): the trajectory's time slows so that the tool decelerates at the
    // halt deceleration, then the arm settles on the last command. Before the
    // stream starts it ends with nothing sent; while waiting for arrival, the
    // wait ends. Returns at once; the stream ends with State::halted.
    void halt();
    void setHaltDecel(double mm_s2) { halt_decel_mm_s2_ = mm_s2; }

    // After the last point (or a halt's last command) the stream keeps
    // sending it every period: through the wait for arrival, then for hold_s
    // after arrival, so that a following trajectory can continue the same
    // ServoJ segment without a MovJ. Ends at once if hold_s is 0.
    void setHold(double s) { hold_s_ = s; }
    // Continue the segment with traj (its first point at the held point) from
    // the next period. False if the stream is not holding: the next
    // trajectory then needs a new segment.
    bool append(const trajectory_msgs::msg::JointTrajectory &traj,
                const GoalTolerance &tolerance);
    bool isHolding() const { return holding_; }
    // The point being held (deg), valid while isHolding().
    std::array<double, 6> heldPoint() const;
    // Stop holding: the stream ends without Stop(), the arm being at rest.
    void endHold();

    // How much the trajectory's time rate (1 = as planned) falls per period in
    // a halt from tool_speed (mm/s) at decel (mm/s^2): to 0 over
    // tool_speed / decel, and over no less than one period.
    static double haltRateStep(double tool_speed_mm_s, double decel_mm_s2, double period);

    bool isRunning() const;

    struct State
    {
        double elapsed = 0.0;
        double total_duration = 0.0;
        std::array<double, 6> desired{};  // current target (degrees)
        std::array<double, 6> actual{};   // from real-time feedback (degrees)
        std::array<double, 6> error{};    // desired - actual (degrees)
        std::array<double, 6> desired_vel{};  // deg/s, of the commanded target
        std::array<double, 6> actual_vel{};   // deg/s, measured
        bool done = false;
        std::string error_msg;            // non-empty on failure
        bool goal_tolerance_violated = false;  // streamed, but did not arrive
        bool halted = false;              // ended by halt()
    };

    State getState() const;

    // Query the interpolated joint position (degrees) at time t.
    // Thread-safe — acquires mutex internally.  Useful for testing
    // and for external code that needs to preview the trajectory.
    std::array<double, 6> interpolateAt(double t) const;

    static constexpr double kPi = 3.14159265358979323846;
    static inline double rad2Deg(double rad) { return rad * 180.0 / kPi; }
    static inline double deg2Rad(double deg) { return deg * kPi / 180.0; }

    // When the next ServoJ is due (s from the stream's start), given when the
    // last one was due and when it was sent: one period later, or one period
    // after the send if that was more than kLateSend late. Each command
    // carries the trajectory point for its own slot (index × period), so a
    // late send delays the rest of the stream instead of being made up. The
    // next command never follows a late one early: on the CR5, a ServoJ 10 ms
    // early made the controller's target speed surge 30 % and raised alarm 225.
    static double nextDue(double due, double sent, double period);

    // How late a send may be and still keep the schedule (s): wake-up jitter.
    static constexpr double kLateSend = 0.001;

    // At a stream's start the controller's target follows the commands, behind
    // them, so it stays within the range of the commands sent so far and its
    // own target before the stream ([lo, hi] per joint, deg). On the CR5 it
    // can instead leave that range at once: every joint toward 0 by the same
    // fraction of its angle, or one joint against the commanded direction
    // (after Stop() on an arrived stream). Returns what the target did, or an
    // empty string while it stays within kStartFaultDeg of the range. start:
    // the stream's first point.
    static std::string startFault(const std::array<double, 6> &start,
                                  const std::array<double, 6> &lo,
                                  const std::array<double, 6> &hi,
                                  const std::array<double, 6> &ctrl_target);
    static constexpr double kStartFaultDeg = 0.05;
    static constexpr double kStartWatchS = 0.5;   // how long into a stream it is checked

    // Arrival: true once every joint's offset from the last point has stayed
    // within its tolerance for kSettleS of feed time.
    class ArrivalCheck
    {
    public:
        explicit ArrivalCheck(const std::array<double, 6> &tolerance_deg)
            : tolerance_(tolerance_deg) {}
        bool update(double t, const std::array<double, 6> &offset_deg);

    private:
        std::array<double, 6> tolerance_;
        double within_since_ = -1.0;
    };
    static constexpr double kSettleS = 0.05;

private:
    void servoLoop();
    void fail(const std::string &msg);
    std::array<double, 6> interpolate(double t) const;  // caller must hold mutex_

    std::shared_ptr<CRCommanderRos2> commander_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> settling_{false};  // streamed: arrival wait or hold
    std::atomic<bool> holding_{false};   // arrived; a trajectory may be appended
    double hold_s_ = 1.0;
    // A trajectory append() passed to the stream; guarded by mutex_.
    bool pending_ = false;
    std::vector<double> pending_times_;
    std::vector<std::array<double, 6>> pending_joints_;
    GoalTolerance pending_tol_;
    bool behind_movj_ = false;           // start(): stream at once behind a MovJ
    std::atomic<bool> halt_requested_{false};
    double halt_decel_mm_s2_ = 2000.0;   // tool deceleration of a halt
    mutable std::mutex mutex_;  // guards trajectory data and state_

    // Trajectory data — stored in degrees (Dobot convention)
    std::vector<double> timestamps_;
    std::vector<std::array<double, 6>> joints_deg_;
    GoalTolerance tolerance_;

    // Configuration
    double servo_rate_;
    double servo_t_;
    double aheadtime_;
    double gain_;
    int rt_priority_;               // SCHED_FIFO priority of the servo thread; 0: none

    // Current execution state
    State state_;
};

#endif  // TRAJECTORY_EXECUTOR_H
