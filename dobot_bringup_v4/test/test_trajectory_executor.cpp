/**
 * Unit tests for TrajectoryExecutor.
 *
 * These tests exercise interpolation, rad/deg conversion, trajectory loading,
 * and edge cases.  They do NOT require a robot connection — the executor is
 * constructed with a nullptr commander and never started.
 */

#include <gtest/gtest.h>
#include <dobot_bringup/trajectory_executor.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cmath>

using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr double kDeg2Rad = M_PI / 180.0;
static constexpr double kRad2Deg = 180.0 / M_PI;
static constexpr double kTol = 1e-6;  // tolerance for floating-point comparison

// Build a JointTrajectoryPoint with 6 joint positions (radians) and a timestamp.
static JointTrajectoryPoint makePoint(
    std::array<double, 6> joints_rad, double time_sec)
{
    JointTrajectoryPoint pt;
    for (auto j : joints_rad)
        pt.positions.push_back(j);
    pt.time_from_start.sec = static_cast<int32_t>(time_sec);
    pt.time_from_start.nanosec =
        static_cast<uint32_t>((time_sec - std::floor(time_sec)) * 1e9);
    return pt;
}

// Build a simple trajectory with the given waypoints.
static JointTrajectory makeTrajectory(
    const std::vector<std::pair<std::array<double, 6>, double>> &waypoints)
{
    JointTrajectory traj;
    traj.joint_names = {"joint1", "joint2", "joint3",
                        "joint4", "joint5", "joint6"};
    for (const auto &wp : waypoints)
        traj.points.push_back(makePoint(wp.first, wp.second));
    return traj;
}

// Create a TrajectoryExecutor with no commander (for interpolation tests only).
static std::unique_ptr<TrajectoryExecutor> makeExecutor()
{
    return std::make_unique<TrajectoryExecutor>(nullptr);
}

// ---------------------------------------------------------------------------
// rad2Deg / deg2Rad
// ---------------------------------------------------------------------------

TEST(TrajectoryExecutorConversion, Rad2Deg)
{
    EXPECT_NEAR(TrajectoryExecutor::rad2Deg(0.0), 0.0, kTol);
    EXPECT_NEAR(TrajectoryExecutor::rad2Deg(M_PI), 180.0, kTol);
    EXPECT_NEAR(TrajectoryExecutor::rad2Deg(-M_PI / 2), -90.0, kTol);
    EXPECT_NEAR(TrajectoryExecutor::rad2Deg(2 * M_PI), 360.0, kTol);
}

TEST(TrajectoryExecutorConversion, Deg2Rad)
{
    EXPECT_NEAR(TrajectoryExecutor::deg2Rad(0.0), 0.0, kTol);
    EXPECT_NEAR(TrajectoryExecutor::deg2Rad(180.0), M_PI, kTol);
    EXPECT_NEAR(TrajectoryExecutor::deg2Rad(-90.0), -M_PI / 2, kTol);
}

TEST(TrajectoryExecutorConversion, Roundtrip)
{
    for (double deg = -360.0; deg <= 360.0; deg += 17.3)
    {
        double result = TrajectoryExecutor::rad2Deg(
            TrajectoryExecutor::deg2Rad(deg));
        EXPECT_NEAR(result, deg, kTol) << "roundtrip failed for " << deg;
    }
}

// ---------------------------------------------------------------------------
// loadTrajectory + getState
// ---------------------------------------------------------------------------

TEST(TrajectoryExecutorLoad, EmptyTrajectory)
{
    auto exec = makeExecutor();
    JointTrajectory traj;
    exec->loadTrajectory(traj);

    auto state = exec->getState();
    EXPECT_EQ(state.total_duration, 0.0);
    EXPECT_FALSE(exec->isRunning());
}

TEST(TrajectoryExecutorLoad, SinglePoint)
{
    auto exec = makeExecutor();
    auto traj = makeTrajectory({
        {{10 * kDeg2Rad, 20 * kDeg2Rad, 30 * kDeg2Rad,
          40 * kDeg2Rad, 50 * kDeg2Rad, 60 * kDeg2Rad},
         0.0},
    });
    exec->loadTrajectory(traj);

    auto state = exec->getState();
    EXPECT_NEAR(state.total_duration, 0.0, kTol);
}

TEST(TrajectoryExecutorLoad, DurationFromLastPoint)
{
    auto exec = makeExecutor();
    auto traj = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{0, 0, 0, 0, 0, 0}, 1.5},
        {{0, 0, 0, 0, 0, 0}, 3.7},
    });
    exec->loadTrajectory(traj);

    auto state = exec->getState();
    EXPECT_NEAR(state.total_duration, 3.7, kTol);
}

TEST(TrajectoryExecutorLoad, RadiansConvertedToDegrees)
{
    auto exec = makeExecutor();
    // Load a single point at pi/2 radians (= 90 degrees)
    auto traj = makeTrajectory({
        {{M_PI / 2, -M_PI / 4, M_PI, 0, -M_PI / 6, M_PI / 3}, 0.0},
    });
    exec->loadTrajectory(traj);

    // interpolateAt should return degrees
    auto joints = exec->interpolateAt(0.0);
    EXPECT_NEAR(joints[0], 90.0, kTol);
    EXPECT_NEAR(joints[1], -45.0, kTol);
    EXPECT_NEAR(joints[2], 180.0, kTol);
    EXPECT_NEAR(joints[3], 0.0, kTol);
    EXPECT_NEAR(joints[4], -30.0, kTol);
    EXPECT_NEAR(joints[5], 60.0, kTol);
}

TEST(TrajectoryExecutorLoad, StartNotDone)
{
    auto exec = makeExecutor();
    auto traj = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{1, 1, 1, 1, 1, 1}, 2.0},
    });
    exec->loadTrajectory(traj);

    auto state = exec->getState();
    EXPECT_FALSE(state.done);
    EXPECT_TRUE(state.error_msg.empty());
}

TEST(TrajectoryExecutorLoad, ReloadClearsState)
{
    auto exec = makeExecutor();

    // Load first trajectory
    auto traj1 = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{1, 1, 1, 1, 1, 1}, 5.0},
    });
    exec->loadTrajectory(traj1);
    EXPECT_NEAR(exec->getState().total_duration, 5.0, kTol);

    // Load second trajectory — should replace first
    auto traj2 = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{2, 2, 2, 2, 2, 2}, 2.0},
    });
    exec->loadTrajectory(traj2);
    EXPECT_NEAR(exec->getState().total_duration, 2.0, kTol);
}

// ---------------------------------------------------------------------------
// Interpolation
// ---------------------------------------------------------------------------

TEST(TrajectoryExecutorInterp, EmptyReturnsZeros)
{
    auto exec = makeExecutor();
    JointTrajectory traj;
    exec->loadTrajectory(traj);

    auto joints = exec->interpolateAt(0.0);
    for (size_t i = 0; i < 6; i++)
        EXPECT_EQ(joints[i], 0.0);
}

TEST(TrajectoryExecutorInterp, SinglePointAlwaysReturnsSame)
{
    auto exec = makeExecutor();
    auto traj = makeTrajectory({
        {{45 * kDeg2Rad, 0, 0, 0, 0, 0}, 0.0},
    });
    exec->loadTrajectory(traj);

    // Before, at, and after the point — should all return same value
    for (double t : {-1.0, 0.0, 0.5, 100.0})
    {
        auto joints = exec->interpolateAt(t);
        EXPECT_NEAR(joints[0], 45.0, kTol) << "at t=" << t;
    }
}

TEST(TrajectoryExecutorInterp, TwoPointLinear)
{
    auto exec = makeExecutor();
    // From 0 deg to 90 deg over 2 seconds
    auto traj = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{90 * kDeg2Rad, 0, 0, 0, 0, 0}, 2.0},
    });
    exec->loadTrajectory(traj);

    // At start
    EXPECT_NEAR(exec->interpolateAt(0.0)[0], 0.0, kTol);

    // At end
    EXPECT_NEAR(exec->interpolateAt(2.0)[0], 90.0, kTol);

    // At midpoint
    EXPECT_NEAR(exec->interpolateAt(1.0)[0], 45.0, kTol);

    // At quarter
    EXPECT_NEAR(exec->interpolateAt(0.5)[0], 22.5, kTol);

    // At three-quarters
    EXPECT_NEAR(exec->interpolateAt(1.5)[0], 67.5, kTol);
}

TEST(TrajectoryExecutorInterp, ThreePointSegments)
{
    auto exec = makeExecutor();
    // Segment 1: 0→60 deg over 0–1s
    // Segment 2: 60→30 deg over 1–3s
    auto traj = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{60 * kDeg2Rad, 0, 0, 0, 0, 0}, 1.0},
        {{30 * kDeg2Rad, 0, 0, 0, 0, 0}, 3.0},
    });
    exec->loadTrajectory(traj);

    // Segment 1 midpoint (t=0.5): expect 30 deg
    EXPECT_NEAR(exec->interpolateAt(0.5)[0], 30.0, kTol);

    // Boundary (t=1.0): expect 60 deg
    EXPECT_NEAR(exec->interpolateAt(1.0)[0], 60.0, kTol);

    // Segment 2 midpoint (t=2.0): expect 45 deg
    EXPECT_NEAR(exec->interpolateAt(2.0)[0], 45.0, kTol);

    // End (t=3.0): expect 30 deg
    EXPECT_NEAR(exec->interpolateAt(3.0)[0], 30.0, kTol);
}

TEST(TrajectoryExecutorInterp, AllJointsInterpolated)
{
    auto exec = makeExecutor();
    std::array<double, 6> start_deg = {10, 20, 30, 40, 50, 60};
    std::array<double, 6> end_deg = {110, 120, 130, 140, 150, 160};

    std::array<double, 6> start_rad, end_rad;
    for (size_t i = 0; i < 6; i++)
    {
        start_rad[i] = start_deg[i] * kDeg2Rad;
        end_rad[i] = end_deg[i] * kDeg2Rad;
    }

    auto traj = makeTrajectory({
        {start_rad, 0.0},
        {end_rad, 4.0},
    });
    exec->loadTrajectory(traj);

    // At midpoint (t=2.0), each joint should be halfway
    auto joints = exec->interpolateAt(2.0);
    for (size_t i = 0; i < 6; i++)
    {
        double expected = (start_deg[i] + end_deg[i]) / 2.0;
        EXPECT_NEAR(joints[i], expected, kTol) << "joint " << i;
    }
}

TEST(TrajectoryExecutorInterp, NegativeAngles)
{
    auto exec = makeExecutor();
    auto traj = makeTrajectory({
        {{-90 * kDeg2Rad, 0, 0, 0, 0, 0}, 0.0},
        {{90 * kDeg2Rad, 0, 0, 0, 0, 0}, 2.0},
    });
    exec->loadTrajectory(traj);

    // Midpoint: should be 0 degrees
    EXPECT_NEAR(exec->interpolateAt(1.0)[0], 0.0, kTol);

    // Quarter: should be -45 degrees
    EXPECT_NEAR(exec->interpolateAt(0.5)[0], -45.0, kTol);
}

// ---------------------------------------------------------------------------
// Clamping: t before start / after end
// ---------------------------------------------------------------------------

TEST(TrajectoryExecutorInterp, BeforeStartClampsToFirst)
{
    auto exec = makeExecutor();
    auto traj = makeTrajectory({
        {{45 * kDeg2Rad, 0, 0, 0, 0, 0}, 0.5},
        {{90 * kDeg2Rad, 0, 0, 0, 0, 0}, 2.0},
    });
    exec->loadTrajectory(traj);

    // t=0 is before the first timestamp (0.5) — should return first point
    auto joints = exec->interpolateAt(0.0);
    EXPECT_NEAR(joints[0], 45.0, kTol);
}

TEST(TrajectoryExecutorInterp, AfterEndClampsToLast)
{
    auto exec = makeExecutor();
    auto traj = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{90 * kDeg2Rad, 0, 0, 0, 0, 0}, 2.0},
    });
    exec->loadTrajectory(traj);

    // t=100 is way past the end — should return last point
    auto joints = exec->interpolateAt(100.0);
    EXPECT_NEAR(joints[0], 90.0, kTol);
}

// ---------------------------------------------------------------------------
// Lifecycle (no commander — just state machine checks)
// ---------------------------------------------------------------------------

TEST(TrajectoryExecutorLifecycle, NotRunningByDefault)
{
    auto exec = makeExecutor();
    EXPECT_FALSE(exec->isRunning());
}

TEST(TrajectoryExecutorLifecycle, StartEmptyTrajectoryMarksDone)
{
    auto exec = makeExecutor();
    JointTrajectory traj;
    exec->loadTrajectory(traj);
    exec->start();

    // Empty trajectory → done immediately, never actually running
    EXPECT_FALSE(exec->isRunning());
    EXPECT_TRUE(exec->getState().done);
}

TEST(TrajectoryExecutorLifecycle, SubsecondTimestamp)
{
    auto exec = makeExecutor();
    // Test that fractional seconds are parsed correctly
    auto traj = makeTrajectory({
        {{0, 0, 0, 0, 0, 0}, 0.0},
        {{M_PI, 0, 0, 0, 0, 0}, 0.456},
    });
    exec->loadTrajectory(traj);

    EXPECT_NEAR(exec->getState().total_duration, 0.456, 1e-3);
}

// ---------------------------------------------------------------------------
// Many-segment trajectory (stress test for segment finding)
// ---------------------------------------------------------------------------

// --- Send schedule: index timing -------------------------------------------

TEST(TrajectoryExecutorSchedule, OnTimeKeepsTheGrid)
{
    EXPECT_DOUBLE_EQ(TrajectoryExecutor::nextDue(0.040, 0.0405, 0.02), 0.060);
}

TEST(TrajectoryExecutorSchedule, LateUnderHalfAPeriodMovesTheGrid)
{
    // 9 ms late: the next command waits a full period, not 11 ms
    EXPECT_DOUBLE_EQ(TrajectoryExecutor::nextDue(0.040, 0.049, 0.02), 0.069);
}

TEST(TrajectoryExecutorSchedule, LateSendMovesTheGridInsteadOfBunching)
{
    // 40 ms late: the next command is one period after the send, not at once
    EXPECT_DOUBLE_EQ(TrajectoryExecutor::nextDue(0.040, 0.080, 0.02), 0.100);
}

TEST(TrajectoryExecutorSchedule, NextFollowsTheLastSendByAlmostAPeriod)
{
    const double period = 0.02;
    for (double late : {0.0, 0.0005, 0.001, 0.0015, 0.005, 0.0101, 0.03, 0.2})
        EXPECT_GE(TrajectoryExecutor::nextDue(1.0, 1.0 + late, period) - (1.0 + late),
                  period - TrajectoryExecutor::kLateSend - 1e-12)
            << "late " << late;
}

TEST(TrajectoryExecutorInterp, ManySegments)
{
    auto exec = makeExecutor();
    // 100 points, joint0 goes from 0 to 99 degrees linearly
    std::vector<std::pair<std::array<double, 6>, double>> waypoints;
    for (int i = 0; i < 100; i++)
    {
        std::array<double, 6> joints{};
        joints[0] = static_cast<double>(i) * kDeg2Rad;
        waypoints.push_back({joints, static_cast<double>(i) * 0.1});
    }
    exec->loadTrajectory(makeTrajectory(waypoints));

    // Check a few points
    EXPECT_NEAR(exec->interpolateAt(0.0)[0], 0.0, kTol);
    EXPECT_NEAR(exec->interpolateAt(4.95)[0], 49.5, 0.01);
    EXPECT_NEAR(exec->interpolateAt(9.9)[0], 99.0, kTol);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// --- Goal checks: joint order -----------------------------------------------

TEST(TrajectoryExecutorGoal, JointsReorderedByName)
{
    JointTrajectory traj;
    traj.joint_names = {"joint6", "joint5", "joint4", "joint3", "joint2", "joint1"};
    traj.points.push_back(makePoint({6, 5, 4, 3, 2, 1}, 0.0));
    traj.points.push_back(makePoint({60, 50, 40, 30, 20, 10}, 1.0));

    JointTrajectory out;
    EXPECT_EQ(TrajectoryExecutor::toDriverOrder(traj, out), "");
    ASSERT_EQ(out.points.size(), 2u);
    EXPECT_EQ(out.joint_names.front(), "joint1");
    for (size_t j = 0; j < 6; j++)
    {
        EXPECT_DOUBLE_EQ(out.points[0].positions[j], static_cast<double>(j + 1));
        EXPECT_DOUBLE_EQ(out.points[1].positions[j], 10.0 * static_cast<double>(j + 1));
    }
}

TEST(TrajectoryExecutorGoal, UnknownOrMissingJointRefused)
{
    std::array<size_t, 6> from{};
    EXPECT_NE(TrajectoryExecutor::jointOrder(
                  {"joint1", "joint2", "joint3", "joint4", "joint5", "wrist"}, from)
                  .find("joint6"),
              std::string::npos);
    EXPECT_NE(TrajectoryExecutor::jointOrder({"joint1", "joint2"}, from), "");
    EXPECT_EQ(TrajectoryExecutor::jointOrder(
                  {"joint2", "joint1", "joint3", "joint4", "joint5", "joint6"}, from),
              "");
    EXPECT_EQ(from[0], 1u);
    EXPECT_EQ(from[1], 0u);
}

TEST(TrajectoryExecutorGoal, BadPointsRefused)
{
    JointTrajectory out;
    EXPECT_NE(TrajectoryExecutor::toDriverOrder(makeTrajectory({}), out), "");

    auto backwards = makeTrajectory({{{0, 0, 0, 0, 0, 0}, 1.0}, {{0, 0, 0, 0, 0, 0}, 0.5}});
    EXPECT_NE(TrajectoryExecutor::toDriverOrder(backwards, out).find("earlier"),
              std::string::npos);

    auto short_point = makeTrajectory({{{0, 0, 0, 0, 0, 0}, 0.0}});
    short_point.points[0].positions.pop_back();
    EXPECT_NE(TrajectoryExecutor::toDriverOrder(short_point, out), "");

    auto nan = makeTrajectory({{{0, 0, std::nan(""), 0, 0, 0}, 0.0}});
    EXPECT_NE(TrajectoryExecutor::toDriverOrder(nan, out).find("non-finite"),
              std::string::npos);
}

// --- Arrival -----------------------------------------------------------------

TEST(TrajectoryExecutorArrival, ArrivesAfterStayingWithinTolerance)
{
    TrajectoryExecutor::ArrivalCheck check({0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
    EXPECT_FALSE(check.update(0.000, {0.05, 0, 0, 0, 0, 0}));
    EXPECT_FALSE(check.update(0.010, {0.005, 0, 0, 0, 0, 0}));
    EXPECT_FALSE(check.update(0.040, {-0.008, 0, 0, 0, 0, 0}));
    EXPECT_TRUE(check.update(0.010 + TrajectoryExecutor::kSettleS, {0.002, 0, 0, 0, 0, 0}));
}

TEST(TrajectoryExecutorArrival, LeavingToleranceRestartsTheWait)
{
    TrajectoryExecutor::ArrivalCheck check({0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
    EXPECT_FALSE(check.update(0.00, {0, 0, 0, 0, 0, 0}));
    EXPECT_FALSE(check.update(0.04, {0, 0, 0, 0, 0.02, 0}));  // overshoot on joint5
    EXPECT_FALSE(check.update(0.06, {0, 0, 0, 0, 0, 0}));
    EXPECT_FALSE(check.update(0.10, {0, 0, 0, 0, 0, 0}));
    EXPECT_TRUE(check.update(0.06 + TrajectoryExecutor::kSettleS, {0, 0, 0, 0, 0, 0}));
}

TEST(TrajectoryExecutorArrival, PerJointTolerance)
{
    TrajectoryExecutor::ArrivalCheck check({0.01, 0.01, 0.01, 0.01, 0.01, 1e9});
    EXPECT_FALSE(check.update(0.0, {0, 0, 0, 0, 0, 5.0}));
    EXPECT_TRUE(check.update(TrajectoryExecutor::kSettleS, {0, 0, 0, 0, 0, 5.0}));
}

// --- Start-of-stream fault -----------------------------------------------------

TEST(TrajectoryExecutorStartFault, FollowingTheCommandsIsNoFault)
{
    // Commands J1 -90.50 -> -90.39; the controller's target lags between them.
    std::array<double, 6> start{-90.50, 45.68, -95.11, -15.44, 90.49, 0.41};
    std::array<double, 6> lo = start, hi = start;
    hi[0] = -90.39;
    std::array<double, 6> ctrl = start;
    ctrl[0] = -90.45;
    EXPECT_EQ(TrajectoryExecutor::startFault(start, lo, hi, ctrl), "");
    ctrl[0] = -90.39 + 0.04;   // within kStartFaultDeg of the range
    EXPECT_EQ(TrajectoryExecutor::startFault(start, lo, hi, ctrl), "");
}

TEST(TrajectoryExecutorStartFault, EveryJointTowardZero)
{
    // Bench 2026-10-23 (case 4): 25 ms in, every joint 0.1562 % toward 0.
    std::array<double, 6> start{-35.733, 43.658, -95.110, -15.389, 90.493, 0.414};
    std::array<double, 6> lo = start, hi = start;
    lo[0] = -35.745;                                   // commands so far
    std::array<double, 6> ctrl{-35.677, 43.590, -94.964, -15.365, 90.352, 0.413};
    std::string what = TrajectoryExecutor::startFault(start, lo, hi, ctrl);
    EXPECT_NE(what.find("every joint moved toward 0"), std::string::npos) << what;
    EXPECT_NE(what.find("0.15"), std::string::npos) << what;
}

TEST(TrajectoryExecutorStartFault, OneJointAgainstTheCommands)
{
    // Bench 2026-10-22 (lurch after Stop()): J1 commanded up from -99.147,
    // the controller's target went down.
    std::array<double, 6> start{-99.147, 45.682, -95.105, -15.441, 90.493, 0.413};
    std::array<double, 6> lo = start, hi = start;
    hi[0] = -99.114;
    std::array<double, 6> ctrl = start;
    ctrl[0] = -99.410;
    std::string what = TrajectoryExecutor::startFault(start, lo, hi, ctrl);
    EXPECT_NE(what.find("joint1 moved to -99.410"), std::string::npos) << what;
    EXPECT_NE(what.find("0.26 deg outside"), std::string::npos) << what;
}

