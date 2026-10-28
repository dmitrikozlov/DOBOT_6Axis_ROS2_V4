#!/usr/bin/env python3
"""How far the tool travels after a FollowJointTrajectory cancel. MOTION TEST.

Through a running driver: for each --speeds, a move at that tool speed after
--accel is sent as a goal and cancelled --cruise s into the cruise. The driver
halts along the path. From the feed: the tool's travel from the moment the
cancel was sent until it stood still (within 0.05 mm from then on), and the
goal's result. Then a goal back to the start.

The move is a J1 turn (the tool point's arc), or with --line a straight line in
the base frame (e.g. --line 0,0,1: up), built with the driver's InverseKin
service at a fixed orientation; with --line the travel is measured along it.

The travel includes everything after the client's cancel: the action's cancel
round trip, the driver's reaction, the halt and the arm's own lag.

Needs a running driver and the arm enabled; it reads port 30004 alongside the
driver. J1 turns by at most --max-deg each way.

    ROS_DOMAIN_ID=77 python3 fjt_halt.py --speeds 20,40,60
    ROS_DOMAIN_ID=77 python3 fjt_halt.py --speeds 20,40,60 --line 0,0,1
"""
import argparse
import math
import os
import re
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from dobot_msgs_v4.srv import GetPose, InverseKin
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dobot_proto as dp  # noqa: E402

NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
STILL_MM = 0.05


def dur(t):
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def j1_goal(q0, profile, duration, period=0.02):
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = NAMES
    n = int(math.ceil(duration / period))
    for k in range(n + 1):
        t = min(k * period, duration)
        q = list(q0)
        q[0] += profile(t)
        g.trajectory.points.append(JointTrajectoryPoint(
            positions=[math.radians(v) for v in q], time_from_start=dur(t)))
    return g


def numbers(text):
    # A reply may echo its command's own {...}: take the first group.
    m = re.search(r'\{([^}]*)\}', text)
    return [float(x) for x in re.findall(r'-?\d+(?:\.\d+)?(?:[eE]-?\d+)?',
                                         m.group(1) if m else text)]


def call(node, client, req):
    f = client.call_async(req)
    rclpy.spin_until_future_complete(node, f, timeout_sec=3.0)
    if not f.done() or f.result() is None:
        raise SystemExit('%s: no reply' % client.srv_name)
    return f.result()


def line_goal(node, args, q0, v, step=0.5, period=0.02):
    """The straight line along args.line_u from the flange's pose, at v mm/s."""
    pose = numbers(call(node, args.getpose, GetPose.Request(user=0, tool=0)).robot_return)
    if len(pose) != 6:
        raise SystemExit('GetPose: no pose')
    length = min(v * v / args.accel + v * (args.cruise + 0.3), args.max_mm)
    grid, near = [], q0
    for k in range(int(math.ceil(length / step)) + 1):
        sk = min(k * step, length)
        req = InverseKin.Request(
            x=pose[0] + sk * args.line_u[0], y=pose[1] + sk * args.line_u[1],
            z=pose[2] + sk * args.line_u[2], rx=pose[3], ry=pose[4], rz=pose[5],
            use_joint_near='1', joint_near='{%s}' % ','.join('%.4f' % j for j in near),
            user='0', tool='0')
        res = call(node, args.ik, req)
        q = numbers(res.robot_return)
        if res.res != 0 or len(q) != 6:
            raise SystemExit('InverseKin failed at %.1f mm: %s' % (sk, res.robot_return))
        grid.append((sk, q))
        near = q
    if max(abs(a - b) for a, b in zip(grid[0][1], q0)) > 0.05:
        raise SystemExit('InverseKin at the current pose is off the arm')
    grid[0] = (0.0, list(q0))
    jump = max(max(abs(a - b) for a, b in zip(qa, qb))
               for (_, qa), (_, qb) in zip(grid, grid[1:]))
    if jump / step > 2.0:
        raise SystemExit('the line turns a joint %.2f deg per mm' % (jump / step))

    def q_at(sv):
        i = min(int(sv / step), len(grid) - 2)
        (s0, qa), (s1, qb) = grid[i], grid[i + 1]
        f = 0.0 if s1 == s0 else (sv - s0) / (s1 - s0)
        return [a + (b - a) * f for a, b in zip(qa, qb)]

    seg, duration = dp.trapezoid_1d(length, v, args.accel)
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = NAMES
    for k in range(int(math.ceil(duration / period)) + 1):
        t = min(k * period, duration)
        g.trajectory.points.append(JointTrajectoryPoint(
            positions=[math.radians(j) for j in q_at(seg(t))], time_from_start=dur(t)))
    return g, pose[:3]


def run(node, client, rt, args, v):
    pkt = rt.current(max_age=0.5)
    q0 = list(pkt['q_actual'])
    x, y = pkt['tool_vector_actual'][0], pkt['tool_vector_actual'][1]
    radius = math.hypot(x, y)                     # J1 turns about the base z axis
    t_cancel_traj = v / args.accel + args.cruise
    if args.line_u:
        goal, _ = line_goal(node, args, q0, v)
        start = list(pkt['tool_vector_actual'][:3])
    else:
        w = math.degrees(v / radius)              # deg/s of J1 for v mm/s at the tool
        a = math.degrees(args.accel / radius)
        cruise_deg = w * (args.cruise + 0.5)      # room past the cancel
        total_deg = min(w * w / a + cruise_deg, args.max_deg) * args.sign
        seg, duration = dp.trapezoid_1d(total_deg, w, a)
        goal = j1_goal(q0, seg, duration)

    i0 = rt.index()
    # The cancel is timed by the trajectory time the driver reports in its
    # feedback (the driver sends a MovJ before the stream starts).
    progress = {'t': 0.0}

    def on_feedback(msg):
        tfs = msg.feedback.desired.time_from_start
        progress['t'] = tfs.sec + tfs.nanosec / 1e9

    f = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, f)
    handle = f.result()
    if not handle.accepted:
        return {'speed': v, 'error': 'goal rejected'}
    r = handle.get_result_async()
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not r.done():
        rclpy.spin_once(node, timeout_sec=0.002)
        if progress['t'] >= t_cancel_traj:
            break
    t_cancel = dp.now()
    wall_cancel = time.time()        # the driver's log uses the same clock
    c = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, c, timeout_sec=5.0)
    rclpy.spin_until_future_complete(node, r, timeout_sec=10.0)
    res = r.result()
    time.sleep(0.5)
    packets = rt.slice(i0)

    path, prev = [], None                          # (t, mm along the tool's path)
    s = 0.0
    for t, p in packets:
        pos = p['tool_vector_actual'][:3]
        if args.line_u:
            s = sum((a - b) * u for a, b, u in zip(pos, start, args.line_u))
        elif prev is not None:
            s += math.dist(pos, prev)
        prev = pos
        path.append((t, s))
    s_cancel = dp.interp([t for t, _ in path], [s for _, s in path], t_cancel)
    s_final = path[-1][1]
    i_still = len(path) - 1
    while i_still > 0 and abs(path[i_still - 1][1] - s_final) <= STILL_MM:
        i_still -= 1
    v_at = None
    for t, p in packets:
        if t >= t_cancel:
            v_at = math.sqrt(sum(c * c for c in p['tcp_speed_actual'][:3]))
            break
    return {'speed': v, 'radius_mm': round(radius, 1), 'status': res.status,
            'message': res.result.error_string, 'tool_mm_s_at_cancel': round(v_at or 0, 1),
            'travel_mm': round(s_final - (s_cancel or 0), 3),
            'stop_ms': round((path[i_still][0] - t_cancel) * 1e3, 1),
            'cancel_at_s': round(progress['t'], 3),     # trajectory time in the last feedback
            'cancel_wall': round(wall_cancel, 4),
            'q0': q0}


def back(node, client, rt, q0, period=0.02):
    q = list(rt.current(max_age=0.5)['q_actual'])
    d = max(abs(a - b) for a, b in zip(q0, q))
    seg, duration = dp.trapezoid_1d(d, 5.0, 20.0)
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = NAMES
    for k in range(int(math.ceil(duration / period)) + 1):
        t = min(k * period, duration)
        f = seg(t) / d if d > 0 else 1.0
        g.trajectory.points.append(JointTrajectoryPoint(
            positions=[math.radians(a + (b - a) * f) for a, b in zip(q, q0)],
            time_from_start=dur(t)))
    f = client.send_goal_async(g)
    rclpy.spin_until_future_complete(node, f)
    r = f.result().get_result_async()
    rclpy.spin_until_future_complete(node, r, timeout_sec=duration + 10)
    return r.result().result.error_code


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--speeds', default='20,40,60', help='tool speeds, mm/s')
    p.add_argument('--accel', type=float, default=500.0, help='tool acceleration, mm/s^2')
    p.add_argument('--cruise', type=float, default=0.5, help='s at speed before the cancel')
    p.add_argument('--max-deg', type=float, default=12.0, help='largest J1 turn')
    p.add_argument('--sign', type=float, default=1.0, help='1 or -1: J1 direction')
    p.add_argument('--line', default='', help='straight line direction, base frame, e.g. 0,0,1')
    p.add_argument('--max-mm', type=float, default=60.0, help='longest line, mm')
    p.add_argument('--yes', action='store_true')
    args = p.parse_args()
    speeds = [float(v) for v in args.speeds.split(',')]
    args.line_u = None
    if args.line:
        u = [float(v) for v in args.line.split(',')]
        n = math.sqrt(sum(c * c for c in u))
        args.line_u = [c / n for c in u]
    if max(speeds) > 100:
        raise SystemExit('speeds over 100 mm/s')

    rt = dp.RealtimeReader(os.environ.get('DOBOT_IP', dp.DEFAULT_IP))
    rt.start()
    rt.wait_first()
    if args.line_u:
        print('Plan: straight lines of at most %g mm along [%s] (base frame) from the tool, at '
              '%s mm/s, each cancelled %.1f s into the cruise, then back.' % (
                  args.max_mm, ', '.join('%.2f' % c for c in args.line_u), args.speeds,
                  args.cruise))
    else:
        print('Plan: J1 moves of at most %g deg %s from %.2f deg, tool at %s mm/s, each '
              'cancelled %.1f s into the cruise, then back.' % (
                  args.max_deg, '+' if args.sign > 0 else '-', rt.current()['q_actual'][0],
                  args.speeds, args.cruise))
    if not args.yes and input("Clear workspace, e-stop in hand? Type 'yes': ").strip() != 'yes':
        raise SystemExit('cancelled')
    rclpy.init()
    node = Node('fjt_halt')
    client = ActionClient(node, FollowJointTrajectory,
                          '/cr5_group_controller/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=20):
        raise SystemExit('no FJT server')
    args.getpose = node.create_client(GetPose, '/dobot_bringup_ros2/srv/GetPose')
    args.ik = node.create_client(InverseKin, '/dobot_bringup_ros2/srv/InverseKin')
    if args.line_u and not (args.getpose.wait_for_service(timeout_sec=5)
                            and args.ik.wait_for_service(timeout_sec=5)):
        raise SystemExit('no GetPose / InverseKin service')

    rows = []
    for v in speeds:
        row = run(node, client, rt, args, v)
        print('cancel sent at wall %.4f' % row.get('cancel_wall', 0))
        print('%3g mm/s: status %s (%s), cancel at %s s of the trajectory, tool %s mm/s, '
              'travel %.2f mm, still after %.0f ms' % (
                  v, row.get('status'), row.get('message'), row.get('cancel_at_s'),
                  row.get('tool_mm_s_at_cancel'), row.get('travel_mm', -1), row.get('stop_ms', -1)))
        rows.append(row)
        if 'q0' in row:
            code = back(node, client, rt, row['q0'])
            if code != 0:
                print('the return failed (%s); stopping' % code)
                break
    print('\nspeed  tool at cancel  travel mm  still ms  result')
    for row in rows:
        print('%5g  %14s  %9.2f  %8.0f  %s' % (row['speed'], row.get('tool_mm_s_at_cancel'),
                                                row.get('travel_mm', -1), row.get('stop_ms', -1),
                                                row.get('message', row.get('error'))))
    node.destroy_node()
    rclpy.shutdown()
    rt.stop()


if __name__ == '__main__':
    main()
