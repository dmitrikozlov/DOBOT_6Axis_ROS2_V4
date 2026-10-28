#!/usr/bin/env python3
"""How far the tool travels after a FollowJointTrajectory cancel. MOTION TEST.

Through a running driver: for each --speeds, a J1 move whose tool point (the
feed's tool position) travels its arc at that speed after --accel, is sent as
a goal and cancelled --cruise s into the cruise. The driver halts along the
path. From the feed: the tool's travel along its path from the moment the
cancel was sent until it stood still (within 0.05 mm from then on), and the
goal's result. Then a goal back to the start.

The travel includes everything after the client's cancel: the action's cancel
round trip, the driver's reaction, the halt and the arm's own lag.

Needs a running driver and the arm enabled; it reads port 30004 alongside the
driver. J1 turns by at most --max-deg each way.

    ROS_DOMAIN_ID=77 python3 fjt_halt.py --speeds 20,40,60
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
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


def run(node, client, rt, args, v):
    pkt = rt.current(max_age=0.5)
    q0 = list(pkt['q_actual'])
    x, y = pkt['tool_vector_actual'][0], pkt['tool_vector_actual'][1]
    radius = math.hypot(x, y)                     # J1 turns about the base z axis
    w = math.degrees(v / radius)                  # deg/s of J1 for v mm/s at the tool
    a = math.degrees(args.accel / radius)
    t_acc = w / a
    cruise_deg = w * (args.cruise + 0.5)          # room past the cancel
    total_deg = min(a * t_acc ** 2 + cruise_deg, args.max_deg) * args.sign
    seg, duration = dp.trapezoid_1d(total_deg, w, a)
    goal = j1_goal(q0, seg, duration)
    t_cancel_traj = t_acc + args.cruise

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
        if prev is not None:
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


def back(node, client, rt, q0):
    q = list(rt.current(max_age=0.5)['q_actual'])
    d = q0[0] - q[0]
    seg, duration = dp.trapezoid_1d(d, 5.0, 20.0)
    f = client.send_goal_async(j1_goal(q, seg, duration))
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
    p.add_argument('--yes', action='store_true')
    args = p.parse_args()
    speeds = [float(v) for v in args.speeds.split(',')]
    if max(speeds) > 100:
        raise SystemExit('speeds over 100 mm/s')

    rt = dp.RealtimeReader(os.environ.get('DOBOT_IP', dp.DEFAULT_IP))
    rt.start()
    rt.wait_first()
    print('Plan: J1 moves of at most %g deg %s from %.2f deg, tool at %s mm/s, each cancelled '
          '%.1f s into the cruise, then back.' % (
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
