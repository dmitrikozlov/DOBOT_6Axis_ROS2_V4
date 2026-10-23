#!/usr/bin/env python3
"""Bench check of the driver's FollowJointTrajectory server. MOTION TEST.

Sends J1 moves of +-10 deg from where the arm stands and checks each result:
a plain move, a move with the joint names reversed, an unknown joint, a start
away from the arm, a second goal during a move, a cancel mid-move, and an
arrival time limit too short to meet. Needs a running driver (the action
server) and the arm enabled, with J1 clear to turn 10 deg each way; it reads
the arm from port 30004 alongside the driver.

    ROS_DOMAIN_ID=42 python3 fjt_bench.py
"""
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
CODES = {v: k for k, v in vars(FollowJointTrajectory.Result).items()
         if k.isupper() and isinstance(v, int)}


def dur(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def goal_for(q_deg, joint, delta, duration=1.5, step=0.1, names=NAMES, offset=0.0):
    """Smooth (cosine) move of one joint, MoveIt-like points every `step` s."""
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = list(names)
    n = int(round(duration / step))
    for k in range(n + 1):
        s = 0.5 - 0.5 * math.cos(math.pi * k / n)
        q = list(q_deg)
        q[joint] += offset + delta * s
        rad = [math.radians(v) for v in q]
        order = [NAMES.index(nm) if nm in NAMES else 0 for nm in names]
        p = JointTrajectoryPoint(positions=[rad[i] for i in order], time_from_start=dur(k * step))
        g.trajectory.points.append(p)
    return g


class Bench(Node):
    def __init__(self, rt):
        super().__init__('fjt_bench')
        self.rt = rt
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/cr5_group_controller/follow_joint_trajectory')

    def q(self):
        return list(self.rt.current(max_age=0.5)['q_actual'])

    def send(self, goal):
        f = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        return f.result()

    def result(self, handle, timeout=15.0):
        f = handle.get_result_async()
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        r = f.result().result
        return CODES.get(r.error_code, r.error_code), r.error_string, f.result().status

    def run(self, name, goal, expect):
        t0 = time.monotonic()
        h = self.send(goal)
        if not h.accepted:
            print(f'{name}: REJECTED at the goal callback (expected {expect})')
            return False
        code, msg, _ = self.result(h)
        ok = code == expect
        print(f'{name}: {code} after {time.monotonic() - t0:.2f} s{": " + msg if msg else ""}'
              f'  [{"ok" if ok else "EXPECTED " + expect}]')
        return ok


def main():
    class A:
        ip = dp.DEFAULT_IP
    rt = dp.RealtimeReader(A.ip)
    rt.start()
    rt.wait_first()
    rclpy.init()
    b = Bench(rt)
    if not b.client.wait_for_server(timeout_sec=20):
        raise SystemExit('no FJT server')
    results = []
    J1 = 0
    start = b.q()

    q = b.q()
    results.append(b.run('plain +10 deg', goal_for(q, J1, 10.0), 'SUCCESSFUL'))
    print('   arm offset after success: %.4f deg' % (b.q()[J1] - (q[J1] + 10.0)))
    time.sleep(0.3)
    q = b.q()
    results.append(b.run('names reversed -10 deg', goal_for(q, J1, -10.0, names=NAMES[::-1]),
                         'SUCCESSFUL'))
    q = b.q()
    results.append(b.run('unknown joint', goal_for(q, J1, 5.0, names=NAMES[:5] + ['wrist']),
                         'INVALID_JOINTS'))
    results.append(b.run('start 2 deg away', goal_for(q, J1, 5.0, offset=2.0), 'INVALID_GOAL'))

    # A second goal while the first runs.
    q = b.q()
    h1 = b.send(goal_for(q, J1, 10.0, duration=2.0))
    time.sleep(0.5)
    h2 = b.send(goal_for(q, J1, 5.0))
    c2, m2, _ = b.result(h2)
    c1, m1, _ = b.result(h1)
    ok = c1 == 'SUCCESSFUL' and c2 == 'INVALID_GOAL'
    print(f'second goal during a move: second {c2} ({m2}); first {c1}  '
          f'[{"ok" if ok else "EXPECTED INVALID_GOAL / SUCCESSFUL"}]')
    results.append(ok)

    # Cancel mid-move.
    time.sleep(0.3)
    q = b.q()
    h = b.send(goal_for(q, J1, -10.0, duration=2.0))
    time.sleep(0.8)
    cf = h.cancel_goal_async()
    rclpy.spin_until_future_complete(b, cf)
    code, msg, status = b.result(h)
    ok = status == 5  # CANCELED
    time.sleep(1.0)
    print(f'cancel mid-move: status {status} ({msg}); arm stopped at J1 {b.q()[J1]:.3f}, '
          f'{b.q()[J1] - q[J1]:+.2f} deg of -10  [{"ok" if ok else "EXPECTED CANCELED"}]')
    results.append(ok)

    # Too short an arrival limit: streamed, but GOAL_TOLERANCE_VIOLATED.
    time.sleep(1.0)
    # It also takes the arm back to where the bench started.
    q = b.q()
    g = goal_for(q, J1, start[J1] - q[J1])
    g.goal_time_tolerance = dur(0.05)
    results.append(b.run('arrival limit 50 ms', g, 'GOAL_TOLERANCE_VIOLATED'))

    print('%d of %d as expected' % (sum(results), len(results)))
    b.destroy_node()
    rclpy.shutdown()
    rt.stop()


if __name__ == '__main__':
    main()
