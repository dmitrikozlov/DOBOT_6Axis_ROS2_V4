#!/usr/bin/env python3
"""The same joint move by the controller's MovJ and by a FollowJointTrajectory goal. MOTION TEST.

Through a running driver (its MovJ service and its FJT action), for each --speeds
(MovJ v=, %): a MovJ by --delta from where the arm stands and back, then the
same move as an FJT goal and back. The FJT goal is a synchronised trapezoid in
joint space (all joints finishing together, as MovJ does) with the peak speed
and acceleration the MovJ's leading joint reached, so the two differ only in
who shapes and runs the motion.

For each move, from the realtime feed: its duration; the leading joint's peak
speed; its speed ripple in the cruise (rms about a 0.25 s moving average, % of
the cruise speed) for the controller's target and for the arm; the peak joint
acceleration and jerk of the arm (deg/s^2, deg/s^3, from qd_actual smoothed
over 5 packets); and the tool's peak speed.

    ROS_DOMAIN_ID=77 python3 movj_vs_fjt.py --speeds 5
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from dobot_msgs_v4.srv import MovJ
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dobot_proto as dp  # noqa: E402

NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def dur(t):
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def wait_motion(rt, start_timeout=3.0, timeout=60.0):
    """Until the controller has run a queued motion: RUNNING, then ENABLE."""
    t0 = dp.now()
    while dp.now() - t0 < start_timeout and rt.current(max_age=0.5)['robot_mode'] != 7:
        time.sleep(0.005)
    while dp.now() - t0 < timeout:
        mode = rt.current(max_age=0.5)['robot_mode']
        if mode == 5:
            return True
        if mode in dp.ERROR_MODES:
            return False
        time.sleep(0.005)
    return False


def smooth(seq, n):
    out = []
    for i in range(len(seq)):
        lo, hi = max(0, i - n // 2), min(len(seq), i + n // 2 + 1)
        out.append(sum(seq[lo:hi]) / (hi - lo))
    return out


def derivative(seq, dt):
    return [(seq[min(i + 1, len(seq) - 1)] - seq[max(i - 1, 0)]) / (2 * dt) for i in range(len(seq))]


def measure(packets, lead, period):
    qdt = [p['qd_target'][lead] for _, p in packets]
    qda = [p['qd_actual'][lead] for _, p in packets]
    moving = [i for i, v in enumerate(qdt) if abs(v) > 0.05 * max(abs(x) for x in qdt)]
    if not moving:
        return None
    a, b = moving[0], moving[-1]
    peak = max(abs(v) for v in qdt[a:b + 1])
    cruise = [i for i in range(a, b + 1) if abs(qdt[i]) > 0.9 * peak]
    out = {'duration_s': round((b - a) * period, 2), 'peak_deg_s': round(peak, 2)}
    for name, sig in (('target', qdt), ('arm', qda)):
        if len(cruise) > 40:
            c0, c1 = cruise[0] + 8, cruise[-1] - 8
            ref = smooth(sig, 31)
            dev = [sig[i] - ref[i] for i in range(c0, c1)]
            rms = math.sqrt(sum(d * d for d in dev) / len(dev))
            out['ripple_%s_pct' % name] = round(100.0 * rms / peak, 2)
        else:
            out['ripple_%s_pct' % name] = float('nan')
    # The arm's acceleration and jerk, every joint, from qd_actual smoothed over 5 packets.
    # The peak, and the 99th percentile over the move (one brief spike does not set it).
    accs, jerks = [], []
    for j in range(6):
        v = smooth([p['qd_actual'][j] for _, p in packets[a:b + 1]], 5)
        acc = smooth(derivative(v, period), 5)
        accs.extend(abs(x) for x in acc)
        jerks.extend(abs(x) for x in derivative(acc, period))
    accs.sort()
    jerks.sort()
    out['peak_acc_deg_s2'] = round(accs[-1], 1)
    out['peak_jerk_deg_s3'] = round(jerks[-1], 0)
    out['p99_acc_deg_s2'] = round(accs[int(0.99 * (len(accs) - 1))], 1)
    out['p99_jerk_deg_s3'] = round(jerks[int(0.99 * (len(jerks) - 1))], 0)
    out['tool_peak_mm_s'] = round(max(math.sqrt(sum(c * c for c in p['tcp_speed_actual'][:3]))
                                      for _, p in packets[a:b + 1]), 1)
    # The leading joint's acceleration from the controller's target: for the FJT profile.
    vt = smooth(qdt[a:b + 1], 5)
    out['lead_acc_deg_s2'] = max(abs(x) for x in smooth(derivative(vt, period), 5))
    return out


def scurve_1d(distance, vmax, amax):
    """Distance along a profile whose speed ramps up and down as a half cosine
    (acceleration rises and falls smoothly, peak amax). Returns (f(t), duration)."""
    d = abs(distance)
    sign = 1.0 if distance >= 0 else -1.0
    ramp = math.pi * vmax / (2 * amax)               # the ramp's time at vmax
    if vmax * ramp > d:                              # no cruise: a lower peak speed
        vmax = math.sqrt(2 * amax * d / math.pi)
        ramp = math.pi * vmax / (2 * amax)
    d_ramp = vmax * ramp / 2
    cruise = (d - 2 * d_ramp) / vmax
    total = 2 * ramp + cruise

    def up(t):
        return vmax / 2 * (t - ramp / math.pi * math.sin(math.pi * t / ramp))

    def f(t):
        t = min(max(t, 0.0), total)
        if t < ramp:
            s = up(t)
        elif t < ramp + cruise:
            s = d_ramp + vmax * (t - ramp)
        else:
            s = d - up(total - t)
        return sign * s
    return f, total


class Bench(Node):
    def __init__(self):
        super().__init__('movj_vs_fjt')
        self.movj = self.create_client(MovJ, '/dobot_bringup_ros2/srv/MovJ')
        self.fjt = ActionClient(self, FollowJointTrajectory,
                                '/cr5_group_controller/follow_joint_trajectory')

    def call_movj(self, q, v, a):
        req = MovJ.Request(mode=True, a=q[0], b=q[1], c=q[2], d=q[3], e=q[4], f=q[5],
                           param_value=['v=%g' % v, 'a=%g' % a])
        f = self.movj.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return f.result()

    def run_fjt(self, q_from, q_to, vmax, amax, profile, period=0.02):
        lead = max(range(6), key=lambda j: abs(q_to[j] - q_from[j]))
        span = q_to[lead] - q_from[lead]
        shape = scurve_1d if profile == 'scurve' else dp.trapezoid_1d
        seg, duration = shape(abs(span), vmax, amax)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = NAMES
        for k in range(int(math.ceil(duration / period)) + 1):
            t = min(k * period, duration)
            f = seg(t) / abs(span)
            g.trajectory.points.append(JointTrajectoryPoint(
                positions=[math.radians(a + (b - a) * f) for a, b in zip(q_from, q_to)],
                time_from_start=dur(t)))
        sf = self.fjt.send_goal_async(g)
        rclpy.spin_until_future_complete(self, sf)
        h = sf.result()
        if not h.accepted:
            return 'rejected'
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=duration + 15)
        r = rf.result().result
        return 'SUCCESSFUL' if r.error_code == 0 else '%d %s' % (r.error_code, r.error_string)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--speeds', default='5', help='MovJ v= values, %%')
    p.add_argument('--accel', type=float, default=20.0, help='MovJ a=, %%')
    p.add_argument('--delta', default='20,-8,12,0,0,0', help='the move, deg per joint')
    p.add_argument('--profiles', default='trap,scurve',
                   help='FJT speed profiles: trap (trapezoid), scurve (half-cosine ramps)')
    p.add_argument('--yes', action='store_true')
    args = p.parse_args()
    delta = [float(v) for v in args.delta.split(',')]
    rt = dp.RealtimeReader(os.environ.get('DOBOT_IP', dp.DEFAULT_IP))
    rt.start()
    rt.wait_first()
    q0 = list(rt.current(max_age=0.5)['q_actual'])
    q1 = [a + d for a, d in zip(q0, delta)]
    lead = max(range(6), key=lambda j: abs(delta[j]))
    print('Plan: from [%s] by [%s] deg and back, MovJ at v=%s %% (a=%g %%), then the same as FJT.'
          % (', '.join('%.1f' % v for v in q0), args.delta, args.speeds, args.accel))
    if not args.yes and input("Clear workspace, e-stop in hand? Type 'yes': ").strip() != 'yes':
        raise SystemExit('cancelled')
    rclpy.init()
    b = Bench()
    if not (b.movj.wait_for_service(timeout_sec=10) and b.fjt.wait_for_server(timeout_sec=10)):
        raise SystemExit('no MovJ service or FJT server')
    period = 0.00822
    rows = []
    for v in [float(x) for x in args.speeds.split(',')]:
        legs = []
        for name, qa, qb in (('out', q0, q1), ('back', q1, q0)):
            i0 = rt.index()
            r = b.call_movj(qb, v, args.accel)
            if r is None or r.res != 0:
                raise SystemExit('MovJ refused: %s' % (r.robot_return if r else 'no reply'))
            if not wait_motion(rt):
                raise SystemExit('the MovJ did not finish (mode %s)' % rt.current()['robot_mode'])
            time.sleep(0.3)
            pk = rt.slice(i0)
            dp.write_realtime_csv('movj_v%g_%s.csv' % (v, name), pk)
            m = measure(pk, lead, period)
            if m is None:
                raise SystemExit('MovJ %s: no motion seen' % name)
            legs.append((name, m))
            rows.append(dict(speed=v, how='MovJ', leg=name, **m))
            time.sleep(0.5)
        # The FJT moves at the profile the MovJ's leading joint had.
        vmax = sum(m['peak_deg_s'] for _, m in legs) / len(legs)
        amax = sum(m['lead_acc_deg_s2'] for _, m in legs) / len(legs)
        n_rows = 2
        for profile in args.profiles.split(','):
            for name, qa, qb in (('out', q0, q1), ('back', q1, q0)):
                time.sleep(1.5)                  # the previous segment's hold and servo tail
                i0 = rt.index()
                res = b.run_fjt(list(rt.current(max_age=0.5)['q_actual']), qb, vmax, amax, profile)
                time.sleep(0.3)
                pk = rt.slice(i0)
                dp.write_realtime_csv('fjt_%s_v%g_%s.csv' % (profile, v, name), pk)
                m = measure(pk, lead, period)
                if m is None:
                    raise SystemExit('FJT %s: no motion seen (%s)' % (name, res))
                rows.append(dict(speed=v, how='FJT-' + profile, leg=name, result=res, **m))
                n_rows += 1
                if res != 'SUCCESSFUL':
                    raise SystemExit('FJT %s %s: %s; stopping' % (profile, name, res))
        time.sleep(1.5)
        for r in rows[-n_rows:]:
            print('v=%-4g %-11s %-4s %5.2f s  lead %6.2f deg/s  ripple target %5.2f %%  arm %5.2f %%  '
                  'acc p99 %5.1f max %5.1f deg/s2  jerk p99 %5.0f max %5.0f deg/s3  tool %6.1f mm/s' % (
                      r['speed'], r['how'], r['leg'], r['duration_s'], r['peak_deg_s'],
                      r['ripple_target_pct'], r['ripple_arm_pct'], r['p99_acc_deg_s2'],
                      r['peak_acc_deg_s2'], r['p99_jerk_deg_s3'], r['peak_jerk_deg_s3'],
                      r['tool_peak_mm_s']))
    b.destroy_node()
    rclpy.shutdown()
    rt.stop()


if __name__ == '__main__':
    main()
