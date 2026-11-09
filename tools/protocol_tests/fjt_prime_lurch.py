#!/usr/bin/env python3
"""Does a new ServoJ segment lurch at its start? MOTION TEST (run), or analysis (analyse).

run: J1 moves of +-10 deg (cosine, 1.5 s) through the driver's
FollowJointTrajectory server, each after --pause s, so that the previous goal's
hold has ended and every goal starts a new segment: the driver's MovJ to the
arm's own joints, then its ServoJ straight behind it. Appends each goal's send
time (monotonic) to --marks. Needs a running driver, the arm enabled and J1
clear to turn 10 deg each way; rec_ports.py and imu_record.py record alongside.

analyse: per goal, from a session recorded by rec_ports.py (+ imu.csv):
how long the controller ran the MovJ (port 65511 field 84: blocks of 4 ms),
how many queue numbers after it never ran (skipped ServoJ), the largest J1
target speed in the first 0.3 s of motion against the cosine's speed there,
and the flange's 4-8 Hz acceleration over the move.

    ROS_DOMAIN_ID=77 python3 fjt_prime_lurch.py run --marks S/marks.txt --reps 3
    python3 fjt_prime_lurch.py analyse S
"""
import argparse
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DELTA, DURATION = 10.0, 1.5
PEAK = DELTA * math.pi / (2 * DURATION)          # the cosine's peak speed, deg/s


def run(args):
    import rclpy
    import dobot_proto as dp
    from fjt_bench import Bench, goal_for
    rt = dp.RealtimeReader(dp.DEFAULT_IP)
    rt.start()
    rt.wait_first()
    rclpy.init()
    b = Bench(rt)
    if not b.client.wait_for_server(timeout_sec=20):
        raise SystemExit('no FJT server')
    for k in range(2 * args.reps):
        time.sleep(args.pause)
        delta = DELTA if k % 2 == 0 else -DELTA
        with open(args.marks, 'a') as f:
            f.write('%.3f goal%d\n' % (time.monotonic(), k + 1))
        ok = b.run('goal %d: J1 %+g deg after %.1f s' % (k + 1, delta, args.pause),
                   goal_for(b.q(), 0, delta, duration=DURATION), 'SUCCESSFUL')
        if not ok:
            raise SystemExit('stopping')
    time.sleep(args.pause)
    os._exit(0)


def analyse(args):
    import numpy as np
    from compare_lines import band_rms, load_blocks
    ports = os.path.join(args.session, 'ports')
    rows = list(csv.DictReader(open(os.path.join(ports, 'realtime.csv'))))
    t = np.array([float(r['host_t']) for r in rows])
    q1 = np.array([float(r['q_target_1']) for r in rows])
    v1 = np.array([float(r['qd_target_1']) for r in rows])
    bt, bq = load_blocks(ports)
    imu = np.loadtxt(os.path.join(args.session, 'imu.csv'), delimiter=',', skiprows=1)
    it, acc = imu[:, 0], imu[:, 1:4]
    marks = [(float(a), n) for a, n in (l.split() for l in open(os.path.join(args.session, 'marks.txt')))]
    print('%-7s %9s %8s %10s %10s %9s %9s' % ('goal', 'MovJ ms', 'skipped', 'J1 max', 'cosine',
                                             'ratio', 'flange'))
    print('%-7s %9s %8s %10s %10s %9s %9s' % ('', '', 'ServoJ', '0.3 s deg/s', 'at 0.3 s', '', 'm/s2'))
    for i, (t0, name) in enumerate(marks):
        t1 = marks[i + 1][0] if i + 1 < len(marks) else t0 + 5.0
        w = (bt >= t0) & (bt < t1)
        q = bq[w]
        movj_ms = skipped = float('nan')
        if len(q):
            # the first queue number the window runs after t0 is the MovJ
            runs = []
            for a in q:
                if runs and runs[-1][0] == a:
                    runs[-1][1] += 1
                else:
                    runs.append([a, 1])
            first_new = next((k for k, (a, n) in enumerate(runs) if a != q[0]), None)
            if first_new is not None:
                movj, n = runs[first_new]
                movj_ms = n * 4.0
                after = runs[first_new + 1][0] if first_new + 1 < len(runs) else movj + 1
                skipped = after - movj - 1
        m = (t >= t0) & (t < t1)
        idx = np.flatnonzero(m)
        moving = idx[np.abs(q1[idx] - q1[idx[0]]) > 0.01]
        if not len(moving):
            print('%-7s no motion' % name)
            continue
        s = moving[0]
        early = (t >= t[s]) & (t <= t[s] + 0.3)
        vmax = np.abs(v1[early]).max()
        cos03 = PEAK * math.sin(math.pi * 0.3 / DURATION)
        leg = (it >= t[s]) & (it <= t[s] + DURATION + 0.5)
        print('%-7s %9.0f %8.0f %10.2f %10.2f %9.2f %9.3f' % (
            name, movj_ms, skipped, vmax, cos03, vmax / cos03, band_rms(it[leg], acc[leg], 4, 8)))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--marks', required=True)
    r.add_argument('--reps', type=int, default=3)
    r.add_argument('--pause', type=float, default=3.0, help='s before each goal (hold is 1 s)')
    a = sub.add_parser('analyse')
    a.add_argument('session')
    args = p.parse_args()
    (run if args.cmd == 'run' else analyse)(args)


if __name__ == '__main__':
    main()
