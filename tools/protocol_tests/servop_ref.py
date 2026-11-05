#!/usr/bin/env python3
"""Dobot's ServoP reference loop (ServoP Windows抖动优化, 2026-11), on a line out and back. MOTION TEST.

Sends ServoP through Dobot's own dobot_api.py (put it next to this file, from
Dobot's demo) the way their reference program does: one command every
--period s, timed with time.perf_counter() (sleep half the remainder, then
busy-wait), t = --t, the process bound to one CPU core and, on Windows, the
timer resolution raised to 1 ms. Runs on Windows and Linux.

Each rep: MovJ to --start-joints, then the line by --delta (mm, base axes, at
the start's orientation) out and back. --mode servop streams it; movl and
movj send one command per leg, for comparison. Each stream starts straight
after a MovJ to the arm's own joints (not in Dobot's program: our CR5 needs it).

Without --accel the stream is Dobot's: a constant step, speed x period, from
the first command (their test: --delta 100,0,100 at 31.4 mm/s, 100 commands).
With --accel the speed ramps up and down (half-cosine) at that acceleration.

Writes <out>/<leg>.csv per leg: each command's send and reply times
(perf_counter, s) and the reply. The motion itself is measured separately, on
the controller's ports, by whatever records them (rec_ports.py on the Pi).

    pip install numpy requests psutil
    python servop_ref.py --start-joints -90,40,-87,-25,91,0 --reps 3
    python servop_ref.py --start-joints ... --delta 300,0,0 --speed 150 --accel 600
    python servop_ref.py --start-joints ... --delta 300,0,0 --mode movl --speed 150 --a 30
"""
import argparse
import csv
import math
import os
import re
import sys
import time

from dobot_api import DobotApiDashboard

if sys.platform == 'win32':
    import ctypes
    ctypes.windll.winmm.timeBeginPeriod(1)


def floats(text):
    return [float(v) for v in text.split(',')]


def profile(distance, speed, accel, period):
    """Fractions of the line, one per command."""
    if not accel:
        n = max(1, round(distance / (speed * period)))
        return [(k + 1) / n for k in range(n)]
    ramp = math.pi * speed / (2 * accel)
    if speed * ramp > distance:
        speed = math.sqrt(2 * accel * distance / math.pi)
        ramp = math.pi * speed / (2 * accel)
    d_ramp = speed * ramp / 2
    total = 2 * ramp + (distance - 2 * d_ramp) / speed

    def up(t):
        return speed / 2 * (t - ramp / math.pi * math.sin(math.pi * t / ramp))

    out = []
    k = 1
    while True:
        t = min(k * period, total)
        s = up(t) if t < ramp else (distance - up(total - t) if t > total - ramp
                                    else d_ramp + speed * (t - ramp))
        out.append(s / distance)
        if t >= total:
            return out
        k += 1


def reply_values(raw):
    m = re.match(r'\s*(-?\d+),\{([^}]*)\}', raw or '')
    if not m:
        raise SystemExit('unexpected reply: %r' % raw)
    return int(m.group(1)), [float(v) for v in m.group(2).split(',') if v.strip()]


def wait_idle(dash, timeout=30.0):
    """Until RobotMode is 5 (enabled, idle); the move must have started (7) first or 1 s passed."""
    t0 = time.perf_counter()
    started = False
    while time.perf_counter() - t0 < timeout:
        err, v = reply_values(dash.RobotMode())
        mode = int(v[0]) if v else -1
        started = started or mode == 7
        if mode == 5 and (started or time.perf_counter() - t0 > 1.0):
            return
        if mode == 9:
            raise SystemExit('the arm is in error (mode 9)')
        time.sleep(0.02)
    raise SystemExit('the arm did not come to rest in %.0f s' % timeout)


def stream(dash, p0, p1, fractions, args):
    """Dobot's loop: ServoP, then perf_counter until the period is up."""
    rows = []
    for k, f in enumerate(fractions):
        a = time.perf_counter()
        target = [u + (v - u) * f for u, v in zip(p0, p1)]
        raw = dash.ServoP(*target, t=args.t)
        b = time.perf_counter()
        rows.append({'k': k, 'fraction': '%.6f' % f, 't_send': '%.6f' % a,
                     't_reply': '%.6f' % b, 'reply': raw.strip()})
        if reply_values(raw)[0] != 0:
            raise SystemExit('ServoP rejected: %s' % raw)
        remaining = args.period - (b - a)
        if remaining > 0.01:
            time.sleep(remaining * 0.5)
        while time.perf_counter() - a < args.period:
            pass
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ip', default='192.168.5.1')
    p.add_argument('--start-joints', required=True, type=floats, help='deg, j1..j6')
    p.add_argument('--delta', default='100,0,100', type=floats, help='mm, x,y,z')
    p.add_argument('--mode', choices=('servop', 'movl', 'movj'), default='servop')
    p.add_argument('--speed', type=float, default=1000 * math.sqrt(2) / 45,
                   help='mm/s (default: Dobot\'s, 1.41 mm per 45 ms)')
    p.add_argument('--accel', type=float, default=0.0, help='mm/s^2; 0: no ramps (Dobot\'s)')
    p.add_argument('--period', type=float, default=0.045, help='s between ServoP')
    p.add_argument('--t', type=float, default=0.05, help='ServoP t, s')
    p.add_argument('--a', type=int, default=20, help='movl/movj acceleration, %%')
    p.add_argument('--v', type=int, default=20, help='movj speed, %%')
    p.add_argument('--hold', type=float, default=0.3,
                   help='s: send the start point this long before moving (not in Dobot\'s program: '
                        'the primer MovJ can take ~170 ms after a stream, and the controller skips '
                        'the ServoP queued behind it)')
    p.add_argument('--cpu', type=int, default=2, help='CPU core to run on (-1: any)')
    p.add_argument('--reps', type=int, default=3)
    p.add_argument('--out', default='servop_ref_' + time.strftime('%Y%m%d-%H%M%S'))
    p.add_argument('--yes', action='store_true')
    args = p.parse_args()
    if len(args.start_joints) != 6 or len(args.delta) != 3:
        raise SystemExit('--start-joints takes 6 values, --delta 3')
    distance = math.sqrt(sum(d * d for d in args.delta))
    if args.speed > 250:
        raise SystemExit('over 250 mm/s, the safety level\'s tool speed')

    if args.cpu >= 0:
        try:
            import psutil
            psutil.Process().cpu_affinity([args.cpu])
        except Exception as e:     # psutil missing, or fewer cores
            print('not bound to a CPU core: %s' % e)
    os.makedirs(args.out, exist_ok=True)
    dash = DobotApiDashboard(args.ip, 29999)

    fractions = profile(distance, args.speed, args.accel, args.period)
    hold = [0.0] * round(args.hold / args.period)
    if args.mode == 'servop':
        plan = ('ServoP every %.1f ms, t=%.3f, %d commands per leg (%.1f s), peak %.1f mm/s'
                % (args.period * 1e3, args.t, len(fractions), len(fractions) * args.period,
                   max(b - a for a, b in zip([0] + fractions, fractions)) * distance / args.period))
    else:
        plan = '%s, %s' % (args.mode, 'speed=%g mm/s a=%d%%' % (args.speed, args.a)
                           if args.mode == 'movl' else 'v=%d%% a=%d%%' % (args.v, args.a))
    print('Plan: MovJ to [%s], then [%s] mm out and back, %d times. %s.'
          % (','.join('%g' % v for v in args.start_joints),
             ','.join('%g' % v for v in args.delta), args.reps, plan))
    print('Results: %s' % args.out)
    if not args.yes and input('Type yes to move the arm: ').strip() != 'yes':
        return

    for rep in range(args.reps):
        dash.MovJ(*args.start_joints, 1, a=20, v=10)
        wait_idle(dash)
        time.sleep(1.0)
        err, p0 = reply_values(dash.GetPose())
        p1 = [v + d for v, d in zip(p0, args.delta + [0, 0, 0])]
        for leg, a, b in (('out', p0, p1), ('back', p1, p0)):
            name = '%s_%d' % (leg, rep + 1)
            if args.mode == 'servop':
                # a MovJ to the arm's own joints straight before the stream: without it, a
                # stream after an idle wait can plan every joint toward 0 (alarm 225)
                err, q = reply_values(dash.GetAngle())
                dash.MovJ(*q, 1, a=20, v=5)
                rows = stream(dash, a, b, hold + fractions, args)
                iv = [float(y['t_send']) - float(x['t_send']) for x, y in zip(rows, rows[1:])]
                print('%s: %d ServoP, interval %.3f ms mean, %.3f..%.3f ms' % (
                    name, len(rows), 1e3 * sum(iv) / len(iv), 1e3 * min(iv), 1e3 * max(iv)))
            else:
                t0 = time.perf_counter()
                raw = (dash.MovL(*b, 0, a=args.a, speed=int(args.speed)) if args.mode == 'movl'
                       else dash.MovJ(*b, 0, a=args.a, v=args.v))
                rows = [{'k': 0, 'fraction': 1, 't_send': '%.6f' % t0,
                         't_reply': '%.6f' % time.perf_counter(), 'reply': raw.strip()}]
                if reply_values(raw)[0] != 0:
                    raise SystemExit('%s rejected: %s' % (args.mode, raw))
                print('%s: %s' % (name, raw.strip()))
            wait_idle(dash)
            with open(os.path.join(args.out, name + '.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            time.sleep(1.5)


if __name__ == '__main__':
    main()
