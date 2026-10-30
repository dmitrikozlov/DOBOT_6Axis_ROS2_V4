#!/usr/bin/env python3
"""T25: what comes before a spike in the controller's target speed? MOTION TEST.

Streams a joint move the way the driver does (a ServoJ right after every
--every-th realtime packet, t = every x the feed period) and records, for every
command: the packet it was due on and the one it followed, that packet's
arrival, the send and reply times and the reply; and every realtime packet with
the controller's own clock (controller_timer). The move is synchronised in
joint space (all joints finishing together) with half-cosine speed ramps.

With --shape-hz F the move goes through a ZV input shaper for a mode of F Hz
(damping --shape-zeta): two copies of the move, the second half a period
later, weighted so the mode's swing from the first is cancelled by the second.
The move takes half a period longer.

With --timer PERIOD the ServoJ go out by the Pi's clock every PERIOD s
instead, with t = PERIOD: in step with a controller clock of an exact PERIOD
divisor, if it runs on one.

Each stream starts with a MovJ to the arm's own joints. Files per leg in the
results directory: commands.csv (one row per ServoJ) and realtime.csv.

    python3 t25_spike_trace.py --delta 80,-8,12,0,0,0 --speed 38 --accel 45 --reps 2
"""
import csv
import math
import os
import time

import dobot_proto as dp


def scurve_1d(distance, vmax, amax):
    d = abs(distance)
    sign = 1.0 if distance >= 0 else -1.0
    ramp = math.pi * vmax / (2 * amax)
    if vmax * ramp > d:
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


def zv_shaper(f, duration, hz, zeta):
    """f (a move of the given duration) through a ZV shaper for a mode at hz."""
    if hz <= 0:
        return f, duration
    k = math.exp(-zeta * math.pi / math.sqrt(1 - zeta * zeta))
    delay = 0.5 / hz
    a1, a2 = 1 / (1 + k), k / (1 + k)
    return (lambda t: a1 * f(t) + a2 * f(t - delay)), duration + delay


def wait_enable(rt, timeout=5.0):
    t0 = dp.now()
    while dp.now() - t0 < timeout:
        if rt.current(max_age=0.5)['robot_mode'] == 5:
            return True
        time.sleep(0.005)
    return False


def stream(dash, rt, q_from, q_to, args, period):
    lead = max(range(6), key=lambda j: abs(q_to[j] - q_from[j]))
    span = q_to[lead] - q_from[lead]
    seg, duration = scurve_1d(abs(span), args.speed, args.accel)
    seg, duration = zv_shaper(seg, duration, args.shape_hz, args.shape_zeta)
    r = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.4f' % v for v in q_from))
    if r.err != 0:
        raise SystemExit('MovJ rejected: %s' % r.raw)
    i0 = rt.index()
    rows = []
    sent_on = 0
    k = 0
    start = dp.now()
    while True:
        if args.timer:
            time.sleep(max(0.0, start + k * period - dp.now()))
            want = got = rt.count
        else:
            want = rt.count + 1 if sent_on == 0 else sent_on + args.every
            while rt.count < want:
                time.sleep(0.0002)
            got = rt.count
        with rt.lock:
            arrived = rt.packets[got - 1][0]
        sent_on = got
        traj_t = min(k * period, duration)
        f = seg(traj_t) / abs(span)
        target = [a + (b - a) * f for a, b in zip(q_from, q_to)]
        t_wake = dp.now()
        rep = dash.query(dp.servoj_cmd(target, period, args.aheadtime, args.gain))
        rows.append({'k': k, 'traj_t': traj_t, 'want': want, 'got': got, 'packet_arrived': arrived,
                     't_wake': t_wake, 't_send': rep.t_send, 't_reply': rep.t_recv,
                     'reply': rep.raw, 'target_lead': target[lead]})
        if rep.err != 0:
            raise SystemExit('ServoJ rejected: %s' % rep.raw)
        if traj_t >= duration:
            break
        k += 1
    wait_enable(rt)
    time.sleep(0.3)
    return rows, rt.slice(i0)


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(aheadtime=100.0, gain=500.0)
    p.add_argument('--delta', default='80,-8,12,0,0,0', help='deg per joint')
    p.add_argument('--speed', type=float, default=38.0, help="the leading joint's peak, deg/s")
    p.add_argument('--accel', type=float, default=45.0, help="its peak acceleration, deg/s^2")
    p.add_argument('--every', type=int, default=3)
    p.add_argument('--timer', type=float, default=0.0,
                   help='send by the clock every this many s (t the same) instead of on packets')
    p.add_argument('--shape-hz', type=float, default=0.0, help='ZV input shaper for a mode at this Hz')
    p.add_argument('--shape-zeta', type=float, default=0.04, help="the mode's damping ratio")
    p.add_argument('--reps', type=int, default=2)
    args = p.parse_args()
    delta = [float(v) for v in args.delta.split(',')]
    if args.speed > 60:
        raise SystemExit('over 60 deg/s')
    s = dp.Session(args.out, 't25_spike_trace', args)
    dash, rt = dp.connect(args, s.log)
    q0 = dp.preflight(dash, rt, s.log)
    q1 = [a + d for a, d in zip(q0, delta)]
    n0, t0 = rt.count, dp.now()
    time.sleep(3.0)
    feed = (dp.now() - t0) / (rt.count - n0)
    if args.timer:
        period = args.timer
        s.log('feed period %.4f ms; a ServoJ every %.2f ms by the clock' % (feed * 1e3, period * 1e3))
    else:
        period = args.every * feed
        s.log('feed period %.4f ms; a ServoJ every %d packets (%.2f ms)'
              % (feed * 1e3, args.every, period * 1e3))
    dp.confirm(['', 'Plan: by [%s] deg and back, %d times, the leading joint at %g deg/s.'
                % (args.delta, args.reps, args.speed)], args.yes, s.log)
    for rep in range(args.reps):
        for name, qa, qb in (('out', q0, q1), ('back', q1, q0)):
            rows, packets = stream(dash, rt, list(rt.current(max_age=0.5)['q_actual']), qb, args, period)
            d = s.run_dir('%s_%d' % (name, rep + 1))
            with open(os.path.join(d, 'commands.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            dp.write_realtime_csv(os.path.join(d, 'realtime.csv'), packets)
            slow = max(r['t_reply'] - r['t_send'] for r in rows) * 1e3
            late = sum(1 for r in rows if r['got'] > r['want'])
            s.log('%s %d: %d ServoJ, slowest reply %.1f ms, %d followed a later packet than due'
                  % (name, rep + 1, len(rows), slow, late))
            time.sleep(1.0)
    rt.stop()


if __name__ == '__main__':
    main()
