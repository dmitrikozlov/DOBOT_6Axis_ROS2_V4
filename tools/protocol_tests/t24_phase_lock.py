#!/usr/bin/env python3
"""T24: does sending ServoJ in step with the controller's cycle smooth the motion? MOTION TEST.

The driver sends a ServoJ every 20.0 ms by the Pi's clock; the controller
takes the latest command once per its own cycle (8.22 ms, the realtime
feed's period). The same 300 mm line through the driver ran smooth once and
with a ~10 Hz speed ripple once. If commands arriving near the controller's
cycle boundary are what makes the ripple, sending each ServoJ right after a
feed packet (every --every-th packet, t = every x the feed period) removes it.

A straight line of --length mm along --direction (base frame) at --speed mm/s
is built from the controller's InverseKin every --ik-step mm, then streamed
out and back --reps times in each mode, alternating:

    timer   a ServoJ every --period s, by the clock (the driver's way)
    locked  a ServoJ right after every --every-th realtime packet

Each stream starts with a MovJ to the arm's own joints (Dobot: every ServoJ
segment starts with a MovJ). For each run: the tool's target speed ripple in
the cruise (rms about a 0.25 s moving average) and the number of spikes on
the joints' target speeds.

    python3 t24_phase_lock.py --length 400 --speed 80 --reps 3
"""
import math
import time

import dobot_proto as dp
from t23_halt_distance import Line, get_pose, norm, wait_enable

SPIKE_DEG_S = 0.25      # a joint's target speed this far off its 0.25 s average


def trapezoid_s(s_from, s_to, v, accel):
    seg, duration = dp.trapezoid_1d(s_to - s_from, v, accel)
    return (lambda t: s_from + seg(t)), duration


def stream(dash, rt, line, s_of_t, duration, mode, args):
    """Stream one move; returns (packets during it, ServoJ sent, error or None)."""
    q = list(rt.current(max_age=0.2)['q_actual'])
    r = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.4f' % v for v in q))
    if r.err != 0:
        return [], 0, 'MovJ rejected: %s' % r.raw
    i0 = rt.index()
    sent = 0
    if mode == 'timer':
        period = args.period
        start = dp.now()
        k = 0
        while True:
            traj_t = min(k * period, duration)
            target = line.at(s_of_t(traj_t))
            time.sleep(max(0.0, start + k * period - dp.now()))
            rep = dash.query(dp.servoj_cmd(target, period, args.aheadtime, args.gain))
            sent += 1
            if rep.err != 0:
                return rt.slice(i0), sent, 'ServoJ rejected: %s' % rep.raw
            if traj_t >= duration:
                break
            k += 1
    else:
        period = args.every * args.feed_period
        k = 0
        last = rt.count
        n = 0
        while True:
            while rt.count == last:          # the next packet: the controller's cycle
                time.sleep(0.0002)
            last = rt.count
            n += 1
            if n % args.every:
                continue
            traj_t = min(k * period, duration)
            target = line.at(s_of_t(traj_t))
            rep = dash.query(dp.servoj_cmd(target, period, args.aheadtime, args.gain))
            sent += 1
            if rep.err != 0:
                return rt.slice(i0), sent, 'ServoJ rejected: %s' % rep.raw
            if traj_t >= duration:
                break
            k += 1
    mode_after = wait_enable(rt, 5.0)
    packets = rt.slice(i0)
    if mode_after != 5:
        return packets, sent, 'not ENABLE after the stream (mode %s)' % mode_after
    return packets, sent, None


def ripple(packets, speed):
    t = [p[0] for p in packets]
    vt = [norm(p[1]['tcp_speed_target'][:3]) for p in packets]
    qd = [p[1]['qd_target'] for p in packets]
    cruise = [i for i, v in enumerate(vt) if abs(v - speed) < 0.15 * speed]
    if len(cruise) < 60:
        return float('nan'), 0, 0.0
    a, b = cruise[0] + 15, cruise[-1] - 15
    n = 31

    def ma(seq, i):
        lo, hi = max(0, i - n // 2), min(len(seq), i + n // 2 + 1)
        return sum(seq[lo:hi]) / (hi - lo)
    dev = [vt[i] - ma(vt, i) for i in range(a, b)]
    rms = math.sqrt(sum(d * d for d in dev) / len(dev))
    spikes, inside = 0, False
    for i in range(a, b):
        off = max(abs(qd[i][j] - ma([q[j] for q in qd], i)) for j in range(6))
        if off > SPIKE_DEG_S and not inside:
            spikes += 1
        inside = off > SPIKE_DEG_S
    return 100.0 * rms / speed, spikes, (t[b] - t[a])


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--length', type=float, default=400.0, help='mm')
    p.add_argument('--speed', type=float, default=80.0, help='mm/s')
    p.add_argument('--accel', type=float, default=500.0, help='mm/s^2')
    p.add_argument('--direction', default='0,-1,0', help='base frame')
    p.add_argument('--ik-step', type=float, default=0.5, help='mm between InverseKin points')
    p.add_argument('--period', type=float, default=0.02, help='timer mode: s between ServoJ')
    p.add_argument('--every', type=int, default=3, help='locked mode: a ServoJ every N packets')
    p.add_argument('--feed-period', default='0.00822',
                   help='the controller cycle, s, or auto: measured over 5 s of the feed')
    p.add_argument('--reps', type=int, default=3)
    p.add_argument('--modes', default='timer,locked')
    p.add_argument('--tool', type=int, default=1)
    args = p.parse_args()
    if args.speed > 100 or args.length > 500:
        raise SystemExit('speed over 100 mm/s or a line over 500 mm')
    u = [float(v) for v in args.direction.split(',')]
    u = [v / norm(u) for v in u]
    s = dp.Session(args.out, 't24_phase_lock', args)
    dash, rt = dp.connect(args, s.log)
    if args.feed_period == 'auto':
        n0, t0 = rt.count, dp.now()
        time.sleep(5.0)
        args.feed_period = (dp.now() - t0) / (rt.count - n0)
        s.log('feed period measured: %.5f ms' % (args.feed_period * 1e3))
    else:
        args.feed_period = float(args.feed_period)
    q0 = dp.preflight(dash, rt, s.log)
    for cmd in ('SpeedFactor(100)', 'User(0)', f'Tool({args.tool})'):
        s.log(f'{cmd}: {dash.query(cmd).raw}')
    time.sleep(0.1)
    wait_enable(rt)
    pose0 = get_pose(dash, 0, args.tool)
    line = Line(dash, q0, pose0, u, args.length, args.ik_step, 0, args.tool, s.log)
    top = args.speed * line.deg_per_mm
    if top > args.max_speed:
        raise SystemExit('%.0f mm/s turns a joint at up to %.1f deg/s (limit %.0f)'
                         % (args.speed, top, args.max_speed))
    reach = max(abs(a - b) for a, b in zip(line.at(line.length), q0))
    if reach > max(args.max_excursion, 60.0):
        raise SystemExit('the line moves a joint %.1f deg' % reach)
    modes = args.modes.split(',')
    dp.confirm(['', 'Plan: a %.0f mm line along [%s] at %.0f mm/s, out and back, %d times in '
                'each of %s; joints move up to %.1f deg at up to %.1f deg/s.'
                % (line.length, ', '.join('%.2f' % v for v in u), args.speed, args.reps,
                   ' and '.join(modes), reach, top)], args.yes, s.log)

    table = []
    for rep in range(args.reps):
        for mode in modes:
            for name, s_from, s_to in (('out', 0.0, line.length), ('back', line.length, 0.0)):
                s_of_t, duration = trapezoid_s(s_from, s_to, args.speed, args.accel)
                packets, sent, err = stream(dash, rt, line, s_of_t, duration, mode, args)
                pct, spikes, cruise = ripple(packets, args.speed)
                s.log('%-6s %-4s rep %d: %d ServoJ, cruise %.1f s, tool speed ripple %.2f %%, '
                      '%d spikes%s' % (mode, name, rep + 1, sent, cruise, pct, spikes,
                                       '; ' + err if err else ''))
                table.append({'mode': mode, 'leg': name, 'rep': rep + 1, 'ripple_pct': pct,
                              'spikes': spikes, 'cruise_s': cruise, 'error': err})
                dp.write_realtime_csv('%s/%s_%s_%d.csv' % (s.dir, mode, name, rep + 1), packets)
                if err:
                    s.log('stopping: %s' % err)
                    dp.write_json(s.dir + '/summary.json', table)
                    return
                time.sleep(0.5)
    s.log('\nmode    runs  ripple % (each)                spikes (each)')
    for mode in modes:
        rows = [r for r in table if r['mode'] == mode]
        s.log('%-7s %4d  %-30s %s' % (mode, len(rows), ' '.join('%.2f' % r['ripple_pct'] for r in rows),
                                       ' '.join(str(r['spikes']) for r in rows)))
    dp.write_json(s.dir + '/summary.json', table)
    rt.stop()


if __name__ == '__main__':
    main()
