#!/usr/bin/env python3
"""T23: how far the tool travels after a halt signal on a straight ServoJ line. MOTION TEST.

drover's final approach is to run a straight line at a set speed until a
sensor signals and then stop within a few mm. This measures what the arm
does from the moment a halt is decided:

  - Streams ServoJ along a straight tool line from where the arm stands
    (--direction, in the base frame, user 0; default up), accelerating at
    --accel to --speed, cruising --cruise s, then halting.
  - The halt decelerates the commands along the same line at --decel, so the
    tool stays on it. --halt-to picks where the commands come to rest:
      commanded  decelerate from the commanded position (the commands run on
                 ahead of the arm by its lag, and the arm follows them)
      actual     end where the arm would stop decelerating at --decel from
                 its own position; the commands decelerate harder to make up
                 the lag, at most --max-decel
      stop       stop sending and send the controller's Stop() (--decels
                 does not apply; one run per speed)
  - Records the feed until the arm is idle, then streams back to the start
    at --return-speed. Otherwise no Stop() is sent (except on an alarm or a
    rejected ServoJ): Stop() on an arrived stream led to a lurch on the next
    one. Stop() while the arm moves never did.

For each run: the arm's lag behind the commands in the cruise (ms, mm), its
travel from the halt to standstill (mm) and how long that took, how far it
went past the last command, and its largest distance from the line. The
signal's own latency is not included: add speed x latency.

The line is built from the controller's InverseKin every --ik-step mm, so
the tool point is --tool's. The arm starts where it stands, enabled and
idle; drover-arm must be stopped.

    python3 t23_halt_distance.py --speeds 20,40 --decels 1000 --halt-to commanded
    python3 t23_halt_distance.py --direction 0,0,-1 --speeds 40 --decels 500,1000,2000
"""
import math
import re
import time

import dobot_proto as dp

STILL_MM = 0.05       # within this of its final position from then on: the arm has stopped
NEAR_START_DEG = 0.05  # the line's first point may differ this much from q_actual


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def numbers(text):
    return [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?(?:[eE]-?\d+)?', text)]


def get_pose(dash, user, tool):
    reply = dash.query(f'GetPose(user={user},tool={tool})')
    values = numbers(reply.value)
    if reply.err != 0 or len(values) != 6:
        raise SystemExit(f'GetPose failed: {reply.raw}')
    return values


def inverse_kin(dash, pose, near, user, tool):
    reply = dash.query('InverseKin(%s,user=%d,tool=%d,useJointNear=1,jointNear={%s})' % (
        ','.join('%.4f' % v for v in pose), user, tool, ','.join('%.4f' % v for v in near)))
    # The echo carries jointNear={...} too: take the first {...} only.
    q = numbers(reply.raw[reply.raw.find('{') + 1:reply.raw.find('}')])
    if reply.err != 0 or len(q) != 6:
        raise SystemExit(f'InverseKin failed at {pose}: {reply.raw}')
    return q


class Line:
    """The straight tool line as joint positions every `step` mm."""

    def __init__(self, dash, q0, pose0, u, length, step, user, tool, log):
        self.p0, self.u, self.step = pose0[:3], u, step
        n = int(math.ceil(length / step))
        self.s = [i * step for i in range(n + 1)]
        self.q = []
        near = q0
        t0 = dp.now()
        for s in self.s:
            pose = [pose0[i] + s * u[i] for i in range(3)] + list(pose0[3:])
            near = inverse_kin(dash, pose, near, user, tool)
            self.q.append(near)
        log('  line: %d points over %.1f mm in %.1f s' % (len(self.s), self.s[-1], dp.now() - t0))
        off = max(abs(a - b) for a, b in zip(self.q[0], q0))
        if off > NEAR_START_DEG:
            raise SystemExit('InverseKin at the current pose is %.3f deg from q_actual: '
                             'is --tool the tool GetPose reports?' % off)
        self.q[0] = list(q0)
        jump = max(max(abs(a - b) for a, b in zip(qa, qb)) for qa, qb in zip(self.q, self.q[1:]))
        self.deg_per_mm = jump / step
        if self.deg_per_mm > 2.0:
            raise SystemExit('the line turns a joint %.2f deg per mm: a singularity or a '
                             'branch change' % self.deg_per_mm)
        self.length = self.s[-1]

    def at(self, s):
        s = min(max(s, 0.0), self.length)
        i = min(int(s / self.step), len(self.s) - 2)
        f = (s - self.s[i]) / self.step
        return [a + (b - a) * f for a, b in zip(self.q[i], self.q[i + 1])]

    def where(self, pkt, key='tool_vector_actual'):
        """(distance along the line, distance from it), mm."""
        d = sub(pkt[key][:3], self.p0)
        along = dot(d, self.u)
        return along, norm([d[i] - along * self.u[i] for i in range(3)])


class HaltProfile:
    """Distance along the line (mm) against trajectory time: accelerate,
    cruise, halt. Called once per command, in order."""

    def __init__(self, line, rt, v, accel, cruise, decel, halt_to, max_decel):
        self.line, self.rt = line, rt
        self.v, self.accel, self.decel, self.halt_to, self.max_decel = v, accel, decel, halt_to, max_decel
        self.t_acc = v / accel
        self.s_acc = 0.5 * accel * self.t_acc ** 2
        self.t_halt = self.t_acc + cruise
        self.s_halt = self.s_acc + v * cruise
        self.halt_decel = None      # the deceleration the commands use, set at the halt
        self.gap_at_halt = None     # commanded - arm, mm, as the halt was decided
        self.s_by_t = {}
        self.duration = self.t_halt + v / min(decel, max_decel) + 0.04

    def planned_end(self):
        return self.s_halt + self.v ** 2 / (2 * self.decel)

    def s(self, t):
        if t <= self.t_acc:
            return 0.5 * self.accel * t * t
        if t <= self.t_halt:
            return self.s_acc + self.v * (t - self.t_acc)
        if self.halt_decel is None:
            self._decide()
        tau = min(t - self.t_halt, self.v / self.halt_decel)
        return self.s_halt + self.v * tau - 0.5 * self.halt_decel * tau * tau

    def _decide(self):
        latest_t, pkt = self.rt.latest
        along, _ = self.line.where(pkt)
        along += self.v * (dp.now() - latest_t)        # the packet's age
        self.gap_at_halt = self.s_halt - along
        if self.halt_to == 'commanded':
            self.halt_decel = self.decel
            return
        room = self.v ** 2 / (2 * self.decel) - self.gap_at_halt
        self.halt_decel = self.max_decel if room <= 0 else min(self.v ** 2 / (2 * room), self.max_decel)
        self.duration = self.t_halt + self.v / self.halt_decel + 0.04

    def __call__(self, t):
        s = self.s(t)
        self.s_by_t[round(t, 6)] = s
        return self.line.at(s)


def return_profile(line, s_from, v, accel):
    seg, duration = dp.trapezoid_1d(-s_from, v, accel)
    return (lambda t: line.at(s_from + seg(t))), duration


def wait_enable(rt, timeout=5.0):
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        mode = rt.current(max_age=0.5)['robot_mode']
        if mode == 5 or mode in dp.ERROR_MODES:
            return mode
        time.sleep(0.005)
    return rt.current(max_age=0.5)['robot_mode']


def measure(line, prof, result, packets):
    rows = result['rows']
    k = next((i for i, r in enumerate(rows) if r['traj_t'] > prof.t_halt + 1e-9), None)
    if prof.halt_to == 'stop':
        k = len(rows)
    if k is None or not packets:
        return {'error': 'the stream ended before the halt'}
    t_sig = result['stop_sent'] if prof.halt_to == 'stop' else rows[k]['t_send']
    ts = [t for t, _ in packets]
    along = [line.where(p)[0] for _, p in packets]
    off = [line.where(p)[1] for _, p in packets]
    s_cmd_t = [r['t_send'] for r in rows]
    s_cmd = [prof.s_by_t.get(round(r['traj_t'], 6)) for r in rows]
    s_final = along[-1]
    i_still = len(along) - 1
    while i_still > 0 and abs(along[i_still - 1] - s_final) <= STILL_MM:
        i_still -= 1
    arm_sig = dp.interp(ts, along, t_sig)
    cruise_gaps = []
    for t, a in zip(ts, along):
        if prof.t_acc + 0.2 <= t - rows[0]['t_send'] <= prof.t_halt:
            c = dp.interp(s_cmd_t, s_cmd, t)
            if c is not None:
                cruise_gaps.append(c - a)
    gap = sorted(cruise_gaps)[len(cruise_gaps) // 2] if cruise_gaps else float('nan')
    i_sig = next(i for i, t in enumerate(ts) if t >= t_sig)
    return {
        'cruise_gap_mm': round(gap, 3),
        'cruise_lag_ms': round(gap / prof.v * 1e3, 1),
        'gap_at_halt_mm': None if prof.gap_at_halt is None else round(prof.gap_at_halt, 3),
        'halt_decel_mm_s2': None if prof.halt_decel is None else round(prof.halt_decel, 1),
        'travel_mm': round(s_final - arm_sig, 3),
        'stop_ms': round((ts[i_still] - t_sig) * 1e3, 1),
        'commanded_travel_mm': round(s_cmd[-1] - prof.s_halt, 3),
        'past_last_command_mm': round(max(along[i_sig:]) - s_cmd[-1], 3),
        'max_off_line_mm': round(max(off), 3),
        'max_off_line_in_halt_mm': round(max(off[i_sig:]), 3),
        'final_mm': round(s_final, 3),
    }


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--speeds', default='20,40,60', help='cruise speeds, mm/s')
    p.add_argument('--decels', default='500,1000,2000', help='halt decelerations, mm/s^2')
    p.add_argument('--halt-to', default='commanded,actual',
                   help='any of commanded, actual, stop, comma-separated')
    p.add_argument('--accel', type=float, default=500.0, help='acceleration to --speed, mm/s^2')
    p.add_argument('--cruise', type=float, default=0.5, help='s at --speed before the halt')
    p.add_argument('--max-decel', type=float, default=5000.0,
                   help='--halt-to actual: the most the commands decelerate, mm/s^2')
    p.add_argument('--direction', default='0,0,1', help='line direction, base frame (user 0)')
    p.add_argument('--max-distance', type=float, default=80.0, help='longest line, mm')
    p.add_argument('--ik-step', type=float, default=0.5, help='mm between InverseKin points')
    p.add_argument('--return-speed', type=float, default=20.0, help='mm/s back to the start')
    p.add_argument('--reps', type=int, default=1)
    p.add_argument('--budget', type=float, default=3.0, help='travel to flag as within, mm')
    p.add_argument('--tool', type=int, default=1)
    args = p.parse_args()
    speeds = [float(v) for v in args.speeds.split(',')]
    decels = [float(v) for v in args.decels.split(',')]
    modes = args.halt_to.split(',')
    if set(modes) - {'commanded', 'actual', 'stop'}:
        raise SystemExit('--halt-to takes commanded, actual and stop')
    u = [float(v) for v in args.direction.split(',')]
    u = [v / norm(u) for v in u]
    if max(speeds) > 100 or args.max_distance > 150:
        raise SystemExit('speeds over 100 mm/s or a line over 150 mm')

    longest = max(HaltProfile(None, None, v, args.accel, args.cruise, d, 'commanded',
                              args.max_decel).planned_end() for v in speeds for d in decels)
    length = longest + 10.0       # room for --halt-to actual and the arm's overshoot
    if length > args.max_distance:
        raise SystemExit('the runs need %.0f mm of line; shorten --cruise or raise '
                         '--max-distance (%.0f)' % (length, args.max_distance))

    s = dp.Session(args.out, 't23_halt_distance', args)
    dash, rt = dp.connect(args, s.log)
    q0 = dp.preflight(dash, rt, s.log)
    for cmd in ('SpeedFactor(100)', 'User(0)', f'Tool({args.tool})'):
        s.log(f'{cmd}: {dash.query(cmd).raw}')
    time.sleep(0.1)
    wait_enable(rt)
    pose0 = get_pose(dash, 0, args.tool)
    feed = rt.current(max_age=0.2)['tool_vector_actual']
    s.log('pose: [%s]; feed tool position [%s]' % (', '.join('%.2f' % v for v in pose0),
                                                   ', '.join('%.2f' % v for v in feed[:3])))
    if norm(sub(feed[:3], pose0[:3])) > 0.5:
        raise SystemExit('the feed\'s tool position is %.1f mm from GetPose: the feed does '
                         'not report --tool\'s point' % norm(sub(feed[:3], pose0[:3])))
    line = Line(dash, q0, pose0, u, length, args.ik_step, 0, args.tool, s.log)
    top = max(speeds) * line.deg_per_mm
    if top > args.max_speed:
        raise SystemExit('%.0f mm/s turns a joint at up to %.1f deg/s (limit %.0f, '
                         '--max-speed)' % (max(speeds), top, args.max_speed))
    reach = max(abs(a - b) for a, b in zip(line.at(length), q0))
    if reach > args.max_excursion:
        raise SystemExit('the line moves a joint %.1f deg (limit %.0f, --max-excursion)'
                         % (reach, args.max_excursion))

    runs = [(v, d, m) for _ in range(args.reps) for v in speeds for d in decels for m in modes
            if m != 'stop'] + [(v, None, 'stop') for _ in range(args.reps) for v in speeds
                               if 'stop' in modes]
    dp.confirm(['', 'Plan: %d runs along a %.0f mm line from the tool\'s position, direction '
                '[%s] (base frame): speeds %s mm/s, halts at %s mm/s^2 to %s; back at %g '
                'mm/s after each. Joints move up to %.1f deg, at up to %.1f deg/s.'
                % (len(runs), length, ', '.join('%.2f' % v for v in u), args.speeds,
                   args.decels, args.halt_to, args.return_speed, reach, top)],
               args.yes, s.log)

    cfg = dp.stream_config(args)
    table = []
    for n, (v, decel, mode) in enumerate(runs, 1):
        name = 'r%02d_%gmm_s_%s_%s' % (n, v, 'stop' if decel is None else '%gmm_s2' % decel, mode)
        s.log('\n%s' % name)
        start = dp.idle_q(rt)
        if max(abs(a - b) for a, b in zip(start, q0)) > NEAR_START_DEG:
            s.log('  the arm is not back at the start; stopping')
            break
        prof = HaltProfile(line, rt, v, args.accel, args.cruise, decel or max(decels), mode,
                           args.max_decel)
        i0 = rt.index()
        run_cfg = dp.stream_config(args, stop_at=prof.t_halt, stop_cmd=True) if mode == 'stop' else cfg
        result = dp.run_stream(dash, rt, prof, prof.duration, run_cfg, log=s.log)
        mode_after = wait_enable(rt)
        time.sleep(0.3)
        packets = rt.slice(i0)
        m = measure(line, prof, result, packets) if not result['abort'] else {}
        summary = {'speed_mm_s': v, 'decel_mm_s2': decel, 'halt_to': mode,
                   'abort': result['abort'], 'error_ids': result['error_ids'],
                   'mode_after': dp.mode_name(mode_after), **m}
        s.save_run(name, result, packets, summary)
        if result['abort'] or mode_after != 5 or 'error' in m:
            s.log('  %s; stopping' % (result['abort'] or m.get('error') or
                                     'mode %s after the halt' % dp.mode_name(mode_after)))
            break
        s.log('  lag %.1f ms (%.2f mm) | halt: decel %s, travel %.2f mm in %.0f ms, %.2f mm '
              'past the last command | off the line %.2f mm' % (
                  m['cruise_lag_ms'], m['cruise_gap_mm'],
                  'Stop()' if mode == 'stop' else '%.0f' % m['halt_decel_mm_s2'], m['travel_mm'],
                  m['stop_ms'], m['past_last_command_mm'], m['max_off_line_mm']))
        table.append(summary)

        here = rt.current(max_age=0.2)
        back, duration = return_profile(line, m['final_mm'], args.return_speed, args.accel)
        if max(abs(a - b) for a, b in zip(back(0.0), here['q_actual'])) > 0.2:
            s.log('  the arm is off the line; not streaming back. Bring it back with a MovJ.')
            break
        r = dp.run_stream(dash, rt, back, duration, cfg, log=s.log)
        if r['abort'] or wait_enable(rt) != 5:
            s.log('  the return failed: %s' % (r['abort'] or 'not idle'))
            break
        time.sleep(0.3)

    s.log('\nspeed  decel  halt-to    lag ms  travel mm  stop ms  past cmd  off line')
    for t in table:
        s.log('%5g  %5s  %-9s  %6.1f  %8.2f%s  %7.0f  %8.2f  %8.2f' % (
            t['speed_mm_s'], '-' if t['decel_mm_s2'] is None else '%g' % t['decel_mm_s2'], t['halt_to'], t['cruise_lag_ms'], t['travel_mm'],
            ' ' if t['travel_mm'] <= args.budget else '*', t['stop_ms'],
            t['past_last_command_mm'], t['max_off_line_mm']))
    s.log('(* over --budget %g mm; add speed x the signal\'s latency to travel)' % args.budget)
    dp.write_json(s.dir + '/summary.json', table)
    rt.stop()


if __name__ == '__main__':
    main()
