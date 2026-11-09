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

With --t-scale S each ServoJ carries t = S x its send interval while the
trajectory still advances by the interval: S > 1 sends each target before the
controller has finished the previous one (Dobot: send at t or earlier, by 3 %
or more).

With --timer PERIOD the ServoJ go out by the Pi's clock every PERIOD s
instead, with t = PERIOD: in step with a controller clock of an exact PERIOD
divisor, if it runs on one.

With --cycles N the ServoJ go out right after every N-th block of the
controller's log port 65511, with t = N x 4 ms: the controller's motion module
logs one [ch8] block per 4 ms cycle and takes ServoJ on that grid, dividing
each step by min(t, the gap in whole cycles), so commands must arrive exactly N
cycles apart for the target speed to stay even. With --track-phase as well,
field 84 of each block (the queue number being executed) shows the cycle each
command was taken up in; a command taken up a cycle later than usual (counted
from the block it followed) means the commands sit near the late edge, and they
move 1 ms earlier; one taken up early, 1 ms later.

With --line X,Y,Z the move is a straight line of the tool (mm, base axes) by
ServoP, at --speed mm/s and --accel mm/s^2, for comparison with Dobot's ServoP
reference loop (servop_ref.py).

Each stream starts with a MovJ to the arm's own joints. Files per leg in the
results directory: commands.csv (one row per ServoJ) and realtime.csv.

    python3 t25_spike_trace.py --delta 80,-8,12,0,0,0 --speed 38 --accel 45 --reps 2
"""
import csv
import math
import os
import re
import socket
import threading
import time

import dobot_proto as dp

ALGS_LOG_PORT = 65511    # the controller's motion-module log: one [ch8] block per cycle
ALGS_CYCLE = 0.004       # s
EXECUTING = re.compile(rb'\n84:(\d+)')   # field 84 of a block: the queue number being executed
PHASE_STEP_MS = 1.0      # a step away from the edge a command was taken up beyond
PHASE_BASELINE = 3       # commands since the last step before their usual take-up delay counts
CLOCK_DRIFT = 1e-4       # the fitted cycle clock may run this much late per s (100 ppm)


class CycleClock(threading.Thread):
    """Counts the [ch8] blocks on port 65511, keeping each one's arrival time
    (host monotonic), so commands can be sent on the controller's own cycle.
    Field 84 of a block is the queue number the controller is executing:
    taken[n] is the first block (1-based, as count) in which n appears.
    due(k) is when block k is sent, from the earliest arrivals: block k at
    base + k x 4 ms, base the smallest arrival - k x 4 ms seen, which may rise by
    CLOCK_DRIFT so that the two clocks drifting apart is not read as lateness."""

    MARK = b'[ch8]'

    def __init__(self, ip):
        super().__init__(daemon=True)
        self.sock = socket.create_connection((ip, ALGS_LOG_PORT), timeout=3.0)
        self.sock.settimeout(1.0)
        self.count = 0
        self.times = []
        self.taken = {}
        self.base = None
        self.lock = threading.Lock()

    def due(self, k):
        with self.lock:
            return self.base + k * ALGS_CYCLE

    def run(self):
        tail = b''
        text, parsed = None, 0      # unparsed text from the first mark on; blocks parsed
        while True:
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            t = dp.now()
            data = tail + chunk
            n = data.count(self.MARK)
            tail = data[-(len(self.MARK) - 1):]     # a mark split across chunks
            with self.lock:
                self.times.extend([t] * n)
                if n and self.base is not None:
                    self.base = min(t - (self.count + n) * ALGS_CYCLE,
                                    self.base + CLOCK_DRIFT * (t - self.times[-n - 1]))
                elif n:
                    self.base = t - (self.count + n) * ALGS_CYCLE
                self.count += n
            # field 84 of every complete block (one is complete when the next starts)
            text = chunk if text is None else text + chunk
            if parsed == 0 and self.MARK not in text:
                text = text[-(len(self.MARK) - 1):]
                continue
            start = text.find(self.MARK)
            ends = []
            i = text.find(self.MARK, start + 1)
            while i >= 0:
                ends.append(i)
                i = text.find(self.MARK, i + 1)
            for end in ends:
                parsed += 1
                m = EXECUTING.search(text, start, end)
                if m:
                    with self.lock:
                        self.taken.setdefault(int(m.group(1)), parsed)
                start = end
            text = text[start:]

    def stop(self):
        try:
            self.sock.close()
        except OSError:
            pass


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


def servop_cmd(target, servo_t, aheadtime, gain):
    return ('ServoP(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,t=%.4f,aheadtime=%g,gain=%g)'
            % (tuple(target) + (servo_t, aheadtime, gain)))


def line_knots(dash, p_from, p_to, q_near, step_mm=5.0):
    """Joints along a tool line from the controller's IK, every step_mm (each
    solution nearest the last), for ServoJ."""
    n = max(2, math.ceil(math.dist(p_from[:3], p_to[:3]) / step_mm))
    knots, q = [], q_near
    for i in range(n + 1):
        p = [a + (b - a) * i / n for a, b in zip(p_from, p_to)]
        r = dash.query('InverseKin(%s,useJointNear=1,jointNear={%s})'
                       % (','.join('%.4f' % v for v in p), ','.join('%.4f' % v for v in q)))
        m = re.match(r'\s*0,\{([^}]*)\}', r.raw)
        if not m:
            raise SystemExit('no IK %d mm along the line: %s' % (i * step_mm, r.raw))
        q = [float(v) for v in m.group(1).split(',')]
        knots.append(q)
    return knots


def stream(dash, rt, q_from, q_to, args, period, cycles=None, phase=None):
    """q_from, q_to: joints (deg), or with --line tool poses (mm, deg)."""
    knots = None
    if args.line:
        lead, span = 0, math.dist(q_from[:3], q_to[:3])
        q_here = list(rt.current(max_age=0.5)['q_actual'])
        if args.line_servoj:
            knots = line_knots(dash, q_from, q_to, q_here)
    else:
        lead = max(range(6), key=lambda j: abs(q_to[j] - q_from[j]))
        span = q_to[lead] - q_from[lead]
        q_here = q_from
    seg, duration = scurve_1d(abs(span), args.speed, args.accel)
    seg, duration = zv_shaper(seg, duration, args.shape_hz, args.shape_zeta)
    command = servop_cmd if args.line and knots is None else dp.servoj_cmd
    r = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.4f' % v for v in q_here))
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
        elif cycles is not None:
            want = cycles.count + 1 if sent_on == 0 else sent_on + args.cycles
            if not args.cycle_model:
                while cycles.count < want:
                    time.sleep(0.0002)
                got = cycles.count
            else:
                got = want
            delay = args.cycle_delay
            if phase is not None:
                # take-up delay: cycles from the block a command followed to the block
                # in which the controller took it; it varies when we sit at a cycle edge
                with cycles.lock:
                    for c in phase['sent'][-20:]:
                        if c['lag'] is None and c['queue'] in cycles.taken:
                            c['lag'] = cycles.taken[c['queue']] - c['block']
                lags = [c['lag'] for c in phase['sent'] if c['regime'] == phase['regime']
                        and c['lag'] is not None]
                if len(lags) >= PHASE_BASELINE:
                    phase['usual'] = max(set(lags), key=lags.count)
                odd = [g for g in lags if phase['usual'] is not None and g != phase['usual']]
                if odd:
                    # taken a cycle late: we are near the late edge, so earlier; early: later
                    step = -PHASE_STEP_MS if odd[0] > phase['usual'] else PHASE_STEP_MS
                    unwrapped = phase['delay'] + step
                    phase['delay'] = unwrapped % (ALGS_CYCLE * 1000.0)
                    # past 0 or a whole cycle the send moves to the neighbouring block's
                    # cycle, and every take-up delay with it
                    phase['usual'] += round((phase['delay'] - unwrapped) / (ALGS_CYCLE * 1000.0))
                    phase['regime'] += 1
                    phase['shifts'].append((k, '%d not %d' % (odd[0], phase['usual']),
                                            phase['delay']))
                delay = phase['delay']
            if args.cycle_model:
                # on the fitted clock, not on the block's arrival: blocks reach us up
                # to ~4.5 ms late (99th percentile), which moved commands across a cycle
                due = cycles.due(got) + delay / 1000.0
                time.sleep(max(0.0, due - dp.now() - 0.0005))
                while dp.now() < due:
                    pass
            elif delay:
                # the commands' place in the cycle: away from its edge, jitter cannot move them across
                with cycles.lock:
                    due = cycles.times[got - 1] + delay / 1000.0
                time.sleep(max(0.0, due - dp.now()))
        else:
            want = rt.count + 1 if sent_on == 0 else sent_on + args.every
            while rt.count < want:
                time.sleep(0.0002)
            got = rt.count
        if cycles is not None and args.cycle_model:
            arrived = cycles.due(got)
        elif cycles is not None:
            with cycles.lock:
                arrived = cycles.times[got - 1]
        else:
            with rt.lock:
                arrived = rt.packets[got - 1][0]
        sent_on = got
        traj_t = min(max(0.0, k * period - args.hold), duration)
        f = seg(traj_t) / abs(span)
        if knots is None:
            target = [a + (b - a) * f for a, b in zip(q_from, q_to)]
        else:
            x = f * (len(knots) - 1)
            i = min(int(x), len(knots) - 2)
            target = [a + (b - a) * (x - i) for a, b in zip(knots[i], knots[i + 1])]
        t_wake = dp.now()
        rep = dash.query(command(target, period * args.t_scale, args.aheadtime, args.gain))
        rows.append({'k': k, 'traj_t': traj_t, 'want': want, 'got': got, 'packet_arrived': arrived,
                     't_wake': t_wake, 't_send': rep.t_send, 't_reply': rep.t_recv,
                     'reply': rep.raw, 'target_lead': target[lead],
                     'delay_ms': phase['delay'] if phase is not None else args.cycle_delay})
        if rep.err != 0:
            raise SystemExit('ServoJ rejected: %s' % rep.raw)
        if phase is not None:
            queue = re.match(r'\s*0,\{(\d+)\}', rep.raw)
            if queue:
                phase['sent'].append({'queue': int(queue.group(1)), 'block': got,
                                      'regime': phase['regime'], 'lag': None})
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
    p.add_argument('--line', default=None,
                   help='mm, x,y,z: a straight line of the tool by ServoP instead of --delta '
                        '(--speed mm/s, --accel mm/s^2)')
    p.add_argument('--line-servoj', action='store_true',
                   help='with --line: ServoJ to joints from the controller\'s IK along the line')
    p.add_argument('--speed', type=float, default=38.0, help="the leading joint's peak, deg/s")
    p.add_argument('--accel', type=float, default=45.0, help="its peak acceleration, deg/s^2")
    p.add_argument('--every', type=int, default=3)
    p.add_argument('--timer', type=float, default=0.0,
                   help='send by the clock every this many s (t the same) instead of on packets')
    p.add_argument('--cycles', type=int, default=0,
                   help="send after every this many 4 ms cycles of the controller's motion "
                        "module (port 65511), t = cycles x 4 ms, instead of on packets")
    p.add_argument('--cycle-delay', type=float, default=0.0,
                   help='with --cycles: ms to wait after the block before sending')
    p.add_argument('--cycle-model', action='store_true',
                   help="with --cycles: send on a clock fitted to the blocks' earliest "
                        "arrivals instead of on each block's arrival")
    p.add_argument('--track-phase', action='store_true',
                   help="with --cycles: move the commands half a cycle whenever the cycles "
                        "until the controller takes them up (port 65511 field 84) vary")
    p.add_argument('--shape-hz', type=float, default=0.0, help='ZV input shaper for a mode at this Hz')
    p.add_argument('--shape-zeta', type=float, default=0.04, help="the mode's damping ratio")
    p.add_argument('--t-scale', type=float, default=1.0,
                   help="ServoJ t as a multiple of the send interval")
    p.add_argument('--hold', type=float, default=0.0,
                   help='s: send the start point this long before moving; the primer MovJ can take '
                        '~170 ms after a stream, and the controller skips the ServoJ queued behind it')
    p.add_argument('--reps', type=int, default=2)
    args = p.parse_args()
    if args.line:
        delta = [float(v) for v in args.line.split(',')] + [0.0, 0.0, 0.0]
        if args.speed > 250:
            raise SystemExit("over 250 mm/s, the safety level's tool speed")
    else:
        delta = [float(v) for v in args.delta.split(',')]
        if args.speed > 60:
            raise SystemExit('over 60 deg/s')
    s = dp.Session(args.out, 't25_spike_trace', args)
    dash, rt = dp.connect(args, s.log)
    q0 = dp.preflight(dash, rt, s.log)
    if args.line:
        q0 = list(rt.current(max_age=0.5)['tool_vector_actual'])
    q1 = [a + d for a, d in zip(q0, delta)]
    n0, t0 = rt.count, dp.now()
    time.sleep(3.0)
    feed = (dp.now() - t0) / (rt.count - n0)
    cycles = phase = None
    if args.track_phase and not args.cycles:
        raise SystemExit('--track-phase needs --cycles')
    if args.timer:
        period = args.timer
        s.log('feed period %.4f ms; a ServoJ every %.2f ms by the clock' % (feed * 1e3, period * 1e3))
    elif args.cycles:
        cycles = CycleClock(args.ip)
        cycles.start()
        c0, t0 = cycles.count, dp.now()
        time.sleep(2.0)
        if cycles.count - c0 < 100:
            raise SystemExit('port %d gave %d [ch8] blocks in 2 s: no cycle to pace on'
                             % (ALGS_LOG_PORT, cycles.count - c0))
        period = args.cycles * ALGS_CYCLE
        s.log('feed period %.4f ms; motion-module cycle %.4f ms (port %d); a ServoJ every %d '
              'cycles, %.1f ms after the block, t=%.2f ms'
              % (feed * 1e3, (dp.now() - t0) / (cycles.count - c0) * 1e3, ALGS_LOG_PORT,
                 args.cycles, args.cycle_delay, period * args.t_scale * 1e3))
        if args.track_phase:
            phase = {'delay': args.cycle_delay, 'regime': 0, 'sent': [], 'shifts': [],
                     'usual': None}
            s.log('tracking the phase on port %d field 84: %.0f ms earlier when a command is '
                  'taken up a cycle late, later when early' % (ALGS_LOG_PORT, PHASE_STEP_MS))
    else:
        period = args.every * feed
        s.log('feed period %.4f ms; a ServoJ every %d packets (%.2f ms), t=%.2f ms'
              % (feed * 1e3, args.every, period * 1e3, period * args.t_scale * 1e3))
    dp.confirm(['', 'Plan: by [%s] mm and back by ServoP, %d times, at %g mm/s.'
                % (args.line, args.reps, args.speed) if args.line else
                'Plan: by [%s] deg and back, %d times, the leading joint at %g deg/s.'
                % (args.delta, args.reps, args.speed)], args.yes, s.log)
    for rep in range(args.reps):
        for name, qa, qb in (('out', q0, q1), ('back', q1, q0)):
            shifts_before = len(phase['shifts']) if phase is not None else 0
            # a line runs between its two planned ends; a joint move from where the arm stands
            start = qa if args.line else list(rt.current(max_age=0.5)['q_actual'])
            rows, packets = stream(dash, rt, start, qb, args,
                                   period, cycles, phase)
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
            if phase is not None:
                shifts = phase['shifts'][shifts_before:]
                lags = [c['lag'] for c in phase['sent'][-len(rows):] if c['lag'] is not None]
                s.log('    phase shifts: %d%s; take-up delays (cycles: count) %s' % (
                    len(shifts), ''.join(' (command %d: delay %s -> %.0f ms)' % x for x in shifts),
                    {v: lags.count(v) for v in sorted(set(lags))}))
            time.sleep(1.0)
    if cycles is not None:
        cycles.stop()
    rt.stop()


if __name__ == '__main__':
    main()
