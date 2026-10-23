#!/usr/bin/env python3
"""T20: a ServoJ stream after Stop() ended an earlier one mid-move, and a MovJ
moved the arm. MOTION TEST.

On 2026-10-22 (t19) a stream after Stop(), a MovL and a MovJ drove J1 the
wrong way at ~52 deg/s and raised alarms 225 and 227. This is the cancel path
drover can take (Stop during a move, Home by MovJ, then Start). Per case:

    1. stream J1 +--distance deg and send Stop(): halfway, as a cancel does,
       or once the arm has arrived (J1 within 0.01 deg for 50 ms)
    2. optionally a MovL --lift mm up in z and back, then a MovJ to J1 --away
       deg past the start, against the first move
    3. stream J1 back to the start, against the MovJ as in t19, with or without --hold s of ServoJ at
       the arm's own position first (drover's streams begin so)

    case      Stop()    between       hold
    hold      halfway   MovJ          yes
    nohold    halfway   MovJ          no
    arrival   arrived   MovJ          no
    movl      halfway   MovL, MovJ    no
    arrival-movl  arrived  MovL, MovJ  no    (t19's sequence)

and records the controller's J1 target in the first 0.5 s of the stream: its
largest move against the commanded direction, and any rejection or alarm.

    python3 t20_stream_after_stop.py --cases hold,nohold
    python3 t20_stream_after_stop.py --cases arrival
    python3 t20_stream_after_stop.py --cases arrival-movl
    python3 t20_stream_after_stop.py --cases arrival-movl --repeat

A faulting stream is saved and ends the run: clear the alarm before the next.
drover-arm must be stopped. Keep J1 clear --away + 5 deg either side.
"""
import math
import os
import time

import dobot_proto as dp
from t13_movl_speed import get_pose

CASES = {   # Stop() once arrived, MovL before the MovJ, hold at the stream's start
    'hold': (False, False, True),
    'nohold': (False, False, False),
    'arrival': (True, False, False),
    'movl': (False, True, False),
    'arrival-movl': (True, True, False),
}


def stream(dash, q0, distance, args, hold=0.0, stop_after=None):
    """J1 trapezoid from q0 after `hold` s at q0; Stop() after `stop_after`
    of the move's commands. Returns (first send time, rejection or None)."""
    seg, duration = dp.trapezoid_1d(distance, 20.0, 60.0)
    period = 1.0 / args.rate
    n_hold = int(round(hold / period))
    n_move = int(math.ceil(duration / period)) + 1   # the last one at the final point
    t0 = dp.now()
    for k in range(n_hold + n_move):
        q = list(q0)
        if k >= n_hold:
            q[0] += seg(min((k - n_hold) * period, duration))
        time.sleep(max(0.0, t0 + k * period - dp.now()))
        r = dash.query(dp.servoj_cmd(q, args.servo_t, args.aheadtime, args.gain))
        if r.err != 0:
            return t0, r.raw
        if stop_after is not None and k - n_hold + 1 >= stop_after:
            dash.query('Stop()')
            return t0, None
    return t0, None


def wait_arrival(rt, final, tol=0.01, timeout=1.5):
    """Until J1 has stayed within tol of final for 50 ms; the time, or None."""
    deadline = dp.now() + timeout
    since = None
    while dp.now() < deadline:
        t = dp.now()
        if abs(rt.current(max_age=0.5)['q_actual'][0] - final) <= tol:
            since = since or t
            if t - since >= 0.05:
                return t
        else:
            since = None
        time.sleep(0.004)
    return None


def movl(pose, args):
    return 'MovL(pose={%s},user=%d,tool=%d,a=20,speed=20)' % (
        ','.join(f'{v:.3f}' for v in pose), args.user, args.tool)


def against(packets, t0, q0, direction, window=0.5):
    """Largest move of the controller's J1 target against `direction` (deg)."""
    worst = 0.0
    for t, pk in packets:
        if t0 <= t <= t0 + window:
            worst = max(worst, -direction * (pk['q_target'][0] - q0[0]))
    return worst


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--cases', default='hold,nohold')
    p.add_argument('--distance', type=float, default=6.0, help='J1 deg of the cancelled move')
    p.add_argument('--away', type=float, default=5.0, help='J1 deg past the start the MovJ goes to')
    p.add_argument('--hold', type=float, default=0.3, help='s at the start (hold cases)')
    p.add_argument('--lift', type=float, default=10.0, help='mm of the MovL (movl case)')
    p.add_argument('--pause-after-stop', type=float, default=0.0,
                   help='s to wait after Stop() has stopped the arm')
    p.add_argument('--pause-after-movl', type=float, default=0.0,
                   help='s to wait at the top of the MovL (t19 waits ~4 s there)')
    p.add_argument('--repeat', action='store_true',
                   help='MovJ back to where the stopped move began and stream the same move '
                        'again, as t19 does')
    p.add_argument('--user', type=int, default=0)
    p.add_argument('--tool', type=int, default=1)
    args = p.parse_args()
    unknown = set(args.cases.split(',')) - set(CASES)
    if unknown:
        raise SystemExit(f'unknown cases {sorted(unknown)}; known: {sorted(CASES)}')
    if abs(args.distance) > 10 or abs(args.away) > 10:
        raise SystemExit('distance or away over 10 deg')
    s = dp.Session(args.out, 't20_stream_after_stop', args)
    dash, rt = dp.connect(args, s.log)
    q_start = dp.preflight(dash, rt, s.log)
    s.log('SpeedFactor(100): %s' % dash.query('SpeedFactor(100)').raw)
    dp.confirm(['', f'Plan: for each of {args.cases}: J1 {args.distance:+g} deg ended by Stop(), '
                f'(MovL), MovJ to J1 {args.away:g} deg past the start, stream back to it'],
               args.yes, s.log)

    period = 1.0 / args.rate
    _, duration = dp.trapezoid_1d(args.distance, 20.0, 60.0)
    results = []
    for case in args.cases.split(','):
        at_arrival, with_movl, with_hold = CASES[case]
        q0 = dp.idle_q(rt)
        i0 = rt.index()
        # 1. the move ended by Stop()
        if at_arrival:
            stream(dash, q0, args.distance, args)
            arrived = wait_arrival(rt, q0[0] + args.distance)
            dash.query('Stop()')
            s.log(f'{case:>7}: Stop() {"after arrival" if arrived else "(arm had not arrived)"}')
        else:
            stream(dash, q0, args.distance, args, stop_after=int(duration / period / 2))
        time.sleep(0.05)
        dp.wait_idle(dash, rt, timeout=5.0, log=s.log)
        stopped = dp.idle_q(rt)
        time.sleep(args.pause_after_stop)
        if with_movl:
            pose = get_pose(dash, args.user, args.tool)
            up = list(pose)
            up[2] += args.lift
            for target in (up, pose):
                s.log(f'{case:>7}: MovL {dash.query(movl(target, args)).raw[:14]}')
                time.sleep(0.2)
                dp.wait_idle(dash, rt, timeout=10.0, log=s.log)
                if target is up:
                    time.sleep(args.pause_after_movl)
        # 2. Home-like MovJ away from where it stopped
        away = list(stopped)
        away[0] = q0[0] if args.repeat else q_start[0] - args.away
        mj = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.3f' % v for v in away))
        time.sleep(0.2)
        dp.wait_idle(dash, rt, timeout=15.0, log=s.log)
        time.sleep(0.5)
        # 3. the next stream, back to the start
        q1 = dp.idle_q(rt)
        back = args.distance if args.repeat else q_start[0] - q1[0]
        hold = args.hold if with_hold else 0.0
        t0, rejected = stream(dash, q1, back, args, hold=hold)
        time.sleep(1.0)
        mode = dp.wait_idle(dash, rt, timeout=5.0, log=s.log)
        packets = rt.slice(i0)
        worst = against(packets, t0, q1, 1.0 if back >= 0 else -1.0, window=0.5 + hold)
        err = dash.query('GetErrorID()').raw
        s.log(f'{case:>7}: stopped at J1 {stopped[0] - q0[0]:+.2f} deg, MovJ {mj.raw[:14]}, '
              f'then J1 {back:+.2f} deg streamed{" after %.1f s hold" % hold if hold else ""}: '
              f'target moved {worst:.3f} deg the wrong way; '
              f'{"REJECTED " + rejected[:12] if rejected else "accepted"}; '
              f'mode {dp.mode_name(mode)}; {err}')
        results.append({'case': case, 'stopped_deg': stopped[0] - q0[0], 'stream_deg': back,
                        'hold_s': hold, 'wrong_way_deg': worst, 'rejected': rejected,
                        'mode': mode, 'errors': err})
        dp.write_realtime_csv(os.path.join(s.run_dir(case), 'realtime.csv'), packets)
        if rejected or mode != 5:
            s.log('stopping: clear the alarm before running more')
            break
        time.sleep(0.5)
    dp.write_json(os.path.join(s.dir, 'results.json'), results)
    rt.stop()
    dash.close()


if __name__ == '__main__':
    main()
