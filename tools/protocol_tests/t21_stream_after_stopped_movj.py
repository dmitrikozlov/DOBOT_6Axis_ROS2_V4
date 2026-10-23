#!/usr/bin/env python3
"""T21: a ServoJ stream after Stop() ended a MovJ. MOTION TEST.

drover's hold-to-run Home is a MovJ (v=6, a=20) that Stop() ends when the
button is let go before the arm arrives; Start then streams ServoJ. After
Stop() on a ServoJ session, a later MovJ made the next stream drive J1 the
wrong way (t19, t20). This checks the MovJ version. Per case:

    stop          MovJ J1 --distance deg, Stop() halfway; stream back to the start
    stop-movj     ... Stop() halfway; MovJ to J1 --away deg past the start
                  (Home pressed again, held to the end); stream back to the start
    stop-movl     ... Stop() halfway; MovL --lift mm up and back; MovJ past the
                  start; stream back to the start (t19's order)
    early         MovJ queued and Stop() at once, before it moves; stream
                  J1 +--stream deg

and records the controller's J1 target in the first 0.5 s of the stream: its
largest move against the commanded direction, and any rejection or alarm. A
faulting case is saved, cleared (clear_alarm.py) and the run goes on.

    python3 t21_stream_after_stopped_movj.py --cases stop,stop-movj,stop-movl,early --reps 2

drover-arm must be stopped. Keep J1 clear --distance deg above the start and
--away + 5 deg below it.
"""
import os
import time

import dobot_proto as dp
from clear_alarm import wait_mode
from t13_movl_speed import get_pose
from t20_stream_after_stop import against, movl, stream

CASES = ('stop', 'stop-movj', 'stop-movl', 'early')


def movj(dash, q, args):
    return dash.query('MovJ(joint={%s},a=%g,v=%g)' % (
        ','.join('%.3f' % v for v in q), args.movj_a, args.movj_v))


def wait_moving(rt, timeout=3.0):
    """Until the MovJ is RUNNING; the time, or None."""
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        if rt.current(max_age=0.5)['robot_mode'] == 7:
            return dp.now()
        time.sleep(0.005)
    return None


def clear(dash, rt, s):
    """ClearError, Stop() out of PAUSE, re-enable."""
    s.log('    clearing: %s' % dash.query('ClearError()').raw)
    mode = wait_mode(rt, (4, 5, 10), 3.0)
    if mode == 10:
        dash.query('Stop()')
        mode = wait_mode(rt, (5,), 3.0)
    if mode == 4:
        dash.query('EnableRobot()')
        mode = wait_mode(rt, (5,), 10.0)
    s.log('    now %s, %s' % (dp.mode_name(mode), dash.query('GetErrorID()').raw))
    return mode == 5


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--cases', default=','.join(CASES))
    p.add_argument('--reps', type=int, default=2)
    p.add_argument('--distance', type=float, default=10.0, help='J1 deg of the stopped MovJ')
    p.add_argument('--stop-at', type=float, default=0.5,
                   help='s after the MovJ starts moving to send Stop()')
    p.add_argument('--away', type=float, default=5.0, help='J1 deg past the start (stop-movj, stop-movl)')
    p.add_argument('--stream', type=float, default=5.0, help='J1 deg streamed in the early case')
    p.add_argument('--lift', type=float, default=10.0, help='mm of the MovL (stop-movl)')
    p.add_argument('--movj-v', type=float, default=6.0, help='MovJ v= (drover: movj_speed_percent)')
    p.add_argument('--movj-a', type=float, default=20.0, help='MovJ a= (drover: movj_accel_percent)')
    p.add_argument('--user', type=int, default=0)
    p.add_argument('--tool', type=int, default=1)
    args = p.parse_args()
    unknown = set(args.cases.split(',')) - set(CASES)
    if unknown:
        raise SystemExit(f'unknown cases {sorted(unknown)}; known: {", ".join(CASES)}')
    if abs(args.distance) > 15 or abs(args.away) > 10:
        raise SystemExit('distance over 15 deg or away over 10 deg')
    s = dp.Session(args.out, 't21_stream_after_stopped_movj', args)
    dash, rt = dp.connect(args, s.log)
    q_start = dp.preflight(dash, rt, s.log)
    s.log('SpeedFactor(100): %s' % dash.query('SpeedFactor(100)').raw)
    dp.confirm(['', f'Plan: {args.reps} x each of {args.cases}, from and back to J1 '
                f'{q_start[0]:.2f} deg'], args.yes, s.log)

    results = []
    for rep in range(1, args.reps + 1):
        for case in args.cases.split(','):
            q0 = dp.idle_q(rt)
            i0 = rt.index()
            target = list(q0)
            target[0] += args.distance
            mj = movj(dash, target, args)
            if case == 'early':
                stop = dash.query('Stop()')
            else:
                t_move = wait_moving(rt)
                time.sleep(max(0.0, (t_move or dp.now()) + args.stop_at - dp.now()))
                stop = dash.query('Stop()')
            time.sleep(0.05)
            dp.wait_idle(dash, rt, timeout=5.0, log=s.log)
            stopped = dp.idle_q(rt)
            note = f'MovJ {mj.raw[:12]}, Stop() at J1 {stopped[0] - q0[0]:+.2f} deg'
            if case == 'stop-movl':
                pose = get_pose(dash, args.user, args.tool)
                up = list(pose)
                up[2] += args.lift
                for tgt in (up, pose):
                    dash.query(movl(tgt, args))
                    time.sleep(0.2)
                    dp.wait_idle(dash, rt, timeout=10.0, log=s.log)
                note += ', MovL up and down'
            if case in ('stop-movj', 'stop-movl'):
                away = list(stopped)
                away[0] = q_start[0] - args.away
                movj(dash, away, args)
                time.sleep(0.2)
                dp.wait_idle(dash, rt, timeout=20.0, log=s.log)
                note += f', MovJ to J1 {away[0] - q0[0]:+.2f} deg'
            time.sleep(0.5)
            q1 = dp.idle_q(rt)
            distance = args.stream if case == 'early' else q_start[0] - q1[0]
            t0, rejected = stream(dash, q1, distance, args)
            time.sleep(1.0)
            mode = dp.wait_idle(dash, rt, timeout=5.0, log=s.log)
            packets = rt.slice(i0)
            worst = against(packets, t0, q1, 1.0 if distance >= 0 else -1.0)
            err = dash.query('GetErrorID()').raw
            faulted = bool(rejected) or mode != 5 or worst > 0.05
            s.log(f'{case:>9} rep {rep}: {note}; stream J1 {distance:+.2f} deg: target '
                  f'{worst:.3f} deg the wrong way, {"REJECTED " + rejected[:12] if rejected else "accepted"}, '
                  f'{dp.mode_name(mode)}, {err}{"  FAULT" if faulted else ""}')
            results.append({'case': case, 'rep': rep, 'note': note, 'stream_deg': distance,
                            'wrong_way_deg': worst, 'rejected': rejected, 'mode': mode,
                            'errors': err, 'fault': faulted})
            dp.write_realtime_csv(os.path.join(s.run_dir(f'{case}_{rep}'), 'realtime.csv'), packets)
            if faulted and not clear(dash, rt, s):
                s.log('stopping: could not clear')
                break
            # Back to the start for the next case (a plain MovJ, no Stop()).
            if abs(dp.idle_q(rt)[0] - q_start[0]) > 0.05:
                movj(dash, q_start, args)
                time.sleep(0.2)
                dp.wait_idle(dash, rt, timeout=20.0, log=s.log)
            time.sleep(0.5)
    dp.write_json(os.path.join(s.dir, 'results.json'), results)
    s.log('%d fault(s) in %d case(s)' % (sum(r['fault'] for r in results), len(results)))
    rt.stop()
    dash.close()


if __name__ == '__main__':
    main()
