#!/usr/bin/env python3
"""T18: how a ServoJ stream should end. MOTION TEST.

Streams one trapezoidal move on a joint (default J1, 10 deg at 20 deg/s,
60 deg/s^2), which comes to rest at its last point, then ends it in one of
these ways:

    none   stop sending at the last point of the move
    once   send the last point once more (drover's driver today)
    hold   keep sending the last point every period for --hold s
    stop   send Stop() right after the last point

and records how the arm settles, timed from the move's last point: when the
controller's target reached the final position, when the arm came within --tol of it, its largest overshoot,
when the controller went back to ENABLE (5), and whether a MovJ sent then is
accepted at once. Each ending is run --reps times, alternating directions so
the arm ends where it started.

    python3 t18_stream_end.py --endings none,once,hold,stop --reps 2

For a stream stopped mid-move (the link dropping, or a Stop), see t9.
drover-arm must be stopped. The arm starts where it stands; the joint turns
--distance degrees and back.
"""
import os
import time

import dobot_proto as dp


def stream(dash, q0, joint, distance, args):
    """Stream the move; returns (time of the last command, final joints,
    when the next command would be due)."""
    seg, duration = dp.trapezoid_1d(distance, 20.0, 60.0)
    period = 1.0 / args.rate
    final = list(q0)
    final[joint] += distance
    t0 = dp.now()
    k = 0
    while True:
        traj_t = min(k * period, duration)
        q = list(q0)
        q[joint] += seg(traj_t)
        time.sleep(max(0.0, t0 + k * period - dp.now()))
        r = dash.query(dp.servoj_cmd(q, args.servo_t, args.aheadtime, args.gain))
        if r.err != 0:
            raise RuntimeError(f'ServoJ rejected: {r.raw}')
        if traj_t >= duration:
            return r.t_send, final, t0 + (k + 1) * period
        k += 1


def end(dash, how, final, next_due, args):
    period = 1.0 / args.rate
    if how == 'once':
        time.sleep(max(0.0, next_due - dp.now()))
        return dash.query(dp.servoj_cmd(final, args.servo_t, args.aheadtime, args.gain)).t_send
    if how == 'hold':
        t_end = next_due + args.hold
        t = next_due
        last = None
        while t < t_end:
            time.sleep(max(0.0, t - dp.now()))
            last = dash.query(dp.servoj_cmd(final, args.servo_t, args.aheadtime, args.gain)).t_send
            t += period
        return last
    if how == 'stop':
        return dash.query('Stop()').t_send
    return None


def settle(packets, joint, final, t_last, tol):
    """Times after the move's last point (s): target within 0.1 deg and within
    tol of the final position, arm staying within tol, ENABLE; and the largest
    overshoot past the final position (deg)."""
    rel = [(t - t_last, p) for t, p in packets if t >= t_last]
    target = lambda d: next((dt for dt, p in rel if abs(p['q_target'][joint] - final[joint]) < d),
                            None)
    in_tol = None
    for i, (dt, p) in enumerate(rel):
        if all(abs(q[1]['q_actual'][joint] - final[joint]) < tol for q in rel[i:]):
            in_tol = dt
            break
    sign = 1.0 if final[joint] >= packets[0][1]['q_actual'][joint] else -1.0
    overshoot = max(sign * (p['q_actual'][joint] - final[joint]) for _, p in rel) if rel else 0.0
    enable_at = next((dt for dt, p in rel if p['robot_mode'] == 5), None)
    return {'target_coarse_s': target(0.1), 'target_final_s': target(tol), 'arm_in_tol_s': in_tol,
            'overshoot_deg': overshoot, 'enable_s': enable_at}


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--joint', type=int, default=1, choices=range(1, 7))
    p.add_argument('--distance', type=float, default=10.0)
    p.add_argument('--endings', default='none,once,hold,stop')
    p.add_argument('--hold', type=float, default=0.3, help='s of repeated final point (hold)')
    p.add_argument('--reps', type=int, default=2)
    p.add_argument('--tol', type=float, default=0.01, help='deg counted as arrived')
    args = p.parse_args()
    if abs(args.distance) > args.max_excursion:
        raise SystemExit('distance over --max-excursion')
    j = args.joint - 1
    s = dp.Session(args.out, 't18_stream_end', args)
    dash, rt = dp.connect(args, s.log)
    dp.preflight(dash, rt, s.log)
    s.log('SpeedFactor(100): %s' % dash.query('SpeedFactor(100)').raw)
    dp.confirm(['', f'Plan: J{args.joint} {args.distance:+g} deg moves, alternating direction, '
                f'ended by {args.endings}, {args.reps} each'], args.yes, s.log)

    results = []
    direction = 1.0
    runs = [(rep, how) for rep in range(1, args.reps + 1) for how in args.endings.split(',')]
    for rep, how in runs:
        i0 = rt.index()
        try:
            q0 = dp.idle_q(rt)
            t_last, final, next_due = stream(dash, q0, j, direction * args.distance, args)
            t_end = end(dash, how, final, next_due, args) or t_last
        except RuntimeError as e:
            time.sleep(1.0)
            dp.write_realtime_csv(os.path.join(s.run_dir(f'failed_{how}_{rep}'), 'realtime.csv'),
                                  rt.slice(i0))
            s.log(f'stopping: {e}; {dash.query("GetErrorID()").raw}; feed saved')
            break
        time.sleep(1.0)
        # A MovJ to where the arm is: accepted at once, or refused while
        # the controller is still in its stream?
        here = dp.idle_q(rt) if rt.current()['robot_mode'] == 5 else rt.current()['q_actual']
        t_movj = dp.now()
        movj = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.3f' % v for v in here))
        dp.wait_idle(dash, rt, timeout=5.0, log=s.log)
        time.sleep(0.3)
        packets = rt.slice(i0)
        r = settle(packets, j, final, t_last, args.tol)
        fmt = lambda v: '-' if v is None else '%.0f' % (v * 1e3)
        s.log(f'{how:5s} rep {rep}: ms after the move\'s last point: ending sent to '
              f'{fmt(t_end - t_last)}, target within 0.1 deg {fmt(r["target_coarse_s"])}, '
              f'within {args.tol} {fmt(r["target_final_s"])}, arm within {args.tol} '
              f'{fmt(r["arm_in_tol_s"])}, ENABLE {fmt(r["enable_s"])}; overshoot '
              f'{r["overshoot_deg"]:+.3f} deg; MovJ at {fmt(t_movj - t_last)}: {movj.raw[:12]}')
        results.append(dict(r, ending=how, rep=rep, movj_reply=movj.raw))
        dp.write_realtime_csv(os.path.join(s.run_dir(f'{how}_{rep}'), 'realtime.csv'), packets)
        direction = -direction
        time.sleep(0.5)
    dp.write_json(os.path.join(s.dir, 'results.json'), results)
    rt.stop()
    dash.close()


if __name__ == '__main__':
    main()
