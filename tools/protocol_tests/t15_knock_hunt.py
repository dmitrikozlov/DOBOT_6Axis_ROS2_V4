#!/usr/bin/env python3
"""T15: hunt for knocks — the controller's target speed dropping to zero for
one packet in the middle of a ServoJ stream. MOTION TEST.

Repeats the J1 out-and-back sweep of the ripple tests (25 deg at 20 deg/s,
60 deg/s^2) and checks every run for a knock: |qd_target| of the moving joint
falling below a fifth of its value in one packet while above 10 deg/s, in
RUNNING. On 2026-10-16 knocks came about once in 10 moves at 33.3 Hz with t
shorter than the period (the defaults here), about once in 50 with t equal to
it. Each knock is printed with the send gaps, reply times and current command
id around it, and every run is saved, so a knock can be sent to Dobot.

    python3 t15_knock_hunt.py --runs 40 --stop-after 3
    python3 t15_knock_hunt.py --rate 50 --servo-t 0.02 --runs 60    # drover's setting

drover-arm must be stopped (port 29999 takes one client). The arm starts
where it stands: J1 turns +25 deg and back.
"""
import time

import dobot_proto as dp


def knocks(packets, joint):
    """Indices of packets where the target speed collapsed."""
    out = []
    for i in range(1, len(packets)):
        prev, cur = packets[i - 1][1], packets[i][1]
        a, b = abs(prev['qd_target'][joint]), abs(cur['qd_target'][joint])
        if cur['robot_mode'] == 7 and a > 10.0 and b < 0.2 * a:
            out.append(i)
    return out


def describe(sess, packets, rows, i, joint):
    t_k = packets[i][0]
    sess.log('    KNOCK at %.3f s into the run: qd_target %.2f -> %.2f deg/s' % (
        t_k - packets[0][0], packets[i - 1][1]['qd_target'][joint],
        packets[i][1]['qd_target'][joint]))
    for t, p in packets[max(0, i - 5):i + 4]:
        sess.log('      %+6.1f ms  mode %d  cmd_id %d  qd_target %6.2f  qd_actual %6.2f' % (
            (t - t_k) * 1e3, p['robot_mode'], p['current_command_id'],
            p['qd_target'][joint], p['qd_actual'][joint]))
    near = [r for r in rows if r.get('t_send') and t_k - 0.12 < r['t_send'] < t_k + 0.04]
    for r in near:
        sess.log('      send %+6.1f ms  reply %+6.1f ms  %s' % (
            (r['t_send'] - t_k) * 1e3, ((r['t_reply'] or r['t_send']) - t_k) * 1e3,
            (r.get('reply') or '')[:30]))


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(rate=100.0 / 3, servo_t=0.027, aheadtime=100.0)
    p.add_argument('--runs', type=int, default=40)
    p.add_argument('--stop-after', type=int, default=3, help='stop after this many knocks')
    p.add_argument('--distance', type=float, default=25.0)
    args = p.parse_args()
    s = dp.Session(args.out, 't15_knock_hunt', args)
    dash, rt = dp.connect(args, s.log)
    q0 = dp.preflight(dash, rt, s.log)
    s.log('SpeedFactor(100): %s' % dash.query('SpeedFactor(100)').raw)
    dp.confirm(['', 'Plan: up to %d J1 sweeps, +%.0f deg and back at 20 deg/s, %.1f Hz, t=%.4f, '
                'aheadtime %g, gain %g' % (args.runs, args.distance, args.rate, args.servo_t,
                                           args.aheadtime, args.gain)], args.yes, s.log)
    found = 0
    for run in range(1, args.runs + 1):
        q = dp.idle_q(rt)
        if max(abs(a - b) for a, b in zip(q, q0)) > 0.5:
            s.log('arm not back at the start pose: %s' % q)
            break
        profile, duration = dp.out_and_back_profile(q0, 1, args.distance, vmax=20.0, amax=60.0)
        dp.check_profile(profile, duration, 0.02, q0, args.distance + 1, 25.0)
        cfg = dp.StreamConfig(rate=args.rate, servo_t=args.servo_t, aheadtime=args.aheadtime,
                              gain=args.gain, timing='index')
        summary, result, packets = dp.do_run(s, dash, rt, 'run_%02d' % run, profile, duration,
                                             cfg, joint=1)
        hits = knocks(packets, 0)
        for i in hits:
            describe(s, packets, result['rows'], i, 0)
        found += len(hits)
        s.log('  run %d: %d knock(s); %d so far' % (run, len(hits), found))
        if result['abort'] or summary.get('final_mode') != 'ENABLE(5)':
            s.log('stopping: %s / %s' % (result['abort'], summary.get('final_mode')))
            break
        if found >= args.stop_after:
            break
        time.sleep(0.5)
    s.log('%d knock(s) in %d run(s)' % (found, run))
    rt.stop()
    dash.close()


if __name__ == '__main__':
    main()
