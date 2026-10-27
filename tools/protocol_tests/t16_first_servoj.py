#!/usr/bin/env python3
"""T16: the first ServoJ after a controller start — does it aim at zero? MOTION TEST.

Run it after every controller start, before anything else sends a ServoJ
(stop drover-arm first: port 29999 takes one client). Like drover after a
restart, it enables the arm (EnableRobot(), if it is not enabled already),
waits --pause s, then sends --count ServoJ commands, every one to the arm's
own position, 20 ms apart with drover's settings (t 0.02, aheadtime 100, gain
500), and records the realtime feed. Leave the arm disabled after the start so
the script does the enabling.

Nothing asks for motion, so any movement of the controller's target is the
fault: on 2026-10-09, -12 and -20 the target moved every joint toward 0 by the
same fraction of its angle (e.g. x 0.9977 in one packet) and the safety TCP
speed limit raised alarm 225. The arm moves under a degree before stopping.

The case is appended to --cases (default ~/servoj_first_cases.md) with what
docs/dobot_servoj_first_stream.md asks for: the controller's run_time, what
the controller did, the per-joint fraction toward zero, the alarm and replies.
Write what came before (how long the controller was off, what you did since
it started) with --note. If it faulted, clear the alarm on the pendant or
with ClearError and run it again: the second run should be clean.

With --movj, before the pause it sends a MovJ to the arm's own joints and
waits for it to finish (RUNNING, then ENABLE): Dobot's suggested way to start
ServoJ from the current position (2026-10-27).

    python3 t16_first_servoj.py --note "off 2 h; not enabled, no jog"
    python3 t16_first_servoj.py --movj --note "power cycle; enabled in the app"
"""
import os
import time

import dobot_proto as dp

MOVE_DEG = 0.05      # the target moving this far from the commanded position is the fault


def wait_mode(rt, mode, timeout):
    """Until the feed shows `mode`; True if it did within `timeout` s."""
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        if rt.current(max_age=0.5)['robot_mode'] == mode:
            return True
        time.sleep(0.005)
    return False


def toward_zero(start, target):
    """Per joint, the fraction of its angle the target moved toward 0
    (positive: toward zero), or None for a joint near 0."""
    out = []
    for s, t in zip(start, target):
        out.append(None if abs(s) < 1.0 else round((s - t) / s, 5))
    return out


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--count', type=int, default=10, help='ServoJ commands, 20 ms apart')
    p.add_argument('--after', type=float, default=1.5, help='s of feed kept after the last')
    p.add_argument('--pause', type=float, default=0.5, help='s between ENABLE and the first ServoJ')
    p.add_argument('--note', default='', help='what came before: power-off time, what was done')
    p.add_argument('--movj', action='store_true',
                   help='MovJ to the current joints before the pause, and wait for it to finish')
    p.add_argument('--cases', default=os.path.expanduser('~/servoj_first_cases.md'))
    args = p.parse_args()
    s = dp.Session(args.out, 't16_first_servoj', args)
    dash, rt = dp.connect(args, s.log)
    pkt = rt.current(max_age=0.5)
    run_time = pkt.get('run_time')
    s.log('controller run_time %s s (up %.1f min)' % (run_time, (run_time or 0) / 60.0))
    mode = pkt['robot_mode']
    if mode in dp.ERROR_MODES:
        raise SystemExit('the controller is in %s (%s): clear the alarm first'
                         % (dp.mode_name(mode), dash.query('GetErrorID()').raw))
    enable = mode != 5
    dp.confirm(['', 'Plan: %s, %swait %.1f s, then %d ServoJ to the current position, 20 ms '
                'apart, t=%g' % ('EnableRobot() (releases the brakes)' if enable
                                 else 'the arm is already enabled',
                                 'MovJ to the current joints, ' if args.movj else '',
                                 args.pause, args.count, args.servo_t)], args.yes, s.log)
    i0 = rt.index()     # the recording covers the enable and the pause
    if enable:
        t_en = dp.now()
        s.log('EnableRobot(): %s' % dash.query('EnableRobot()').raw)
        deadline = t_en + 10.0
        while dp.now() < deadline and rt.current(max_age=0.5)['robot_mode'] != 5:
            time.sleep(0.01)
        s.log('ENABLE after %.2f s' % (dp.now() - t_en))
    else:
        s.log('already enabled: no EnableRobot()')
    dp.preflight(dash, rt, s.log)
    s.log('SpeedFactor(100): %s' % dash.query('SpeedFactor(100)').raw)
    movj = None
    if args.movj:
        here = list(rt.current(max_age=0.5)['q_actual'])
        r = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.4f' % v for v in here))
        s.log('MovJ to the current joints: %s' % r.raw)
        if r.err != 0:
            raise SystemExit('the MovJ was rejected; nothing streamed')
        t_movj = dp.now()
        # The MovJ is queued: wait for RUNNING (a zero-length move may be too
        # short for the feed to show it), then for ENABLE.
        ran = wait_mode(rt, 7, 1.0)
        idle = wait_mode(rt, 5, 10.0)
        movj = {'reply': r.raw, 'saw_running': ran, 'enable': idle,
                'done_s': round(dp.now() - t_movj, 2)}
        s.log('MovJ: %s, %s after %.2f s' % ('RUNNING seen' if ran else 'RUNNING not seen',
                                             'ENABLE' if idle else 'NOT back in ENABLE',
                                             movj['done_s']))
        if not idle:
            raise SystemExit('the controller did not return to ENABLE after the MovJ; '
                             'nothing streamed')
    time.sleep(args.pause)
    q = list(rt.current(max_age=0.5)['q_actual'])

    replies = []
    t0 = dp.now()
    for k in range(args.count):
        r = dash.query(dp.servoj_cmd(q, args.servo_t, args.aheadtime, args.gain))
        replies.append({'t_ms': round((r.t_send - t0) * 1e3, 1), 'reply': r.raw[:40]})
        if r.err != 0:
            break
        time.sleep(max(0.0, t0 + (k + 1) * 0.02 - dp.now()))
    time.sleep(args.after)
    packets = rt.slice(i0)
    err = dash.query('GetErrorID()').raw
    mode = dp.mode_name(rt.current(max_age=0.5)['robot_mode'])

    worst, worst_pkt = 0.0, None
    for t, pk in packets:
        if t < t0:      # enable and pause: not yet streaming
            continue
        d = max(abs(a - b) for a, b in zip(pk['q_target'], q))
        if d > worst:
            worst, worst_pkt = d, (t, pk)
    faulted = worst > MOVE_DEG
    s.log('%s: the controller target moved up to %.3f deg from the commanded position; '
          'mode %s; %s' % ('FAULT' if faulted else 'clean', worst, mode, err))
    case = {'time_utc': time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime()), 'run_time_s': run_time,
            'result': 'fault' if faulted else 'clean', 'max_target_move_deg': round(worst, 3),
            'mode_after': mode, 'error_ids': err, 'replies': replies, 'note': args.note,
            'enabled_by_script': enable, 'pause_s': args.pause, 'movj': movj,
            'start_deg': [round(v, 3) for v in q], 'dir': s.dir}
    if faulted:
        t, pk = worst_pkt
        case['toward_zero'] = toward_zero(q, pk['q_target'])
        case['tcp_speed_target_peak'] = round(max(
            sum(x * x for x in p2['tcp_speed_target'][:3]) ** 0.5 for _, p2 in packets), 1)
        s.log('  fraction toward zero per joint: %s' % case['toward_zero'])
    dp.write_json(os.path.join(s.dir, 'case.json'), case)
    dp.write_realtime_csv(os.path.join(s.dir, 'realtime.csv'), packets)
    with open(args.cases, 'a') as f:
        f.write('- %s UTC, controller up %.0f min, %s, %s%.1f s pause: **%s**, target moved '
                '%.3f deg, %s, %s%s%s\n' % (
            case['time_utc'], (run_time or 0) / 60.0,
            'enabled here' if enable else 'already enabled',
            'MovJ to the current joints, ' if args.movj else '', args.pause,
            case['result'], worst, mode, err,
            ('; toward zero %s' % case['toward_zero']) if faulted else '',
            ('; note: %s' % args.note) if args.note else ''))
    s.log('case appended to %s' % args.cases)
    rt.stop()
    dash.close()


if __name__ == '__main__':
    main()
