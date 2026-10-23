#!/usr/bin/env python3
"""T19: a MovL sent right after a ServoJ stream — when does it run? MOTION TEST.

After the last ServoJ the controller stays RUNNING (7) for ~1.7 s before it
returns to ENABLE (5), and drover waits for ENABLE before a MovL. This streams
a short J1 move, then sends a MovL --lift mm up along z, at one of --cases:

    0.3, 0.7, ...  seconds after the last ServoJ
    enable         once the controller is back in ENABLE, as drover does now
    stop           Stop() once the arm has arrived (J1 within --tol deg for
                   50 ms), then the MovL at once
    stop+enable    Stop() once the arm has arrived, the MovL on ENABLE

and records from the feed whether the MovL was accepted, when the tool
started moving, when ENABLE came, where J1 was against the streamed final
point when the MovL went, and any alarm.

    python3 t19_movl_after_stream.py --cases 0.3,0.7,1.2,enable
    python3 t19_movl_after_stream.py --cases enable,stop,stop+enable

drover-arm must be stopped. Each case: J1 turns --distance deg, the tool rises
--lift mm, comes back down, and J1 returns with a slow MovJ.
"""
import os
import time

import dobot_proto as dp
from t13_movl_speed import get_pose
from t18_stream_end import stream

MOVING_MM_S = 5.0     # tool target speed above this: the MovL has started


def linear(v):
    return (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5


def movl(pose, args):
    return 'MovL(pose={%s},user=%d,tool=%d,a=20,speed=%g)' % (
        ','.join(f'{v:.3f}' for v in pose), args.user, args.tool, args.speed)


def wait_enable(rt, timeout=5.0):
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        mode = rt.current(max_age=0.5)['robot_mode']
        if mode == 5 or mode in dp.ERROR_MODES:
            return mode
        time.sleep(0.005)
    return rt.current(max_age=0.5)['robot_mode']


def wait_arrival(rt, final, tol, timeout=1.5):
    """Until J1 has stayed within tol of final for 50 ms; the time, or None."""
    deadline = dp.now() + timeout
    since = None
    while dp.now() < deadline:
        t = dp.now()
        if abs(rt.current(max_age=0.5)['q_actual'][0] - final[0]) <= tol:
            since = since or t
            if t - since >= 0.05:
                return t
        else:
            since = None
        time.sleep(0.004)
    return None


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--cases', default='0.3,0.7,1.2,enable')
    p.add_argument('--tol', type=float, default=0.01, help='J1 deg counted as arrived')
    p.add_argument('--no-user-tool', action='store_true',
                   help='leave the controller\'s global User and Tool as they are (the '
                        'MovL still names --user and --tool)')
    p.add_argument('--return', dest='ret', choices=('both', 'movl', 'movj', 'none'),
                   default='both',
                   help='how each case returns before the next: MovL down, then MovJ to '
                        'where the case began (both), only one of them, or none (the next '
                        'case streams from the top of the MovL)')
    p.add_argument('--no-id-wait', action='store_true',
                   help='do not wait for the MovL\'s command id in the feed (it never shows, '
                        'so the wait lasts its full 4 s)')
    p.add_argument('--movl-delay', type=float, default=0.0,
                   help='s to wait after Stop() (stop cases) before the MovL')
    p.add_argument('--pose-after-stop', action='store_true',
                   help='read the pose (GetPose) just before the MovL, not while the '
                        'servo session is still open after the stream')
    p.add_argument('--distance', type=float, default=5.0, help='J1 deg streamed')
    p.add_argument('--lift', type=float, default=10.0, help='mm the MovL raises the tool')
    p.add_argument('--speed', type=float, default=20.0, help='MovL speed=, mm/s')
    p.add_argument('--user', type=int, default=0)
    p.add_argument('--tool', type=int, default=1)
    args = p.parse_args()
    if abs(args.distance) > 15 or abs(args.lift) > 50:
        raise SystemExit('distance over 15 deg or lift over 50 mm')
    s = dp.Session(args.out, 't19_movl_after_stream', args)
    dash, rt = dp.connect(args, s.log)
    dp.preflight(dash, rt, s.log)
    setup = ['SpeedFactor(100)']
    if not args.no_user_tool:
        setup += [f'User({args.user})', f'Tool({args.tool})']
    for cmd in setup:
        s.log(f'{cmd}: {dash.query(cmd).raw}')
    time.sleep(0.1)
    wait_enable(rt)   # Tool() is queued: RUNNING while it applies
    dp.confirm(['', f'Plan: for each of {args.cases}: J1 {args.distance:+g} deg by ServoJ, '
                f'MovL {args.lift:+g} mm in z at {args.speed:g} mm/s, back down, MovJ back'],
               args.yes, s.log)

    results = []
    for delay in args.cases.split(','):
        q = dp.idle_q(rt)
        i0 = rt.index()
        try:
            t_last, final, _ = stream(dash, q, 0, args.distance, args)
        except RuntimeError as e:
            time.sleep(1.0)
            packets = rt.slice(i0)
            dp.write_realtime_csv(os.path.join(s.run_dir(f'failed_{delay}'), 'realtime.csv'), packets)
            s.log(f'{delay}: {e}; {dash.query("GetErrorID()").raw}; feed saved')
            break
        if not args.pose_after_stop:
            pose = get_pose(dash, args.user, args.tool)
        note = ''
        if delay.startswith('stop'):
            t_arr = wait_arrival(rt, final, args.tol)
            stop = dash.query('Stop()')
            note = (f'arrived {"-" if t_arr is None else "%.0f" % ((t_arr - t_last) * 1e3)} ms, '
                    f'Stop() at {(stop.t_send - t_last) * 1e3:.0f} ms ({stop.raw[:10]}); ')
            if delay == 'stop+enable':
                time.sleep(0.05)
                wait_enable(rt)
            time.sleep(args.movl_delay)
        elif delay == 'enable':
            wait_enable(rt)
        else:
            time.sleep(max(0.0, t_last + float(delay) - dp.now()))
        if args.pose_after_stop:
            pose = get_pose(dash, args.user, args.tool)
        up = list(pose)
        up[2] += args.lift
        pkt = rt.current(max_age=0.5)
        mode_at_send = pkt['robot_mode']
        j1_off = pkt['q_actual'][0] - final[0]
        r = dash.query(movl(up, args))
        cmd_id = int(r.value) if r.err == 0 and r.value.isdigit() else None
        # A brief ENABLE between the stream and the MovL must not pass for its end.
        deadline = dp.now() + (0.0 if args.no_id_wait else 4.0)
        while dp.now() < deadline and rt.current(max_age=0.5)['current_command_id'] != cmd_id:
            time.sleep(0.005)
        time.sleep(0.2)
        end_mode = dp.wait_idle(dash, rt, timeout=10.0, log=s.log)
        time.sleep(0.3)
        packets = rt.slice(i0)
        after = [(t, pk) for t, pk in packets if t >= r.t_send]
        current_at = next((t for t, pk in after if pk['current_command_id'] == cmd_id), None)
        moving_at = next((t for t, pk in after
                          if linear(pk['tcp_speed_target']) > MOVING_MM_S), None)
        enable_at = next((t for t, pk in packets if t > t_last and pk['robot_mode'] == 5), None)
        alarms = sorted({pk['robot_mode'] for _, pk in packets if pk['robot_mode'] in dp.ERROR_MODES})
        reached = abs(rt.current(max_age=0.5)['q_actual'][0] - q[0]) > 0.5 * abs(args.distance)
        rel = lambda t, ref: '-' if t is None else '%.0f' % ((t - ref) * 1e3)
        s.log(f'{delay:>11}: {note}MovL sent {rel(r.t_send, t_last)} ms after the last ServoJ, J1 '
              f'{j1_off:+.4f} deg from the final point, in mode '
              f'{mode_at_send}, reply {r.raw[:14]}; its id current {rel(current_at, r.t_send)} ms '
              f'and the tool moving {rel(moving_at, r.t_send)} ms after the send '
              f'({rel(moving_at, t_last)} ms after the last ServoJ); ENABLE came '
              f'{rel(enable_at, t_last)} ms after the last ServoJ; alarms {alarms or "none"}')
        results.append({'case': delay, 'mode_at_send': mode_at_send, 'reply': r.raw,
                        'j1_offset_deg': j1_off, 'note': note,
                        'send_ms': (r.t_send - t_last) * 1e3,
                        'current_after_send_ms': None if current_at is None else (current_at - r.t_send) * 1e3,
                        'moving_after_send_ms': None if moving_at is None else (moving_at - r.t_send) * 1e3,
                        'enable_after_stream_ms': None if enable_at is None else (enable_at - t_last) * 1e3,
                        'alarms': alarms, 'end_mode': end_mode, 'streamed': reached})
        dp.write_realtime_csv(os.path.join(s.run_dir(f'delay_{delay}'), 'realtime.csv'), packets)
        if alarms or rt.current(max_age=0.5)['robot_mode'] != 5:
            s.log('stopping: %s' % dash.query('GetErrorID()').raw)
            break
        if args.ret in ('both', 'movl'):
            down = dash.query(movl(pose, args))
            s.log(f'        back: {down.raw[:14]}')
            time.sleep(0.2)
            dp.wait_idle(dash, rt, timeout=10.0, log=s.log)
        if args.ret in ('both', 'movj'):
            back = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.3f' % v for v in q))
            s.log(f'        back: {back.raw[:14]}')
            time.sleep(0.2)
            dp.wait_idle(dash, rt, timeout=15.0, log=s.log)
        time.sleep(0.5)
    dp.write_json(os.path.join(s.dir, 'results.json'), results)
    rt.stop()
    dash.close()


if __name__ == '__main__':
    main()
