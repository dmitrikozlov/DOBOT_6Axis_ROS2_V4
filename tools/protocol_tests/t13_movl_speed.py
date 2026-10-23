#!/usr/bin/env python3
"""T13: MovL speed, absolute (speed=, mm/s) and as a percentage (v=). MOTION TEST.

Each run moves the tool tip in a straight line --distance mm along one base
axis and back, with MovL, and records the realtime feed. A run is a list of
settings, e.g. "sf=100,speed=100,a=20":

    sf     the global speed ratio, SpeedFactor (%), set before the run
    speed  MovL's target tool speed, mm/s (speed=), or
    v      MovL's speed ratio, % (v=); one of the two
    a      MovL's acceleration ratio, % (a=; default --accel)

For each leg it reports the tool tip's commanded and actual cruise speed
(port 30004), so a run answers what MovL really does with speed= and v=:
whether speed= is reached as given, whether SpeedFactor scales it, what v=
100 % means in mm/s, and whether a speed near the safety level's tool speed
is clamped or raises an alarm. The safety level is set in Dobot Studio; name
it with --label so the results say which level they were taken at.

    python3 t13_movl_speed.py --label level5 --axis z --distance 100
    python3 t13_movl_speed.py --label level4 --runs "sf=100,speed=100"

drover-arm must be stopped (port 29999 takes one client). The arm starts
where it stands: choose a pose with --distance mm clear along --axis.
"""
import math
import time

import dobot_proto as dp

DEFAULT_RUNS = [
    "sf=100,speed=100",     # the target speed as given?
    "sf=50,speed=100",      # does SpeedFactor scale speed=?
    "sf=100,v=5",           # v=: what is 100 %? (5 % of 2000 mm/s would be 100)
    "sf=100,v=10",
    "sf=100,speed=240",     # near the level-5 tool limit (250): clamped, or alarm 225?
]
AXES = {'x': 0, 'y': 1, 'z': 2}


def parse_run(text, accel):
    run = {}
    for item in text.split(','):
        key, value = item.split('=')
        run[key.strip()] = float(value)
    if ('speed' in run) == ('v' in run):
        raise SystemExit(f"run {text!r}: give speed= or v=, not both or neither")
    run.setdefault('sf', 100.0)
    run.setdefault('a', accel)
    return run


def movl(pose, run, user, tool):
    speed = (f"speed={run['speed']:g}" if 'speed' in run else f"v={run['v']:g}")
    return ('MovL(pose={%s},user=%d,tool=%d,a=%g,%s)'
            % (','.join(f'{v:.3f}' for v in pose), user, tool, run['a'], speed))


def wait_started(rt, timeout=2.0):
    """Until the queued MovL is running (RUNNING, 7), so a wait for idle
    that follows cannot mistake 'not started yet' for 'done'."""
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        if rt.current(max_age=0.5)['robot_mode'] in (7,) + dp.ERROR_MODES:
            return
        time.sleep(0.01)


def get_pose(dash, user, tool):
    reply = dash.query(f'GetPose(user={user},tool={tool})')
    values = [float(v) for v in reply.value.split(',')]
    if reply.err != 0 or len(values) != 6:
        raise SystemExit(f'GetPose failed: {reply.raw}')
    return values


def leg_summary(packets, t0, t1):
    """Tool-tip speed (mm/s) while the leg moved: cruise (median of the
    samples above 80 % of the peak) and peak, commanded and actual."""
    out = {}
    for kind in ('target', 'actual'):
        v = [math.sqrt(sum(p[f'tcp_speed_{kind}'][i] ** 2 for i in range(3)))
             for t, p in packets if t0 <= t <= t1]
        if not v:
            return None
        peak = max(v)
        cruise = sorted(x for x in v if x >= 0.8 * peak)
        out[kind] = (cruise[len(cruise) // 2], peak)
    out['duration'] = t1 - t0
    return out


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.add_argument('--runs', nargs='+', default=DEFAULT_RUNS)
    p.add_argument('--label', default='', help='e.g. level5: the safety level set in the Studio')
    p.add_argument('--axis', choices=AXES, default='z')
    p.add_argument('--distance', type=float, default=100.0, help='mm along --axis, then back')
    p.add_argument('--accel', type=float, default=20.0, help='a= (%%) for runs without one')
    p.add_argument('--user', type=int, default=0)
    p.add_argument('--tool', type=int, default=1)
    p.add_argument('--force', action='store_true',
                   help='allow speed > 300 mm/s, v > 15 %%, distance > 150 mm')
    args = p.parse_args()
    runs = [(text, parse_run(text, args.accel)) for text in args.runs]
    if not args.force:
        if abs(args.distance) > 150:
            raise SystemExit('distance over 150 mm: pass --force')
        for text, run in runs:
            if run.get('speed', 0) > 300 or run.get('v', 0) > 15:
                raise SystemExit(f'{text}: over 300 mm/s or 15 %: pass --force')

    s = dp.Session(args.out, 't13_movl_speed' + (f'_{args.label}' if args.label else ''), args)
    dash, rt = dp.connect(args, s.log)
    dp.preflight(dash, rt, s.log)
    for cmd in (f'User({args.user})', f'Tool({args.tool})'):
        s.log(f'{cmd}: {dash.query(cmd).raw}')
    start = get_pose(dash, args.user, args.tool)
    out = list(start)
    out[AXES[args.axis]] += args.distance
    s.log('start pose: [%s]' % ', '.join(f'{v:.1f}' for v in start))
    dp.confirm(['', f'Plan: {len(runs)} run(s), each MovL {args.distance:+.0f} mm along '
                f'{args.axis} and back, from the pose above:'] + [f'  {t}' for t, _ in runs],
               args.yes, s.log)

    results = []
    try:
        for text, run in runs:
            s.log(f'run {text}')
            sf = 'SpeedFactor(%g)' % run['sf']
            s.log(f'  {sf}: {dash.query(sf).raw}')
            i0 = rt.index()
            legs = []
            for target in (out, start):
                cmd = movl(target, run, args.user, args.tool)
                t_send = dp.now()
                reply = dash.query(cmd)
                s.log(f'  {cmd} -> {reply.raw}')
                if reply.err != 0:
                    raise RuntimeError(f'MovL rejected: {reply.raw}')
                wait_started(rt)
                mode = dp.wait_idle(dash, rt, timeout=30.0, log=s.log)
                legs.append((t_send, dp.now()))
                if mode in dp.ERROR_MODES:
                    err = dash.query('GetErrorID()').raw
                    raise RuntimeError(f'{dp.mode_name(mode)} during the move: {err}')
            packets = rt.slice(i0)
            summary = {'run': text, 'legs': []}
            for (t0, t1), name in zip(legs, ('out', 'back')):
                leg = leg_summary(packets, t0, t1)
                summary['legs'].append({name: leg})
                if leg:
                    s.log('  %-4s %.2f s; tool tip cruise %.0f mm/s commanded, %.0f actual '
                          '(peaks %.0f / %.0f)' % (name, leg['duration'], leg['target'][0],
                                                   leg['actual'][0], leg['target'][1], leg['actual'][1]))
            results.append(summary)
            dp.write_realtime_csv(f"{s.run_dir(text.replace(',', '_').replace('=', ''))}/realtime.csv",
                                  packets)
            time.sleep(0.5)
    except (KeyboardInterrupt, RuntimeError) as e:
        s.log(f'stopping: {e or "interrupted"}; Stop(): {dash.query("Stop()").raw}')
    finally:
        s.log(f"SpeedFactor(10): {dash.query('SpeedFactor(10)').raw}")
        dp.write_json(f'{s.dir}/results.json', results)
        rt.stop()
        dash.close()


if __name__ == '__main__':
    main()
