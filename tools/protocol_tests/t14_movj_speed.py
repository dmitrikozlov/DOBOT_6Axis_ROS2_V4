#!/usr/bin/env python3
"""T14: MovJ speed, by SpeedFactor and by MovJ's own v=. MOTION TEST.

Each run turns one joint --distance degrees with MovJ and back, the other
joints held, and records the realtime feed. A run is a list of settings,
e.g. "sf=100,v=10":

    sf  the global speed ratio, SpeedFactor (%), set before the run
    v   MovJ's speed ratio, % (v=; left out: the controller's default)
    a   MovJ's acceleration ratio, % (a=; default --accel)

For each leg it reports the joint's commanded and actual cruise speed, so a
run answers what MovJ's 100 % is in deg/s, whether v= and SpeedFactor
multiply, and, run at two safety levels, whether the level changes it. Name
the level with --label.

    python3 t14_movj_speed.py --label level4 --joint 1 --distance 30

drover-arm must be stopped (port 29999 takes one client). The arm starts
where it stands: choose a pose where the joint can turn --distance clear.
"""
import time

import dobot_proto as dp

DEFAULT_RUNS = [
    "sf=15",            # SpeedFactor alone (v= left out), as hold-to-run Home: 5.75 deg/s at level 5
    "sf=100,v=15",      # v= alone: the same as SpeedFactor 15 %?
    "sf=100,v=5",
    "sf=100,v=30",      # in proportion?
]


def parse_run(text, accel):
    run = {}
    for item in text.split(','):
        key, value = item.split('=')
        run[key.strip()] = float(value)
    run.setdefault('sf', 100.0)
    run.setdefault('a', accel)
    return run


def movj(joints, run):
    options = 'a=%g' % run['a'] + (',v=%g' % run['v'] if 'v' in run else '')
    return 'MovJ(joint={%s},%s)' % (','.join('%.3f' % j for j in joints), options)


def wait_started(rt, timeout=2.0):
    """Until the queued MovJ is running (RUNNING, 7)."""
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        if rt.current(max_age=0.5)['robot_mode'] in (7,) + dp.ERROR_MODES:
            return
        time.sleep(0.01)


def leg_summary(packets, t0, t1, joint):
    """The joint's speed (deg/s) while the leg moved: cruise (median of the
    samples above 80 % of the peak) and peak, commanded and actual."""
    out = {}
    for kind in ('qd_target', 'qd_actual'):
        v = [abs(p[kind][joint]) for t, p in packets if t0 <= t <= t1]
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
    p.add_argument('--label', default='', help='e.g. level4: the safety level set in the Studio')
    p.add_argument('--joint', type=int, default=1, choices=range(1, 7))
    p.add_argument('--distance', type=float, default=30.0, help='deg, then back')
    p.add_argument('--accel', type=float, default=20.0, help='a= (%%) for runs without one')
    p.add_argument('--force', action='store_true',
                   help='allow sf x v over 30 %% and a distance over 45 deg')
    args = p.parse_args()
    runs = [(text, parse_run(text, args.accel)) for text in args.runs]
    if not args.force:
        if abs(args.distance) > 45:
            raise SystemExit('distance over 45 deg: pass --force')
        for text, run in runs:
            if run['sf'] * run.get('v', 100.0) / 100.0 > 30:
                raise SystemExit(f'{text}: sf x v over 30 %: pass --force')

    j = args.joint - 1
    s = dp.Session(args.out, 't14_movj_speed' + (f'_{args.label}' if args.label else ''), args)
    dash, rt = dp.connect(args, s.log)
    start = dp.preflight(dash, rt, s.log)
    out = list(start)
    out[j] += args.distance
    dp.confirm(['', f'Plan: {len(runs)} run(s), each MovJ J{args.joint} {args.distance:+.0f} deg '
                f'and back, from the joints above:'] + [f'  {t}' for t, _ in runs],
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
                cmd = movj(target, run)
                t_send = dp.now()
                reply = dash.query(cmd)
                s.log(f'  {cmd} -> {reply.raw}')
                if reply.err != 0:
                    raise RuntimeError(f'MovJ rejected: {reply.raw}')
                wait_started(rt)
                mode = dp.wait_idle(dash, rt, timeout=60.0, log=s.log)
                legs.append((t_send, dp.now()))
                if mode in dp.ERROR_MODES:
                    err = dash.query('GetErrorID()').raw
                    raise RuntimeError(f'{dp.mode_name(mode)} during the move: {err}')
            packets = rt.slice(i0)
            summary = {'run': text, 'legs': []}
            for (t0, t1), name in zip(legs, ('out', 'back')):
                leg = leg_summary(packets, t0, t1, j)
                summary['legs'].append({name: leg})
                if leg:
                    s.log('  %-4s %.2f s; J%d cruise %.2f deg/s commanded, %.2f actual '
                          '(peaks %.2f / %.2f)' % (name, leg['duration'], args.joint,
                                                   leg['qd_target'][0], leg['qd_actual'][0],
                                                   leg['qd_target'][1], leg['qd_actual'][1]))
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
