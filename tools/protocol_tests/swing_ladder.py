#!/usr/bin/env python3
"""ServoJ against MovJ on one joint swing, at rising speeds. MOTION TEST (run), or analysis (analyse).

run: from --start, a joint move by --delta and back (straight in joint space,
all joints finishing together, so the tool's path is a curve), at each speed
of --speeds in turn: first --reps times out and back by ServoJ the way
t25_spike_trace.py streams it on the controller's 4 ms cycle (--cycles 6 and
--track-phase, port 65511; --hold 0.3 s at the start point for the primer MovJ),
then --reps times by MovJ at the same peak speed and acceleration of the
leading joint. A speed is the leading joint's peak, deg/s; its acceleration is
--accel-ratio x the speed (deg/s^2). MovJ's v= and a= are percentages of the
playback speed and acceleration (191 deg/s and 200 deg/s^2 for J1-J3,
234 and 1000 for J4-J6, operations.md). Before moving, the tool's peak speed
at each level is estimated from the controller's PositiveKin along the path
(ServoJ runs ~13 % over the plan); a level over --tool-limit is skipped.
Each leg's realtime feed goes to <session>/<level>_<servoj|movj>_<out|back>_<rep>/realtime.csv.
imu_record.py records the flange alongside, into <session>/../imu.csv or --imu.

analyse: per leg: duration; the leading joint's peak target speed; the tool's
peak actual speed; the target's roughness (rms of the leading joint's target
speed minus its 0.1 s moving average, deg/s: ServoJ's steps, a planned
move's ramps barely register); the flange's 4-8 Hz rms over the move and over
the 1 s after it. Needs numpy.

    python3 swing_ladder.py run --out D --speeds 30,45,60,90 --reps 2
    python3 swing_ladder.py analyse D/<session> --imu D/imu.csv
    python3 swing_ladder.py analyse D1/<session> D2/<session> ...   (one mount fit for all)
"""
import argparse
import csv
import glob
import math
import os
import re
import time

import dobot_proto as dp
import t25_spike_trace as t25

START = '-90.349,40.125,-86.737,-24.873,90.773,0.567'   # the package start pose
PLAY_V = [191.0] * 3 + [234.0] * 3      # MovJ v=100 %, deg/s
PLAY_A = [200.0] * 3 + [1000.0] * 3     # MovJ a=100 %, deg/s^2
SERVOJ_OVER = 1.13                      # ServoJ's actual peak over the plan
CYCLE = 0.004                           # s, the controller's motion cycle


def joints(s):
    return [float(v) for v in s.split(',')]


def fk(dash, q):
    r = dash.query('PositiveKin(%s)' % ','.join('%.4f' % v for v in q))
    m = re.match(r'\s*0,\{([^}]*)\}', r.raw)
    if not m:
        raise SystemExit('PositiveKin failed: %s' % r.raw)
    return [float(v) for v in m.group(1).split(',')][:3]


def mm_per_deg(dash, q0, q1, lead, n=40):
    """The tool's largest travel per degree of the leading joint along the move."""
    pts = [fk(dash, [a + (b - a) * i / n for a, b in zip(q0, q1)]) for i in range(n + 1)]
    step = abs(q1[lead] - q0[lead]) / n
    return max(math.dist(a, b) for a, b in zip(pts, pts[1:])) / step, pts


def movj(dash, rt, q, v, a, timeout=20.0):
    i0 = rt.index()
    r = dash.query('MovJ(joint={%s},v=%d,a=%d)' % (','.join('%.4f' % x for x in q), v, a))
    if r.err != 0:
        raise SystemExit('MovJ rejected: %s' % r.raw)
    t0 = dp.now()
    while dp.now() - t0 < 1.0 and rt.current(max_age=0.5)['robot_mode'] != 7:
        time.sleep(0.005)
    while dp.now() - t0 < timeout:
        mode = rt.current(max_age=0.5)['robot_mode']
        if mode in dp.ERROR_MODES:
            raise SystemExit('alarm during MovJ: %s' % dp.mode_name(mode))
        if mode == 5:
            break
        time.sleep(0.005)
    else:
        raise SystemExit('MovJ did not finish in %.0f s' % timeout)
    time.sleep(1.2)     # the settling the IMU window looks at
    return rt.slice(i0)


def run(args):
    q0, delta = joints(args.start), joints(args.delta)
    q1 = [a + d for a, d in zip(q0, delta)]
    lead = max(range(6), key=lambda j: abs(delta[j]))
    speeds = [float(v) for v in args.speeds.split(',')]
    s = dp.Session(args.out, 'swing_ladder', args)
    dash, rt = dp.connect(args, s.log)
    dp.preflight(dash, rt, s.log)
    k, pts = mm_per_deg(dash, q0, q1, lead)
    s.log('tool: from (%s) to (%s) mm, %.0f mm straight, %.0f mm along the path, z %.0f..%.0f'
          % (', '.join('%.0f' % v for v in pts[0]), ', '.join('%.0f' % v for v in pts[-1]),
             math.dist(pts[0], pts[-1]), sum(math.dist(a, b) for a, b in zip(pts, pts[1:])),
             min(p[2] for p in pts), max(p[2] for p in pts)))
    levels = []
    for v in speeds:
        acc = min(v * args.accel_ratio, PLAY_A[lead])
        f, dur = t25.scurve_1d(abs(delta[lead]), v, acc)
        peak = (f(dur / 2 + 0.002) - f(dur / 2 - 0.002)) / 0.004     # reached at mid-move
        tool = peak * k * SERVOJ_OVER
        mv, ma = round(100 * peak / PLAY_V[lead]), round(100 * acc / PLAY_A[lead])
        # the safety system trips at the playback speed (191 deg/s J1-J3), and ServoJ overshoots
        ok = (tool <= args.tool_limit and peak * SERVOJ_OVER <= PLAY_V[lead]
              and 1 <= mv <= 100 and 1 <= ma <= 100)
        s.log('level %5.1f deg/s: peak %5.1f deg/s, %5.1f deg/s^2, MovJ v=%d a=%d, tool ~%.0f mm/s%s'
              % (v, peak, acc, mv, ma, tool, '' if ok else '  SKIPPED'))
        if ok:
            levels.append((v, peak, acc, mv, ma))
    if not levels:
        raise SystemExit('no level within the limits')
    if args.plan_only:
        return
    cycles = t25.CycleClock(args.ip)
    cycles.start()
    time.sleep(2.0)
    if cycles.count < 100:
        raise SystemExit('port %d gave %d blocks in 2 s' % (t25.ALGS_LOG_PORT, cycles.count))
    dp.confirm(['', 'Plan: MovJ to [%s], then J%d leading by [%s] deg and back, %d times by ServoJ '
                'then by MovJ, at %s deg/s.' % (args.start, lead + 1, args.delta, args.reps,
                                                ', '.join('%g' % l[0] for l in levels))],
               args.yes, s.log)
    movj(dash, rt, q0, 10, 10)
    period = args.cycles * t25.ALGS_CYCLE
    for v, peak, acc, mv, ma in levels:
        sargs = argparse.Namespace(
            line=None, line_servoj=False, speed=peak, accel=acc, shape_hz=0.0, shape_zeta=0.04,
            timer=0.0, cycles=args.cycles, cycle_delay=args.cycle_delay,
            cycle_model=args.cycle_model, every=3, t_scale=1.0, hold=0.3,
            aheadtime=100.0, gain=500.0)
        for kind in ('servoj', 'movj'):
            for rep in range(args.reps):
                for name, qb in (('out', q1), ('back', q0)):
                    tag = '%03.0f_%s_%s_%d' % (v, kind, name, rep + 1)
                    phase = None
                    if kind == 'servoj':
                        phase = {'delay': args.cycle_delay, 'regime': 0, 'sent': [],
                                 'shifts': [], 'usual': None}
                        i0 = rt.index()
                        here = list(rt.current(max_age=0.5)['q_actual'])
                        t25.stream(dash, rt, here, qb, sargs, period, cycles, phase)
                        time.sleep(0.9)     # with stream's 0.3 s: the settling the IMU window looks at
                        packets = rt.slice(i0)
                    else:
                        packets = movj(dash, rt, qb, mv, ma)
                    mode = rt.current(max_age=0.5)['robot_mode']
                    dp.write_realtime_csv(os.path.join(s.run_dir(tag), 'realtime.csv'), packets)
                    note = ''
                    if phase is not None:
                        lags = [c['lag'] for c in phase['sent'] if c['lag'] is not None]
                        note = ', %d phase shifts, take-up delays %s' % (
                            len(phase['shifts']), {g: lags.count(g) for g in sorted(set(lags))})
                    s.log('%s: done, mode %s%s' % (tag, dp.mode_name(mode), note))
                    if mode != 5:
                        raise SystemExit('stopping: the arm is not in ENABLE')
                    time.sleep(0.5)
    cycles.stop()
    rt.stop()


def euler_matrix(rx, ry, rz):
    """Dobot Euler angles (deg, fixed-axis XYZ) -> rotations, R = Rz Ry Rx (hand_eye.py)."""
    import numpy as np
    ax, ay, az = np.radians(rx), np.radians(ry), np.radians(rz)
    R = np.zeros(ax.shape + (3, 3))
    cx, sx, cy, sy, cz, sz = np.cos(ax), np.sin(ax), np.cos(ay), np.sin(ay), np.cos(az), np.sin(az)
    R[..., 0, 0], R[..., 0, 1], R[..., 0, 2] = cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx
    R[..., 1, 0], R[..., 1, 1], R[..., 1, 2] = sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx
    R[..., 2, 0], R[..., 2, 1], R[..., 2, 2] = -sy, cy * sx, cy * cx
    return R


def skew(v):
    import numpy as np
    S = np.zeros(v.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2], S[..., 1, 2] = -v[..., 2], v[..., 1], -v[..., 0]
    return S - np.swapaxes(S, -1, -2)


def rigid_model(rows):
    """What a rigid arm following the plan would put on an accelerometer at the TCP:
    host times, the specific force in the tool frame (m/s^2), and M (N x 3 x 3) with
    which a point r (m, tool frame) off the TCP adds M r (alpha x r + w x (w x r)); then the
    rows without repeats and their times on the same clock."""
    import numpy as np
    c = np.array([float(r['controller_timer']) for r in rows])
    h = np.array([float(r['host_t']) for r in rows])
    keep = np.concatenate([[True], np.diff(c) > 0])     # a packet sent twice
    rows, c, h = [r for r, x in zip(rows, keep) if x], c[keep], h[keep]
    # the target advances once per 4 ms cycle and a packet shows the latest: count the
    # cycles between packets from the leading joint's step over its speed, and from the
    # controller's ms stamp where the arm is still
    q = np.array([[float(r['q_target_%d' % i]) for i in range(1, 7)] for r in rows])
    qd = np.array([[float(r['qd_target_%d' % i]) for i in range(1, 7)] for r in rows])
    lead = int(np.argmax(np.abs(qd).max(axis=0)))
    vmid = 0.5 * (qd[1:, lead] + qd[:-1, lead])
    dq = np.diff(q[:, lead])
    with np.errstate(divide='ignore', invalid='ignore'):
        dn = np.where(np.abs(vmid) > 2.0, np.round(dq / (vmid * CYCLE)), np.round(np.diff(c) / 4.0))
    dn = np.maximum(dn, 1)
    tk = np.concatenate([[0.0], np.cumsum(dn)]) * CYCLE
    pos = np.array([[float(r['tool_vector_target_%d' % i]) for i in (1, 2, 3)] for r in rows]) / 1000.0
    ang = np.array([[float(r['tool_vector_target_%d' % i]) for i in (4, 5, 6)] for r in rows])
    ang = np.degrees(np.unwrap(np.radians(ang), axis=0))
    # an even grid, then below 12 Hz: differentiating the raw 8 ms feed twice is noise
    tc = np.arange(0.0, tk[-1] + CYCLE / 2, CYCLE)
    th = tc + np.min(h - tk)        # the earliest arrival: the least delayed packet
    pos = lowpass(tc, np.stack([np.interp(tc, tk, pos[:, j]) for j in range(3)], 1), 12.0)
    Rbt = euler_matrix(*(np.interp(tc, tk, ang[:, j]) for j in range(3)))
    Rbt = lowpass(tc, Rbt.reshape(-1, 9), 12.0).reshape(-1, 3, 3)
    U, _, Vt = np.linalg.svd(Rbt)
    Rbt = U @ Vt                    # back to rotations
    acc = np.gradient(np.gradient(pos, tc, axis=0), tc, axis=0)
    f_base = acc + np.array([0.0, 0.0, 9.81])
    f_tool = np.einsum('nji,nj->ni', Rbt, f_base)
    dR = np.gradient(Rbt, tc, axis=0)
    W = np.einsum('nji,njk->nik', Rbt, dR)           # R^T dR = [w]x, w in the tool frame
    w = np.stack([W[:, 2, 1], W[:, 0, 2], W[:, 1, 0]], axis=1)
    alpha = np.gradient(w, tc, axis=0)
    M = skew(alpha) + np.einsum('nij,njk->nik', skew(w), skew(w))
    return th, f_tool, M, rows, tk + (th[0] - tc[0])


def lowpass(t, x, hz):
    import numpy as np
    # the line from the first sample to the last out, so the FFT sees no jump at the wrap
    u = ((t - t[0]) / (t[-1] - t[0])).reshape((-1,) + (1,) * (x.ndim - 1))
    trend = x[0] + (x[-1] - x[0]) * u
    F = np.fft.rfft(x - trend, axis=0)
    F[np.fft.rfftfreq(len(t), (t[-1] - t[0]) / (len(t) - 1)) > hz] = 0
    return np.fft.irfft(F, n=len(t), axis=0) + trend


def fit_mount(segs, lag, iters=6):
    """IMU = R (f + M r) + b over all legs, below 3 Hz: R by Kabsch, then r and b by
    least squares, in turn. Returns R, r, b and the rms misfit."""
    import numpy as np
    F, MM, A = [], [], []
    for th, f, M, it, acc in segs:
        m = (it >= th[0] + 0.1) & (it <= th[-1] - 0.1)
        ti = it[m] - lag
        F.append(lowpass(ti, np.stack([np.interp(ti, th, f[:, j]) for j in range(3)], 1), 3.0))
        MM.append(lowpass(ti, np.stack([np.interp(ti, th, M[:, i // 3, i % 3]) for i in range(9)],
                                       1), 3.0).reshape(-1, 3, 3))
        A.append(lowpass(ti, acc[m], 3.0))
    F, MM, A = np.concatenate(F), np.concatenate(MM), np.concatenate(A)
    r = np.zeros(3)
    for _ in range(iters):
        X = F + MM @ r
        U, _, Vt = np.linalg.svd((A - A.mean(0)).T @ (X - X.mean(0)))
        R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        # R^T IMU = f + M r + c, c = R^T b
        y = (A @ R) - F
        G = np.concatenate([MM, np.broadcast_to(np.eye(3), MM.shape)], axis=2)
        x = np.linalg.lstsq(G.reshape(-1, 6), y.reshape(-1), rcond=None)[0]
        r, b = x[:3], R @ x[3:]
    res = A - ((F + MM @ r) @ R.T + b)
    return R, r, b, float(np.sqrt(np.mean(np.sum(res ** 2, 1))))


def analyse(args):
    """Fits the camera's mount over all the sessions given (it does not change), then
    reports each session's legs."""
    import numpy as np
    from compare_lines import band_rms
    sessions = []
    for session in args.session:
        imu_path = (args.imu if args.imu and len(args.session) == 1 else
                    os.path.join(os.path.dirname(os.path.abspath(session)), 'imu.csv'))
        imu = np.loadtxt(imu_path, delimiter=',', skiprows=1, ndmin=2)
        it, acc = imu[:, 0], imu[:, 1:4]
        legs = []
        for d in sorted(x for x in glob.glob(os.path.join(session, '*_*_*_*')) if os.path.isdir(x)):
            th, f, M, rows, th_rows = rigid_model(list(csv.DictReader(open(os.path.join(d, 'realtime.csv')))))
            legs.append((os.path.basename(d), rows, th_rows, (th, f, M, it, acc)))
        sessions.append((session, legs))
    segs = [m for _, legs in sessions for _, _, _, m in legs]
    # the IMU's and the feed's clocks: the lag that fits best
    best = None
    for lag in np.arange(-0.02, 0.1001, 0.004):
        err = fit_mount(segs, lag, iters=3)[3]
        if best is None or err < best[0]:
            best = (err, lag)
    lag = best[1]
    R, r, b, err = fit_mount(segs, lag)
    print('rigid model: IMU lag %+.0f ms, IMU %.0f mm from the TCP (%s mm, tool frame), '
          'misfit below 3 Hz %.3f m/s^2' % (lag * 1e3, np.linalg.norm(r) * 1e3,
                                           ', '.join('%.0f' % v for v in r * 1e3), err))
    for session, legs in sessions:
        report(session, legs, lag, R, r, b)


def report(session, legs, lag, R, r, b):
    import numpy as np
    from compare_lines import band_rms
    print()
    print(session)
    print('%-20s %5s %6s %5s %6s %7s %7s %7s %7s' % ('leg', 'dur', 'J peak', 'tool', 'target',
                                                   'flange', 'flange', 'excess', 'excess'))
    print('%-20s %5s %6s %5s %6s %7s %7s %7s %7s' % ('', 's', 'deg/s', 'mm/s', 'rough', 'move',
                                                   '1s after', 'move', '1s after'))
    last = None
    for name, rows, th_rows, (th, f, M, it, acc) in legs:
        t = np.array([float(r_['host_t']) for r_ in rows])
        qd = np.array([[float(r_['qd_target_%d' % i]) for i in range(1, 7)] for r_ in rows])
        lead = int(np.argmax(np.abs(qd).max(axis=0)))
        v = qd[:, lead]
        tool = np.array([math.hypot(*(float(r_['tcp_speed_actual_%d' % i]) for i in (1, 2, 3)))
                         for r_ in rows])
        moving = np.flatnonzero(np.abs(v) > 0.5)
        if not len(moving):
            print('%-20s no motion' % name)
            continue
        a, b_ = moving[0], moving[-1]
        w = max(1, int(round(0.1 / np.median(np.diff(t)))))
        smooth = np.convolve(v, np.ones(w) / w, mode='same')
        rough = float(np.sqrt(np.mean((v - smooth)[a + w:b_ - w] ** 2))) if b_ - a > 3 * w else float('nan')
        ta, tb = th_rows[a], th_rows[b_]
        win = (it >= th[0] + 0.1) & (it <= th[-1] - 0.1)
        ti = it[win] - lag
        fi = np.stack([np.interp(ti, th, f[:, j]) for j in range(3)], 1)
        Mi = np.stack([np.interp(ti, th, M[:, i // 3, i % 3]) for i in range(9)], 1).reshape(-1, 3, 3)
        excess = acc[win] - ((fi + Mi @ r) @ R.T + b)
        tw = it[win]
        mv, after = (tw >= ta) & (tw <= tb), (tw > tb) & (tw <= tb + 1.0)
        level = name.split('_')[0] + name.split('_')[1]
        if last and level != last:
            print()
        last = level
        print('%-20s %5.2f %6.1f %5.0f %6.3f %7.3f %7.3f %7.3f %7.3f' % (
            name, tb - ta, np.abs(v).max(), tool[a:b_ + 1].max(), rough,
            band_rms(tw[mv], acc[win][mv], 4, 8), band_rms(tw[after], acc[win][after], 4, 8),
            band_rms(tw[mv], excess[mv], 4, 8), band_rms(tw[after], excess[after], 4, 8)))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--ip', default=dp.DEFAULT_IP)
    r.add_argument('--out', default='results')
    r.add_argument('--start', default=START, help='joints, deg')
    r.add_argument('--delta', default='80,-8,12,0,0,0', help='deg per joint')
    r.add_argument('--speeds', default='30,45,60,90', help="the leading joint's peak, deg/s")
    r.add_argument('--accel-ratio', type=float, default=2.0,
                   help='acceleration = this x the speed, deg/s^2 (capped at MovJ a=100)')
    r.add_argument('--reps', type=int, default=2)
    r.add_argument('--tool-limit', type=float, default=250.0,
                   help="the safety level's tool speed, mm/s (level 5: 250, level 4: 750)")
    r.add_argument('--cycles', type=int, default=6)
    r.add_argument('--cycle-delay', type=float, default=0.0)
    r.add_argument('--cycle-model', action='store_true',
                   help="send on t25's fitted cycle clock (not yet tested on the arm)")
    r.add_argument('--plan-only', action='store_true', help='print the levels and stop; no motion')
    r.add_argument('--yes', action='store_true')
    a = sub.add_parser('analyse')
    a.add_argument('session', nargs='+')
    a.add_argument('--imu', default=None)
    args = p.parse_args()
    (run if args.cmd == 'run' else analyse)(args)


if __name__ == '__main__':
    main()
