#!/usr/bin/env python3
"""How smoothly each line leg ran, from what the Pi recorded, whoever sent the commands.

Reads a session recorded by rec_ports.py and imu_record.py: realtime.csv and
65511.raw/.idx under <session>/ports, <session>/imu.csv, and marks.txt (one
"<monotonic s> <phase>" line per phase start; without it the whole session is
one phase). A leg is a stretch of motion (target tool speed above 2 mm/s)
whose tool moves --length mm (+-15 %) end to end.

Per leg: duration; peak target tool speed; the target and actual tool speed's
ripple in the cruise (3rd-97th percentile
spread over the cruise's middle 60 %, % of the cruise speed; the cruise is
where |target| is within 15 % of its median over the leg's middle half); the gaps between
commands in 4 ms controller cycles (how many port 65511 blocks each queue
number lasts, field 84); and the flange's acceleration rms in the 4-8 Hz band
(the arm's 6 Hz mode) over the leg plus 0.5 s, and over the cruise alone, each
with its slow trend removed and a Hann window (band_rms).
With --joints, legs are joint moves instead: a stretch where some joint's
target speed is above 1 deg/s and some joint travels at least 10 deg; speed
and ripple are of the joint that travels furthest (deg/s), and the leg column
names it and its travel.
Needs numpy.

    python3 compare_lines.py SESSION --length 300
    python3 compare_lines.py SESSION --joints
"""
import argparse
import bisect
import csv
import os
import re
from collections import Counter

import numpy as np

BLOCK = re.compile(rb'\n1:')     # a block's first line is field 1
EXECUTING = re.compile(rb'\n84:(\d+)')


def band_rms(t, x, lo, hi):
    """rms of x (samples x axes) between lo and hi Hz, after removing each axis's slow
    trend (order-5 polynomial) and with a Hann window: a sensor turning against gravity
    would otherwise leak its several m/s2 of drift into every band."""
    if len(t) < 8:
        return float('nan')
    fs = (len(t) - 1) / (t[-1] - t[0])
    u = np.linspace(-1.0, 1.0, len(t))
    y = np.array(x, dtype=float)
    for k in range(y.shape[1]):
        y[:, k] -= np.polyval(np.polyfit(u, y[:, k], 5), u)
    w = np.hanning(len(t))[:, None]
    y *= w / np.sqrt(np.mean(w ** 2))
    F = np.fft.rfft(y, axis=0)
    f = np.fft.rfftfreq(len(y), 1.0 / fs)
    F[(f < lo) | (f > hi)] = 0
    y = np.fft.irfft(F, n=len(y), axis=0)
    return float(np.sqrt(np.mean(np.sum(y * y, axis=1))))


def ripple(v):
    """3rd-97th percentile spread over the middle 60 %, % of the median."""
    n = len(v)
    mid = v[int(n * 0.2):int(n * 0.8)]
    if len(mid) < 5:
        return float('nan')
    return float((np.percentile(mid, 97) - np.percentile(mid, 3)) / np.median(mid) * 100)


def load_blocks(ports):
    """(host time, queue number executed) of every block on port 65511."""
    raw = open(os.path.join(ports, '65511.raw'), 'rb').read()
    idx = np.loadtxt(os.path.join(ports, '65511.idx'), ndmin=2)
    times, offsets = idx[:, 0], idx[:, 1].astype(int).tolist()
    starts = [m.start() for m in BLOCK.finditer(raw)]
    out_t, out_q = [], []
    for a, b in zip(starts, starts[1:]):
        m = EXECUTING.search(raw, a, b)
        if m:
            out_t.append(times[max(0, bisect.bisect_right(offsets, a) - 1)])
            out_q.append(int(m.group(1)))
    return np.array(out_t), np.array(out_q)


def gaps(bt, bq, t0, t1):
    """Blocks per queue number between t0 and t1, without the first and last."""
    q = bq[(bt >= t0) & (bt <= t1)]
    if len(q) == 0:
        return Counter()
    runs, n = [], 1
    for a, b in zip(q, q[1:]):
        if b == a:
            n += 1
        else:
            runs.append(n)
            n = 1
    return Counter(runs[1:-1])


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('session')
    p.add_argument('--length', type=float, nargs='+', default=[300.0, 141.4],
                   help='leg lengths to accept, mm')
    p.add_argument('--joints', action='store_true', help='legs by joint motion (movj_vs_fjt.py)')
    args = p.parse_args()
    ports = os.path.join(args.session, 'ports')
    rows = list(csv.DictReader(open(os.path.join(ports, 'realtime.csv'))))
    t = np.array([float(r['host_t']) for r in rows])
    vt = np.array([[float(r['tcp_speed_target_%d' % i]) for i in (1, 2, 3)] for r in rows])
    va = np.array([[float(r['tcp_speed_actual_%d' % i]) for i in (1, 2, 3)] for r in rows])
    pos = np.array([[float(r['tool_vector_actual_%d' % i]) for i in (1, 2, 3)] for r in rows])
    st, sa = np.linalg.norm(vt, axis=1), np.linalg.norm(va, axis=1)
    qdt = np.array([[float(r['qd_target_%d' % i]) for i in range(1, 7)] for r in rows])
    qda = np.array([[float(r['qd_actual_%d' % i]) for i in range(1, 7)] for r in rows])
    q = np.array([[float(r['q_actual_%d' % i]) for i in range(1, 7)] for r in rows])
    imu = np.loadtxt(os.path.join(args.session, 'imu.csv'), delimiter=',', skiprows=1, ndmin=2)
    it, acc = imu[:, 0], imu[:, 1:4]
    bt, bq = load_blocks(ports)
    marks = []
    if os.path.exists(os.path.join(args.session, 'marks.txt')):
        for line in open(os.path.join(args.session, 'marks.txt')):
            a, b = line.split()
            marks.append((float(a), b))

    def phase(x):
        names = [n for m, n in marks if m <= x]
        return names[-1] if names else '-'

    moving = np.abs(qdt).max(axis=1) > 1.0 if args.joints else st > 2.0
    edges = np.flatnonzero(np.diff(moving.astype(int)))
    starts = [e + 1 for e in edges if not moving[e]]
    ends = [e for e in edges if moving[e]]
    print('%-16s %-7s %6s %7s %8s %8s  %-28s %9s %9s'
          % ('phase', 'leg', 'dur s', 'peak', 'ripple', 'ripple', 'gaps, cycles (count)',
             'flange', 'flange'))
    print('%-16s %-7s %6s %7s %8s %8s  %-28s %9s %9s'
          % ('', 'deg' if args.joints else 'mm', '', 'deg/s' if args.joints else 'mm/s',
             'target%', 'actual%', '', 'leg m/s2', 'cruise'))
    legs = {}
    for a in starts:
        later = [e for e in ends if e > a]
        if later:
            legs[later[0]] = a     # the latest start before each end
    for b, a in sorted(legs.items()):
        if args.joints:
            travel = np.abs(q[b] - q[a])
            j = int(np.argmax(travel))
            if b - a < 50 or travel[j] < 10:
                continue
            label = 'J%d %.0f' % (j + 1, travel[j])
            st, sa = np.abs(qdt[:, j]), np.abs(qda[:, j])
        else:
            length = np.linalg.norm(pos[b] - pos[a])
            if b - a < 50 or not any(abs(length - L) < 0.15 * L for L in args.length):
                continue
            label = '%.0f' % length
        peak = st[a:b + 1].max()
        n = b - a
        plateau = np.median(st[a + n // 4:b - n // 4])
        cruise = np.flatnonzero(np.abs(st[a:b + 1] - plateau) < 0.15 * plateau) + a
        c0, c1 = cruise[0], cruise[-1]
        leg = (it >= t[a]) & (it <= t[b] + 0.5)
        cr = (it >= t[c0]) & (it <= t[c1])
        g = gaps(bt, bq, t[a], t[b])
        name = phase(t[a])
        print('%-16s %-7s %6.2f %7.1f %8.2f %8.2f  %-28s %9.3f %9.3f' % (
            name[:16], label, t[b] - t[a], peak, ripple(st[c0:c1 + 1]),
            ripple(sa[c0:c1 + 1]), str(dict(sorted(g.items())))[:28] if g else '-',
            band_rms(it[leg], acc[leg], 4.0, 8.0), band_rms(it[cr], acc[cr], 4.0, 8.0)))


if __name__ == '__main__':
    main()
