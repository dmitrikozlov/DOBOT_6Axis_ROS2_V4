#!/usr/bin/env python3
"""Ripple of a joint's target and actual velocity during constant-velocity
stretches, from recorded port-30004 feeds (realtime.csv). Standard library and
numpy only.

    python3 ripple.py DIR [--joint 1]

Every realtime.csv under DIR is split into sweeps (motion separated by at least
0.3 s of rest). In each sweep, a stretch is a maximal run of packets whose
qd_target has one sign and is within 15 % of the sweep's plateau (the median of
|qd_target| above 80 % of its maximum). Ripple of a stretch is the 3rd-to-97th
percentile spread of |qd_target| (target ripple) or |qd_actual| (actual ripple)
over its middle 60 %, as a percentage of the plateau. A sweep's ripple is the
mean over its stretches (normally one out, one back).
"""
import argparse
import csv
import os

import numpy as np


def load(path, joint):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['host_t']) for r in rows])
    qdt = np.array([float(r['qd_target_%d' % joint]) for r in rows])
    qda = np.array([float(r['qd_actual_%d' % joint]) for r in rows])
    return t, qdt, qda


def sweeps(t, qdt, rest_speed=0.5, rest_time=0.3):
    """Index ranges of motion separated by at least rest_time at rest."""
    moving = np.abs(qdt) > rest_speed
    out, start, last = [], None, None
    for i in np.where(moving)[0]:
        if start is None:
            start = i
        elif t[i] - t[last] > rest_time:
            out.append((start, last + 1))
            start = i
        last = i
    if start is not None:
        out.append((start, last + 1))
    return out


def spread(x, plateau):
    return 100.0 * (np.percentile(x, 97) - np.percentile(x, 3)) / plateau


def sweep_ripple(qdt, qda):
    speed = np.abs(qdt)
    if speed.max() <= 0:
        return None
    plateau = np.median(speed[speed > 0.8 * speed.max()])
    target, actual = [], []
    for sign in (1, -1):
        idx = np.where((sign * qdt > 0) & (np.abs(speed - plateau) < 0.15 * plateau))[0]
        if len(idx) < 10:
            continue
        runs = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
        k = max(runs, key=len)
        k = k[int(len(k) * 0.2):int(len(k) * 0.8)]
        if len(k) < 5:
            continue
        target.append(spread(np.abs(qdt[k]), plateau))
        actual.append(spread(np.abs(qda[k]), plateau))
    if not target:
        return None
    return plateau, float(np.mean(target)), float(np.mean(actual)), len(target)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('dir')
    p.add_argument('--joint', type=int, default=1)
    args = p.parse_args()
    print('%-70s %5s %10s %8s %8s' % ('recording / sweep', 'n', 'plateau', 'target%', 'actual%'))
    for root, _, files in sorted(os.walk(args.dir)):
        if 'realtime.csv' not in files:
            continue
        t, qdt, qda = load(os.path.join(root, 'realtime.csv'), args.joint)
        name = os.path.relpath(root, args.dir)
        for i, (a, b) in enumerate(sweeps(t, qdt)):
            r = sweep_ripple(qdt[a:b], qda[a:b])
            if r is None:
                continue
            plateau, target, actual, n = r
            print('%-70s %5d %7.2f d/s %8.1f %8.1f' % ('%s #%d' % (name, i + 1), n, plateau, target, actual))


if __name__ == '__main__':
    main()
