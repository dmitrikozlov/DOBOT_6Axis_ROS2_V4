#!/usr/bin/env python3
"""T17: does the controller queue ServoJ commands? MOTION TEST.

Dobot says ServoJ is a queued command, executed one after another. The
recordings show each new ServoJ taken up within a cycle or two even with t
longer than the period. This sends the same N small steps on J6 (the wrist
roll: the tool tip barely moves) in different ways and records how the
controller's target follows:

    paced      one command per period, each after the previous reply
    burst      all N back to back, each after the previous reply (~5 ms apart)
    pipelined  all N without waiting for replies, then the replies
    prefill    the first --prefill back to back, the rest paced

Command k targets start + k x --step on J6, with t = --servo-t. If ServoJ is
queued, a burst still takes about N x t and the target passes every step,
with current_command_id stepping through the commands one t apart. If each
command replaces the last, a burst ends early and jumps over the steps.

    python3 t17_servoj_queue.py --count 10 --step 0.1

drover-arm must be stopped. The arm starts where it stands; J6 turns
count x step degrees and is returned with a slow MovJ after each mode.
"""
import os
import time

import dobot_proto as dp

J6 = 5


def send_paced(dash, cmds, period, t0):
    replies = []
    for k, cmd in enumerate(cmds):
        time.sleep(max(0.0, t0 + k * period - dp.now()))
        r = dash.query(cmd)
        replies.append((r.t_send, r.t_recv, r.raw))
        if r.err != 0:
            break
    return replies


def send_burst(dash, cmds):
    replies = []
    for cmd in cmds:
        r = dash.query(cmd)
        replies.append((r.t_send, r.t_recv, r.raw))
        if r.err != 0:
            break
    return replies


def send_pipelined(dash, cmds):
    sent = [dash.send(cmd) for cmd in cmds]
    replies = []
    for t_send in sent:
        raw, t_recv = dash.recv_reply()
        replies.append((t_send, t_recv, raw))
    return replies


def analyse(sess, packets, replies, start, targets, t_first):
    """When the target reached each commanded step, and when each command
    became the controller's current one."""
    ids = []
    for _, _, raw in replies:
        err, value = dp.parse_reply(raw)
        ids.append(int(value) if err == 0 and value.isdigit() else None)
    j = [(t, p['q_target'][J6], p['current_command_id'], p['robot_mode']) for t, p in packets]
    final = targets[-1]
    reached = [next(((t - t_first) * 1e3 for t, q, _, _ in j
                     if abs(q - start) >= abs(tg - start) - 0.005), None) for tg in targets]
    current = [next(((t - t_first) * 1e3 for t, _, cid, _ in j if cid == i), None)
               for i in ids if i is not None]
    done = next(((t - t_first) * 1e3 for t, q, _, _ in j if abs(q - final) < 0.005), None)
    sess.log('    sends at (ms): %s' % ', '.join('%.0f' % ((ts - t_first) * 1e3)
                                                for ts, _, _ in replies))
    sess.log('    target reached each step at (ms): %s' % ', '.join(
        '-' if r is None else '%.0f' % r for r in reached))
    sess.log('    each command current at (ms): %s' % ', '.join(
        '-' if c is None else '%.0f' % c for c in current))
    sess.log('    target at the final step at %s ms' % ('-' if done is None else '%.0f' % done))
    return {'reached_ms': reached, 'current_ms': current, 'done_ms': done, 'ids': ids}


def main():
    p = dp.base_parser(__doc__.splitlines()[0], motion=True)
    p.set_defaults(servo_t=0.02, aheadtime=100.0, gain=500.0)
    p.add_argument('--count', type=int, default=10)
    p.add_argument('--step', type=float, default=0.1, help='deg per command on J6')
    p.add_argument('--prefill', type=int, default=3)
    p.add_argument('--modes', default='paced,burst,pipelined,prefill')
    args = p.parse_args()
    if abs(args.count * args.step) > 3.0:
        raise SystemExit('count x step over 3 deg')
    period = 1.0 / args.rate
    s = dp.Session(args.out, 't17_servoj_queue', args)
    dash, rt = dp.connect(args, s.log)
    q0 = dp.preflight(dash, rt, s.log)
    s.log('SpeedFactor(100): %s' % dash.query('SpeedFactor(100)').raw)
    dp.confirm(['', 'Plan: for each of %s, J6 %d steps of %+.2f deg (t=%g), then a slow MovJ '
                'back' % (args.modes, args.count, args.step, args.servo_t)], args.yes, s.log)

    results = {}
    for mode in args.modes.split(','):
        start = dp.idle_q(rt)
        targets = [start[J6] + (k + 1) * args.step for k in range(args.count)]
        cmds = []
        for tg in targets:
            q = list(start)
            q[J6] = tg
            cmds.append(dp.servoj_cmd(q, args.servo_t, args.aheadtime, args.gain))
        s.log('mode %s' % mode)
        i0 = rt.index()
        time.sleep(0.2)
        t0 = dp.now()
        if mode == 'paced':
            replies = send_paced(dash, cmds, period, t0)
        elif mode == 'burst':
            replies = send_burst(dash, cmds)
        elif mode == 'pipelined':
            replies = send_pipelined(dash, cmds)
        elif mode == 'prefill':
            replies = send_burst(dash, cmds[:args.prefill])
            replies += send_paced(dash, cmds[args.prefill:], period, dp.now() + period)
        else:
            raise SystemExit(f'unknown mode {mode}')
        time.sleep(0.1)
        dp.wait_idle(dash, rt, timeout=5.0, log=s.log)
        time.sleep(0.3)
        packets = rt.slice(i0)
        bad = [raw for _, _, raw in replies if dp.parse_reply(raw)[0] != 0]
        if bad:
            s.log('    rejected: %s' % bad[0])
        results[mode] = analyse(s, packets, replies, start[J6], targets, replies[0][0])
        dp.write_realtime_csv(os.path.join(s.run_dir(mode), 'realtime.csv'), packets)
        if rt.current(max_age=0.5)['robot_mode'] in dp.ERROR_MODES:
            s.log('stopping: %s' % dash.query('GetErrorID()').raw)
            break
        back = dash.query('MovJ(joint={%s},a=20,v=5)' % ','.join('%.3f' % v for v in start))
        s.log('    back: %s' % back.raw[:40])
        time.sleep(0.2)
        dp.wait_idle(dash, rt, timeout=10.0, log=s.log)
        time.sleep(0.5)
    dp.write_json(os.path.join(s.dir, 'results.json'), results)
    rt.stop()
    dash.close()


if __name__ == '__main__':
    main()
