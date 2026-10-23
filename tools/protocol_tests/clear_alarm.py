#!/usr/bin/env python3
"""Clear an alarm and re-enable the arm. Enabling releases the brakes.

    GetErrorID -> ClearError() -> EnableRobot() if disabled, Stop() if
    paused -> ENABLE(5)

After an alarm mid-stream ClearError() leaves the controller in PAUSE (10),
its queue holding the rest of the stream; Stop() empties it.

Prints the alarms it cleared. drover-arm must be stopped.
"""
import time

import dobot_proto as dp


def wait_mode(rt, wanted, timeout):
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        mode = rt.current(max_age=0.5)['robot_mode']
        if mode in wanted:
            return mode
        time.sleep(0.02)
    return rt.current(max_age=0.5)['robot_mode']


def main():
    p = dp.base_parser(__doc__.splitlines()[0])
    args = p.parse_args()
    dash, rt = dp.connect(args)
    mode = rt.current(max_age=0.5)['robot_mode']
    print('mode %s, alarms %s' % (dp.mode_name(mode), dash.query('GetErrorID()').raw))
    if mode != 5:
        print(dash.query('ClearError()').raw)
        mode = wait_mode(rt, (4, 5, 10), 3.0)
        if mode == 10:
            print(dash.query('Stop()').raw)
            mode = wait_mode(rt, (5,), 3.0)
        if mode == 4:
            print(dash.query('EnableRobot()').raw)
            mode = wait_mode(rt, (5,), 10.0)
    print('now %s, alarms %s' % (dp.mode_name(mode), dash.query('GetErrorID()').raw))
    rt.stop()
    dash.close()
    raise SystemExit(0 if mode == 5 else 1)


if __name__ == '__main__':
    main()
