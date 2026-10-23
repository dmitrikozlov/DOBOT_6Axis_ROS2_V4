#!/usr/bin/env python3
"""Does the driver diagnose the lurch after Stop()? MOTION TEST.

Through a running driver: an FJT goal moves J1 +--distance deg; once it
succeeds (the arm has arrived, the servo session still open) the driver's Stop
service is called, then MovL --lift mm up, ~4.5 s idle, MovJ back to the
start, and a second FJT goal of the same move. That sequence made the next
stream drive J1 the wrong way (t19, 10 of 10). Expected: the second goal
aborts with the driver's start-of-stream diagnosis. Afterwards the alarm is
cleared through the driver and the arm returned to the start.

    ROS_DOMAIN_ID=42 python3 fjt_lurch.py

Needs the driver running and the arm enabled; J1 clear --distance deg above the
start and 5 deg below.
"""
import os
import sys
import time

import rclpy
from dobot_msgs_v4.srv import ClearError, EnableRobot, GetPose, MovJ, MovL, Stop

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dobot_proto as dp  # noqa: E402
import fjt_bench as fb  # noqa: E402

SRV = '/dobot_bringup_ros2/srv/'


def call(node, srv_type, name, **fields):
    client = node.create_client(srv_type, SRV + name)
    if not client.wait_for_service(timeout_sec=5.0):
        raise SystemExit(f'no service {name}')
    req = srv_type.Request()
    for k, v in fields.items():
        setattr(req, k, v)
    f = client.call_async(req)
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    return f.result()


def wait_idle(rt, timeout=15.0):
    time.sleep(0.2)
    deadline = dp.now() + timeout
    while dp.now() < deadline:
        if rt.current(max_age=0.5)['robot_mode'] in (5, 9, 11):
            return rt.current(max_age=0.5)['robot_mode']
        time.sleep(0.02)
    return rt.current(max_age=0.5)['robot_mode']


def main():
    distance, lift = 5.0, 10.0
    rt = dp.RealtimeReader(dp.DEFAULT_IP)
    rt.start()
    rt.wait_first()
    rclpy.init()
    b = fb.Bench(rt)
    if not b.client.wait_for_server(timeout_sec=20):
        raise SystemExit('no FJT server')
    q0 = b.q()

    b.run('first goal, J1 +%g deg' % distance, fb.goal_for(q0, 0, distance), 'SUCCESSFUL')
    print('Stop:', call(b, Stop, 'Stop').res, ' mode', dp.mode_name(wait_idle(rt)))
    pose = call(b, GetPose, 'GetPose', user=0, tool=1).robot_return
    xyz = [float(v) for v in pose[pose.index('{') + 1:pose.index('}')].split(',')[:6]]
    up = list(xyz)
    up[2] += lift
    params = ['user=0', 'tool=1', 'a=20', 'speed=20']
    before = b.q()
    r = call(b, MovL, 'MovL', mode=False, a=up[0], b=up[1], c=up[2], d=up[3], e=up[4], f=up[5],
             param_value=params)
    t_movl = dp.now()
    while dp.now() < t_movl + 2.0 and rt.current(max_age=0.5)['robot_mode'] != 7:
        time.sleep(0.01)
    print('MovL up:', r.robot_return[:40], ' mode', dp.mode_name(wait_idle(rt)),
          ' joints moved %.3f deg' % max(abs(x - y) for x, y in zip(b.q(), before)))
    time.sleep(4.5)
    r = call(b, MovJ, 'MovJ', mode=True, a=q0[0], b=q0[1], c=q0[2], d=q0[3], e=q0[4], f=q0[5],
             param_value=['a=20', 'v=5'])
    print('MovJ back:', r.res, ' mode', dp.mode_name(wait_idle(rt)))
    time.sleep(0.5)

    b.run('second goal, J1 +%g deg' % distance, fb.goal_for(b.q(), 0, distance),
          'PATH_TOLERANCE_VIOLATED')
    time.sleep(1.0)
    mode = rt.current(max_age=0.5)['robot_mode']
    print('after: mode', dp.mode_name(mode), ' J1 %+.2f deg from the start' % (b.q()[0] - q0[0]))
    if mode != 5:
        call(b, ClearError, 'ClearError')
        time.sleep(0.5)
        if rt.current(max_age=0.5)['robot_mode'] == 10:
            call(b, Stop, 'Stop')
        if rt.current(max_age=0.5)['robot_mode'] == 4:
            call(b, EnableRobot, 'EnableRobot')
        print('cleared: mode', dp.mode_name(wait_idle(rt)))
    call(b, MovJ, 'MovJ', mode=True, a=q0[0], b=q0[1], c=q0[2], d=q0[3], e=q0[4], f=q0[5],
         param_value=['a=20', 'v=5'])
    wait_idle(rt)
    print('back at the start: J1 %+.3f deg' % (b.q()[0] - q0[0]))
    b.destroy_node()
    rclpy.shutdown()
    rt.stop()


if __name__ == '__main__':
    main()
