#!/usr/bin/env python3
"""Record the OAK camera's IMU (accelerometer and gyroscope) to a CSV.

The flange camera's IMU measures what the tool does: its acceleration (m/s^2)
and rotation rate (rad/s), in the IMU's own axes. Timestamps are the host's
monotonic clock (time.monotonic), the clock of dobot_proto's realtime CSVs, so
the two line up; t_rx is when the host received the sample's batch. Runs until
--duration s pass or SIGINT/SIGTERM. Needs the camera free (stop
drover-harvester) and depthai (/opt/drover/env/bin/python3).

    /opt/drover/env/bin/python3 imu_record.py --out imu.csv --duration 600
"""
import argparse
import csv
import signal
import time

import depthai as dai


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--out', default='imu.csv')
    p.add_argument('--rate', type=int, default=250, help='Hz (the BMI270 tops out at 250)')
    p.add_argument('--duration', type=float, default=600.0, help='s')
    args = p.parse_args()

    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(True))

    with dai.Pipeline() as pipeline, open(args.out, 'w', newline='') as f:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], args.rate)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(20)
        q = imu.out.createOutputQueue(maxSize=200, blocking=False)
        w = csv.writer(f)
        w.writerow(['t_acc', 'ax', 'ay', 'az', 't_gyro', 'gx', 'gy', 'gz', 't_rx'])
        pipeline.start()
        t_end = time.monotonic() + args.duration
        n = 0
        while pipeline.isRunning() and not stop and time.monotonic() < t_end:
            data = q.tryGet()
            if data is None:
                time.sleep(0.005)
                continue
            t_rx = time.monotonic()
            for pk in data.packets:
                a, g = pk.acceleroMeter, pk.gyroscope
                w.writerow(['%.6f' % a.getTimestamp().total_seconds(), '%.5f' % a.x, '%.5f' % a.y,
                            '%.5f' % a.z, '%.6f' % g.getTimestamp().total_seconds(),
                            '%.6f' % g.x, '%.6f' % g.y, '%.6f' % g.z, '%.6f' % t_rx])
                n += 1
        print('%d samples to %s' % (n, args.out))


if __name__ == '__main__':
    main()
