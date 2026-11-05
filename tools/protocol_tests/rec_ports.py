#!/usr/bin/env python3
"""Record the controller's feeds without commanding anything, whoever drives the arm.

Writes to --out until SIGINT/SIGTERM: realtime.csv (every port 30004 packet,
as in the tests), and for each log port (default 65511, the motion module's
4 ms [ch8] blocks) <port>.raw, the bytes as received, with <port>.idx, the
receive time and byte offset of each chunk. Times are this computer's
monotonic clock, as in imu_record.py.

    python3 rec_ports.py --out /tmp/cmp/session
"""
import argparse
import csv
import os
import signal
import socket
import threading
import time

import dobot_proto as dp


def log_port(ip, port, out, stop):
    s = socket.create_connection((ip, port), timeout=5)
    s.settimeout(1.0)
    with open(os.path.join(out, '%d.raw' % port), 'wb') as raw, \
            open(os.path.join(out, '%d.idx' % port), 'w') as idx:
        n = 0
        while not stop:
            try:
                b = s.recv(65536)
            except socket.timeout:
                continue
            if not b:
                break
            idx.write('%.6f %d\n' % (dp.now(), n))
            raw.write(b)
            n += len(b)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ip', default=dp.DEFAULT_IP)
    p.add_argument('--out', required=True)
    p.add_argument('--ports', default='65511', help='log ports, comma-separated')
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(True))
    threads = [threading.Thread(target=log_port, args=(args.ip, int(port), args.out, stop),
                                daemon=True) for port in args.ports.split(',')]
    for t in threads:
        t.start()
    rt = dp.RealtimeReader(args.ip)
    rt.start()
    rt.wait_first()
    written = 0
    with open(os.path.join(args.out, 'realtime.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(dp.REALTIME_CSV_HEADER)
        while True:
            done = bool(stop)
            time.sleep(0.5 if not done else 0.0)
            new = rt.slice(written)
            w.writerows(dp.realtime_row(t, pkt) for t, pkt in new)
            written += len(new)
            f.flush()
            if done:
                break
    rt.stop()
    for t in threads:
        t.join(timeout=2.0)
    print('%d realtime packets to %s' % (written, args.out))


if __name__ == '__main__':
    main()
