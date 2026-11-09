#!/usr/bin/env python3
"""The driver's fallback when port 65511 goes away. MOTION TEST.

The driver runs with redirect_65511.so preloaded, so its cycle clock connects
to a proxy here (127.0.0.1:16511) that forwards the controller's port 65511.
FJT goals of J1 (cosine moves from where the arm stands, each a new segment):

  A  proxy off from the start: 2 goals, each should pace on the feed with one
     warning ("gives no motion cycle")
  B  proxy on: 2 goals, back on the cycle ("gives the motion cycle again")
  C  a 4 s goal with the proxy switched off 1.5 s in: the stream continues on
     the feed ("stopped giving the motion cycle") and succeeds; then a goal
     back with the proxy still off

Every goal must succeed. Appends phase marks (monotonic) to --marks.

    ROS_DOMAIN_ID=77 python3 fjt_cycle_fallback.py --marks S/marks.txt
"""
import argparse
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

LISTEN = ('127.0.0.1', 16511)


class Proxy:
    """Forwards LISTEN to the controller's 65511 while on; off closes everything."""

    def __init__(self, ip):
        self.ip = ip
        self.socks = []
        self.lock = threading.Lock()
        self.srv = None

    def on(self):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(LISTEN)
        self.srv.listen(4)
        threading.Thread(target=self._accept, args=(self.srv,), daemon=True).start()

    def off(self):
        with self.lock:
            socks, self.socks = self.socks, []
        for s in [self.srv] + socks:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            s.close()

    def _accept(self, srv):
        while True:
            try:
                c, _ = srv.accept()
            except OSError:
                return
            u = socket.create_connection((self.ip, 65511), timeout=3)
            with self.lock:
                self.socks += [c, u]
            threading.Thread(target=self._pipe, args=(u, c), daemon=True).start()

    @staticmethod
    def _pipe(a, b):
        try:
            while True:
                d = a.recv(65536)
                if not d:
                    break
                b.sendall(d)
        except OSError:
            pass


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--marks', required=True)
    args = p.parse_args()
    import rclpy
    import dobot_proto as dp
    from fjt_bench import Bench, goal_for
    rt = dp.RealtimeReader(dp.DEFAULT_IP)
    rt.start()
    rt.wait_first()
    rclpy.init()
    b = Bench(rt)
    if not b.client.wait_for_server(timeout_sec=20):
        raise SystemExit('no FJT server')
    proxy = Proxy(dp.DEFAULT_IP)

    def mark(name):
        with open(args.marks, 'a') as f:
            f.write('%.3f %s\n' % (time.monotonic(), name))

    def goal(name, delta, duration=1.5):
        time.sleep(2.5)                     # past the previous goal's hold: a new segment
        mark(name)
        if not b.run(name, goal_for(b.q(), 0, delta, duration=duration), 'SUCCESSFUL'):
            proxy.off()
            raise SystemExit('stopping')

    goal('A1 proxy off', 10.0)
    goal('A2 proxy off', -10.0)
    proxy.on()
    mark('proxy on')
    time.sleep(4.0)                         # the cycle clock retries every 2 s
    goal('B1 proxy on', 10.0)
    goal('B2 proxy on', -10.0)
    t = threading.Timer(1.5 + 2.5, lambda: (proxy.off(), mark('proxy off')))
    t.start()                               # 1.5 s into C1 (after its 2.5 s pause)
    goal('C1 proxy off mid-move', 20.0, duration=4.0)
    goal('C2 proxy still off', -20.0)
    os._exit(0)


if __name__ == '__main__':
    main()
