#!/usr/bin/env python3
"""T22: which TCP ports does the controller listen on, and does any but 29999 take commands? No motion.

A ros2_control hardware plugin needs a connection for ServoJ every cycle.
29999 takes one client, so if the dashboard commands (EnableRobot,
ClearError, MovL...) and the stream are to come from separate processes, as
in UR's driver, the controller needs a second command port. T2 tried
30003 (refused), 30005 and 30006; this scans every port.

1. Connects to every port in --ports (default 1-65535), --parallel at a
   time, and lists the ones that accept.
2. Holds 29999 for the rest of the run: opens it and checks RobotMode(). If
   it is closed at once, another client (the driver) already holds it, which
   is just as good.
3. On every open port except 29999, listens --listen s without sending and
   records what arrives (bytes, rate, first bytes). A port that stays silent
   then gets one RobotMode() (read only), and its reply is recorded. Ports
   with a known other protocol (--skip, default 22, 80, 443, 502) only get
   the listen. --no-send sends nothing anywhere.

A port that answers RobotMode() with "0,{...}" while 29999 is held is a
second command port.

    python3 t22_port_scan.py               # drover-arm running or stopped
    python3 t22_port_scan.py --ports 29000-31000
"""
import socket
import threading
import time

import dobot_proto as dp


def parse_ports(spec):
    ports = set()
    for part in spec.split(','):
        if '-' in part:
            a, b = part.split('-')
            ports.update(range(int(a), int(b) + 1))
        elif part:
            ports.add(int(part))
    return sorted(p for p in ports if 0 < p < 65536)


def scan(ip, ports, parallel, timeout):
    open_ports, lock = [], threading.Lock()
    todo = list(ports)

    def worker():
        while True:
            with lock:
                if not todo:
                    return
                port = todo.pop()
            try:
                s = socket.create_connection((ip, port), timeout=timeout)
                s.close()
                with lock:
                    open_ports.append(port)
            except OSError:
                pass

    threads = [threading.Thread(target=worker, daemon=True) for _ in range(parallel)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return sorted(open_ports)


def hold_dashboard(ip, log):
    """Open 29999 and keep it if it answers; returns (connection or None, what happened)."""
    try:
        d = dp.Dashboard(ip)
        time.sleep(0.3)
        r = d.query('RobotMode()')
        log('  29999: we hold it; RobotMode() -> %s' % r.raw)
        return d, 'held by this script'
    except Exception as e:
        log('  29999: not ours (%s: %s), so another client holds it' % (type(e).__name__, e))
        return None, 'held by another client'


def listen(s, seconds):
    s.settimeout(0.1)
    total, chunks, first, closed = 0, 0, None, False
    t_end = dp.now() + seconds
    while dp.now() < t_end:
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            closed = True
            break
        if first is None:
            first = chunk[:32]
        total += len(chunk)
        chunks += 1
    return {'bytes': total, 'chunks': chunks, 'closed_by_peer': closed,
            'first_bytes_hex': first.hex() if first else None,
            'first_bytes_text': first.decode('ascii', 'replace') if first else None}


def ask_robot_mode(s, seconds):
    s.sendall(b'RobotMode()')
    s.settimeout(0.1)
    buf, closed = b'', False
    t_end = dp.now() + seconds
    while dp.now() < t_end and b';' not in buf:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            closed = True
            break
        if not chunk:
            closed = True
            break
        buf += chunk
    reply = buf.decode('ascii', 'replace')
    err, value = dp.parse_reply(reply) if reply else (None, '')
    return {'reply': reply or None, 'error_id': err, 'value': value, 'closed_by_peer': closed,
            'command_port': err == 0}


def probe(ip, port, args, log):
    entry = {'port': port}
    try:
        s = socket.create_connection((ip, port), timeout=2.0)
    except OSError as e:
        entry['connect'] = '%s: %s' % (type(e).__name__, e)
        log('  %5d: reconnect failed: %s' % (port, entry['connect']))
        return entry
    try:
        heard = listen(s, args.listen)
        entry['listen'] = heard
        line = '  %5d: %d bytes in %.1f s' % (port, heard['bytes'], args.listen)
        if heard['bytes']:
            line += ' in %d chunks, first %r' % (heard['chunks'], heard['first_bytes_text'][:24])
        if heard['closed_by_peer']:
            line += ', closed by controller'
        if not args.no_send and port not in args.skip and not heard['bytes'] and not heard['closed_by_peer']:
            asked = ask_robot_mode(s, args.listen)
            entry['robot_mode'] = asked
            line += '; RobotMode() -> %s' % (asked['reply'] if asked['reply'] else
                                             'closed' if asked['closed_by_peer'] else 'no reply')
            if asked['command_port']:
                line += '  <-- COMMAND PORT'
        log(line)
    finally:
        s.close()
    return entry


def main():
    p = dp.base_parser(__doc__.splitlines()[0])
    p.add_argument('--ports', default='1-65535', help='ports to scan, e.g. 1-1024,29999,30000-30010')
    p.add_argument('--parallel', type=int, default=64, help='connects in flight (default %(default)s)')
    p.add_argument('--timeout', type=float, default=0.5, help='connect timeout, s (default %(default)s)')
    p.add_argument('--listen', type=float, default=1.0, help='listen per open port, s (default %(default)s)')
    p.add_argument('--skip', default='22,80,443,502',
                   help='open ports that get no RobotMode() (default %(default)s)')
    p.add_argument('--no-send', action='store_true', help='only connect and listen')
    args = p.parse_args()
    args.skip = set(parse_ports(args.skip))
    s = dp.Session(args.out, 't22_port_scan', args)

    ports = parse_ports(args.ports)
    s.log('== scan: %d ports on %s, %d at a time ==' % (len(ports), args.ip, args.parallel))
    t0 = dp.now()
    open_ports = scan(args.ip, ports, args.parallel, args.timeout)
    s.log('  %.0f s; open: %s' % (dp.now() - t0, ', '.join(map(str, open_ports)) or 'none'))

    s.log('\n== 29999 ==')
    dash, dash_state = hold_dashboard(args.ip, s.log)

    s.log('\n== open ports, 29999 %s ==' % dash_state)
    probes = [probe(args.ip, port, args, s.log) for port in open_ports if port != dp.DASH_PORT]
    if dash:
        dash.close()

    command_ports = [e['port'] for e in probes if e.get('robot_mode', {}).get('command_port')]
    s.log('\nsecond command port: %s' % (', '.join(map(str, command_ports)) or 'none found'))
    dp.write_json(s.dir + '/summary.json', {
        'ip': args.ip, 'ports_scanned': args.ports, 'open': open_ports,
        'dashboard_29999': dash_state, 'probes': probes, 'command_ports': command_ports})


if __name__ == '__main__':
    main()
