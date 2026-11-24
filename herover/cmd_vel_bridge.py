#!/usr/bin/env python3
"""cmd_vel -> WiFi bridge for the herovers.

Subscribes to ``/<namespace>/cmd_vel`` for every robot in the config and
sends each Twist, as soon as it arrives, as one UDP datagram to that robot's
ESP32::

    CMD <id> <linear.x> <linear.y> <angular.z>\\n

Units are the Twist's own (m/s, m/s, rad/s; body frame, REP-103). The robot
does the wheel kinematics. Every message is sent — a command identical to
the previous one is not suppressed. The robots never reply.

Between messages the bridge re-sends each robot's last Twist at
``resend_rate`` Hz, so a datagram lost on WiFi is replaced by the next one.
When no new Twist has arrived for ``cmd_timeout`` s it sends zeros instead,
``STOP_RESENDS`` times, and then nothing: a robot stops when its publisher
does. If the bridge, host or WiFi goes away, the robot's own watchdog stops it
(``CMD_TIMEOUT_MS`` in firmware/herover_openrb/config.h).
"""

import os
import socket
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

DEFAULT_UDP_PORT = 5005
DEFAULT_RESEND_HZ = 10.0  # many per CMD_TIMEOUT_MS (firmware/herover_openrb/config.h)
DEFAULT_CMD_TIMEOUT_S = 2.0
STOP_RESENDS = 10         # zeros sent once cmd_vel has gone quiet
RESOLVE_RETRY_S = 2.0
STATS_PERIOD_S = 5.0


def format_command(robot_id, vx, vy, wz):
    """Wire format of one command (see the module docstring)."""
    return f'CMD {robot_id} {vx:.4f} {vy:.4f} {wz:.4f}\n'.encode('ascii')


def load_robots(path):
    """Parse and validate the robots YAML. Returns ``(robots, udp_port)``.

    Each robot is a dict with ``id`` (int >= 1), ``namespace``, ``host`` and
    ``port`` (per-robot override of the top-level ``udp_port``).
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}

    udp_port = int(data.get('udp_port', DEFAULT_UDP_PORT))
    robots = []
    seen_ids, seen_ns = set(), set()
    for i, entry in enumerate(data.get('robots') or []):
        where = f'{path}: robots[{i}]'
        if not isinstance(entry, dict):
            raise ValueError(f'{where}: expected a mapping, got {entry!r}')
        rid = entry.get('id')
        if isinstance(rid, bool) or not isinstance(rid, int) or rid < 1:
            raise ValueError(f'{where}: id must be an integer >= 1, got {rid!r}')
        ns = str(entry.get('namespace') or '').strip('/')
        if not ns:
            raise ValueError(f'{where}: missing namespace')
        host = entry.get('host')
        if not host:
            raise ValueError(f'{where}: missing host (ESP32 IP or hostname)')
        if rid in seen_ids:
            raise ValueError(f'{where}: duplicate id {rid}')
        if ns in seen_ns:
            raise ValueError(f'{where}: duplicate namespace {ns}')
        seen_ids.add(rid)
        seen_ns.add(ns)
        robots.append({
            'id': rid,
            'namespace': ns,
            'host': str(host),
            'port': int(entry.get('port', udp_port)),
        })

    if not robots:
        raise ValueError(f'{path}: no robots configured')
    return robots, udp_port


class _Robot:
    """Per-robot send state: lazily resolved address, last command, counters."""

    def __init__(self, cfg):
        self.id = cfg['id']
        self.namespace = cfg['namespace']
        self.host = cfg['host']
        self.port = cfg['port']
        self.addr = None
        self.next_resolve = 0.0
        self.payload = None   # last command as sent on the wire
        self.fresh = False    # payload is newer than the last resend tick
        self.cmd_time = 0.0   # time.monotonic() of the last command
        self.zeros_left = 0   # zeros still to send once it has timed out
        self.sent = 0

    def set_command(self, payload, now):
        self.payload = payload
        self.fresh = True
        self.cmd_time = now
        self.zeros_left = STOP_RESENDS

    def take_resend(self, now, timeout):
        """Payload to send on this resend tick, or None.

        The last command while it is younger than ``timeout``, then a limited
        number of zeros, then None. Also None if a new command went out since
        the previous tick. Uses only a cached address: looking one up here
        could block, and a robot never reached has nothing to stop.
        """
        if self.fresh:
            self.fresh = False
            return None
        if self.payload is None or self.addr is None:
            return None
        if now - self.cmd_time < timeout:
            return self.payload
        if self.zeros_left == 0:
            return None
        self.zeros_left -= 1
        return format_command(self.id, 0.0, 0.0, 0.0)

    def resolve(self, logger):
        """Cached (ip, port), or None. Retries at most every RESOLVE_RETRY_S."""
        if self.addr is not None:
            return self.addr
        now = time.monotonic()
        if now < self.next_resolve:
            return None
        self.next_resolve = now + RESOLVE_RETRY_S
        try:
            info = socket.getaddrinfo(self.host, self.port, socket.AF_INET, socket.SOCK_DGRAM)
        except socket.gaierror as e:
            logger.warning(f'{self.namespace}: cannot resolve {self.host}: {e}; retrying')
            return None
        self.addr = info[0][4]
        logger.info(f'{self.namespace}: sending to {self.addr[0]}:{self.addr[1]}')
        return self.addr


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        default_cfg = os.path.join(
            get_package_share_directory('herover'), 'config', 'robots.yaml')
        self.declare_parameter('config_file', default_cfg)
        config_file = self.get_parameter('config_file').value or default_cfg

        robots, _ = load_robots(config_file)
        resend_rate = self.declare_parameter('resend_rate', DEFAULT_RESEND_HZ).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', DEFAULT_CMD_TIMEOUT_S).value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.robots = [_Robot(cfg) for cfg in robots]
        for robot in self.robots:
            robot.resolve(self.get_logger())
            topic = f'/{robot.namespace}/cmd_vel'
            self.create_subscription(
                Twist, topic, lambda msg, r=robot: self.on_twist(msg, r), 10)
            self.get_logger().info(f'robot {robot.id}: {topic} -> {robot.host}:{robot.port}')

        if resend_rate > 0.0:
            self.create_timer(1.0 / resend_rate, self.resend)
        else:
            self.get_logger().warning(
                'resend_rate <= 0: nothing is re-sent and cmd_timeout is off; '
                'only a robot with its own watchdog stops when cmd_vel does')
        self.create_timer(STATS_PERIOD_S, self.log_stats)
        self.get_logger().info(f'loaded {len(self.robots)} robot(s) from {config_file}')

    def on_twist(self, msg, robot):
        payload = format_command(robot.id, msg.linear.x, msg.linear.y, msg.angular.z)
        # Kept even if this send fails, so what gets re-sent is always the
        # newest command and never the one before it.
        robot.set_command(payload, time.monotonic())
        if robot.resolve(self.get_logger()) is None:
            return
        if self.send(robot, payload):
            robot.sent += 1
            self.get_logger().debug(f'{robot.namespace}: {payload.decode().strip()}')

    def resend(self):
        now = time.monotonic()
        for robot in self.robots:
            payload = robot.take_resend(now, self.cmd_timeout)
            if payload is not None:
                self.send(robot, payload)

    def send(self, robot, payload):
        """Send one datagram to the robot's resolved address. False if it failed."""
        try:
            self.sock.sendto(payload, robot.addr)
        except OSError as e:
            self.get_logger().warning(
                f'{robot.namespace}: send to {robot.addr[0]} failed: {e}',
                throttle_duration_sec=2.0)
            return False
        return True

    def log_stats(self):
        counts = [(r.namespace, r.sent) for r in self.robots if r.sent]
        if counts:
            self.get_logger().info(
                f'sent in last {STATS_PERIOD_S:.0f}s: '
                + ', '.join(f'{ns} {n}' for ns, n in counts))
        for r in self.robots:
            r.sent = 0

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
