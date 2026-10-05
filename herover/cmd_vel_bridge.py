#!/usr/bin/env python3
"""cmd_vel -> WiFi bridge for the herovers.

Subscribes to ``/<namespace>/cmd_vel`` for every robot in the config and
sends each Twist, as soon as it arrives, as one UDP datagram to that robot's
ESP32::

    CMD <id> <linear.x> <linear.y> <angular.z>\\n

Units are the Twist's own (m/s, m/s, rad/s; body frame, REP-103). The robot
does the wheel kinematics. Every message is sent — a command identical to
the previous one is not suppressed — so a datagram lost on WiFi is replaced
by the next one. The robots never reply.
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
    """Per-robot send state: lazily resolved address and counters."""

    def __init__(self, cfg):
        self.id = cfg['id']
        self.namespace = cfg['namespace']
        self.host = cfg['host']
        self.port = cfg['port']
        self.addr = None
        self.next_resolve = 0.0
        self.sent = 0

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

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.robots = [_Robot(cfg) for cfg in robots]
        for robot in self.robots:
            robot.resolve(self.get_logger())
            topic = f'/{robot.namespace}/cmd_vel'
            self.create_subscription(
                Twist, topic, lambda msg, r=robot: self.on_twist(msg, r), 10)
            self.get_logger().info(f'robot {robot.id}: {topic} -> {robot.host}:{robot.port}')

        self.create_timer(STATS_PERIOD_S, self.log_stats)
        self.get_logger().info(f'loaded {len(self.robots)} robot(s) from {config_file}')

    def on_twist(self, msg, robot):
        addr = robot.resolve(self.get_logger())
        if addr is None:
            return
        payload = format_command(robot.id, msg.linear.x, msg.linear.y, msg.angular.z)
        try:
            self.sock.sendto(payload, addr)
        except OSError as e:
            self.get_logger().warning(
                f'{robot.namespace}: send to {addr[0]} failed: {e}',
                throttle_duration_sec=2.0)
            return
        robot.sent += 1
        self.get_logger().debug(f'{robot.namespace}: {payload.decode().strip()}')

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
