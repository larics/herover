import re
import socket
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from herover.cmd_vel_bridge import (
    DEFAULT_CMD_TIMEOUT_S, DEFAULT_RESEND_HZ, STOP_RESENDS, CmdVelBridge, _Robot,
    format_command, load_robots)

OPENRB_CONFIG = Path(__file__).parents[1] / 'firmware' / 'herover_openrb' / 'config.h'


def write_yaml(tmp_path, text):
    p = tmp_path / 'robots.yaml'
    p.write_text(text)
    return str(p)


def test_format_command():
    assert format_command(1, 0.1, -0.05, 1.5) == b'CMD 1 0.1000 -0.0500 1.5000\n'
    assert format_command(2, 0.0, 0.0, 0.0) == b'CMD 2 0.0000 0.0000 0.0000\n'


def test_load_robots(tmp_path):
    path = write_yaml(tmp_path, """
udp_port: 6000
robots:
  - {id: 1, namespace: /herover_1, host: 10.0.0.1}
  - {id: 2, namespace: herover_2, host: herover-2.local, port: 6001}
""")
    robots, port = load_robots(path)
    assert port == 6000
    assert robots == [
        {'id': 1, 'namespace': 'herover_1', 'host': '10.0.0.1', 'port': 6000},
        {'id': 2, 'namespace': 'herover_2', 'host': 'herover-2.local', 'port': 6001},
    ]


def test_default_port(tmp_path):
    robots, port = load_robots(write_yaml(tmp_path, """
robots:
  - {id: 1, namespace: herover_1, host: 10.0.0.1}
"""))
    assert port == 5005 and robots[0]['port'] == 5005


@pytest.mark.parametrize('robots_yaml, msg', [
    ('- {id: 1, namespace: a, host: h}\n  - {id: 1, namespace: b, host: h}', 'duplicate id'),
    ('- {id: 1, namespace: a, host: h}\n  - {id: 2, namespace: a, host: h}', 'duplicate namespace'),
    ('- {id: 1, namespace: a}', 'missing host'),
    ('- {id: 0, namespace: a, host: h}', 'id must be'),
    ('- {id: "1", namespace: a, host: h}', 'id must be'),
    ('- {id: 1, host: h}', 'missing namespace'),
    ('[]', 'no robots'),
])
def test_load_robots_rejects(tmp_path, robots_yaml, msg):
    path = write_yaml(tmp_path, f'robots:\n  {robots_yaml}\n')
    with pytest.raises(ValueError, match=msg):
        load_robots(path)


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.fail = False

    def sendto(self, payload, addr):
        if self.fail:
            raise OSError('network is unreachable')
        self.sent.append((payload, addr[0]))


class OfflineBridge(CmdVelBridge):
    """The bridge's send path alone: no ROS node and no network."""

    def __init__(self, *hosts):
        self.sock = FakeSocket()
        self.log = MagicMock()
        self.cmd_timeout = DEFAULT_CMD_TIMEOUT_S
        self.robots = [
            _Robot({'id': i, 'namespace': f'herover_{i}', 'host': host, 'port': 5005})
            for i, host in enumerate(hosts, start=1)]

    def get_logger(self):
        return self.log

    def age(self, seconds, i=0):
        """Make the robot's last command ``seconds`` older."""
        self.robots[i].cmd_time -= seconds

    def take_sent(self):
        sent, self.sock.sent = self.sock.sent, []
        return sent


def twist(vx, vy=0.0, wz=0.0):
    return SimpleNamespace(linear=SimpleNamespace(x=vx, y=vy), angular=SimpleNamespace(z=wz))


DRIVE = (b'CMD 1 0.1000 0.0000 0.0000\n', '10.0.0.1')
STOP = (b'CMD 1 0.0000 0.0000 0.0000\n', '10.0.0.1')


def test_nothing_resent_before_first_command():
    bridge = OfflineBridge('10.0.0.1')
    bridge.robots[0].resolve(bridge.log)
    bridge.resend()
    bridge.resend()
    assert bridge.take_sent() == []


def test_one_command_is_sent_at_once_and_then_resent():
    bridge = OfflineBridge('10.0.0.1')
    bridge.on_twist(twist(0.1), bridge.robots[0])
    assert bridge.take_sent() == [DRIVE]
    bridge.resend()  # the command is newer than the last tick
    assert bridge.take_sent() == []
    for _ in range(3):
        bridge.resend()
        assert bridge.take_sent() == [DRIVE]
    assert bridge.robots[0].sent == 1  # re-sends are not counted


def test_streamed_commands_are_not_duplicated():
    bridge = OfflineBridge('10.0.0.1')
    for _ in range(5):
        bridge.on_twist(twist(0.1), bridge.robots[0])
        bridge.resend()
    assert bridge.take_sent() == [DRIVE] * 5


def test_stop_replaces_the_command_and_is_resent():
    bridge = OfflineBridge('10.0.0.1')
    bridge.on_twist(twist(0.1), bridge.robots[0])
    bridge.on_twist(twist(0.0), bridge.robots[0])
    bridge.resend()
    bridge.take_sent()
    bridge.resend()
    assert bridge.take_sent() == [STOP]


def test_failed_stop_is_resent_not_the_command_before_it():
    bridge = OfflineBridge('10.0.0.1')
    bridge.on_twist(twist(0.1), bridge.robots[0])
    bridge.sock.fail = True
    bridge.on_twist(twist(0.0), bridge.robots[0])
    bridge.resend()
    bridge.resend()
    bridge.sock.fail = False
    bridge.take_sent()
    bridge.resend()
    assert bridge.take_sent() == [STOP]


def test_robots_are_resent_independently():
    bridge = OfflineBridge('10.0.0.1', '10.0.0.2')
    bridge.on_twist(twist(0.0), bridge.robots[1])
    bridge.resend()
    bridge.take_sent()
    bridge.on_twist(twist(0.1), bridge.robots[0])
    bridge.resend()
    assert bridge.take_sent() == [DRIVE, (b'CMD 2 0.0000 0.0000 0.0000\n', '10.0.0.2')]


def quiet_after(bridge, ticks=3 * STOP_RESENDS):
    """Payloads sent over many resend ticks."""
    bridge.take_sent()
    for _ in range(ticks):
        bridge.resend()
    return bridge.take_sent()


def test_robot_is_stopped_when_cmd_vel_goes_quiet():
    bridge = OfflineBridge('10.0.0.1')
    bridge.on_twist(twist(0.1), bridge.robots[0])
    bridge.resend()
    bridge.age(bridge.cmd_timeout - 0.2)
    bridge.take_sent()
    bridge.resend()
    assert bridge.take_sent() == [DRIVE]  # not yet
    bridge.age(0.3)
    assert quiet_after(bridge) == [STOP] * STOP_RESENDS  # then nothing more


def test_stop_survives_a_long_stall():
    # Ticks that come much too late must still send the zeros.
    bridge = OfflineBridge('10.0.0.1')
    bridge.on_twist(twist(0.1), bridge.robots[0])
    bridge.resend()
    bridge.age(60.0)
    assert quiet_after(bridge) == [STOP] * STOP_RESENDS


def test_streaming_publisher_never_times_out():
    # 1 Hz, as `ros2 topic pub` does by default.
    bridge = OfflineBridge('10.0.0.1')
    for _ in range(10):
        bridge.on_twist(twist(0.1), bridge.robots[0])
        bridge.resend()
        bridge.age(1.0)
        bridge.resend()
    assert set(bridge.take_sent()) == {DRIVE}


def test_drives_and_times_out_again_after_a_stop():
    bridge = OfflineBridge('10.0.0.1')
    for _ in range(2):
        bridge.on_twist(twist(0.1), bridge.robots[0])
        assert bridge.take_sent() == [DRIVE]
        bridge.resend()
        bridge.age(bridge.cmd_timeout + 0.1)
        assert quiet_after(bridge) == [STOP] * STOP_RESENDS


def test_one_robot_timing_out_leaves_the_other_driving():
    bridge = OfflineBridge('10.0.0.1', '10.0.0.2')
    for robot in bridge.robots:
        bridge.on_twist(twist(0.1), robot)
    bridge.resend()
    bridge.age(bridge.cmd_timeout + 0.1, i=0)
    bridge.take_sent()
    bridge.resend()
    assert bridge.take_sent() == [STOP, (b'CMD 2 0.1000 0.0000 0.0000\n', '10.0.0.2')]


def test_unresolved_host_is_not_looked_up_or_sent_by_resend(monkeypatch):
    lookups = []

    def getaddrinfo(*args):
        lookups.append(args)
        raise socket.gaierror('unknown host')

    monkeypatch.setattr(socket, 'getaddrinfo', getaddrinfo)
    bridge = OfflineBridge('herover-1.local')
    bridge.on_twist(twist(0.1), bridge.robots[0])
    assert len(lookups) == 1
    bridge.robots[0].next_resolve = 0.0  # a retry would be allowed now
    bridge.resend()
    bridge.resend()
    assert len(lookups) == 1
    assert bridge.take_sent() == []


def test_watchdog_timeout_is_many_resend_periods():
    # A few lost datagrams must not stop the robot.
    m = re.search(r'^#define\s+CMD_TIMEOUT_MS\s+(\d+)', OPENRB_CONFIG.read_text(), re.M)
    assert int(m.group(1)) / 1000.0 * DEFAULT_RESEND_HZ >= 5
