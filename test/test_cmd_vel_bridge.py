import pytest

from herover.cmd_vel_bridge import format_command, load_robots


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
