import pytest

from herover.joy_teleop import F710_DEFAULTS, twist_from_joy

P = F710_DEFAULTS

# F710 in X mode at rest: 8 axes (triggers rest at 1.0), 11 buttons.
REST_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
LB, RB = 4, 5


def joy(axes=None, pressed=()):
    a = list(REST_AXES)
    for i, v in (axes or {}).items():
        a[i] = v
    return a, [1 if i in pressed else 0 for i in range(11)]


def test_nothing_without_enable():
    assert twist_from_joy(*joy({1: 1.0}), P) is None
    assert twist_from_joy(*joy({1: 1.0}, pressed={RB}), P) is None


def test_enabled_at_rest_is_zero():
    assert twist_from_joy(*joy(pressed={LB}), P) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('axes, expected', [
    ({1: 1.0}, (0.10, 0.0, 0.0)),    # left stick up: forward
    ({1: -1.0}, (-0.10, 0.0, 0.0)),  # down: backward
    ({0: 1.0}, (0.0, 0.10, 0.0)),    # left stick left: strafe left (+y)
    ({3: 1.0}, (0.0, 0.0, 0.5)),     # right stick left: turn CCW (+z)
    ({1: 0.5, 0: -0.5, 3: -1.0}, (0.05, -0.05, -0.5)),
])
def test_sticks(axes, expected):
    assert twist_from_joy(*joy(axes, pressed={LB}), P) == pytest.approx(expected)


def test_turbo():
    assert twist_from_joy(*joy({1: 1.0, 3: 1.0}, pressed={LB, RB}), P) == pytest.approx(
        (P['speed_turbo'], 0.0, P['turn_turbo']))


def test_turbo_disabled():
    p = dict(P, turbo_button=-1)
    assert twist_from_joy(*joy({1: 1.0}, pressed={LB, RB}), p) == pytest.approx((0.10, 0, 0))


def test_axes_clamped():
    assert twist_from_joy(*joy({1: 5.0}, pressed={LB}), P) == pytest.approx((0.10, 0, 0))


def test_short_message():
    # Indices beyond the message (another gamepad) read as centred / released.
    assert twist_from_joy([1.0, 1.0], [0, 0, 0, 0, 1], P) == pytest.approx((0.10, 0.10, 0))
    assert twist_from_joy([1.0, 1.0], [1], P) is None
