import math

import pytest

from herover.pose_filter import TwistEstimator, body_twist, rotate


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


ORIGIN = (0.0, 0.0, 0.0)
Q0 = yaw_quat(0.0)


def test_rotate_yaw():
    assert rotate(yaw_quat(math.pi / 2), (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))


@pytest.mark.parametrize('yaw, step, v_body', [
    (0.0, (0.1, 0.0, 0.0), (0.1, 0.0, 0.0)),           # facing +x, moving +x: forward
    (0.0, (0.0, 0.1, 0.0), (0.0, 0.1, 0.0)),           # facing +x, moving +y: strafe left
    (math.pi / 2, (0.0, 0.1, 0.0), (0.1, 0.0, 0.0)),   # facing +y, moving +y: forward
    (math.pi / 2, (0.1, 0.0, 0.0), (0.0, -0.1, 0.0)),  # facing +y, moving +x: strafe right
])
def test_linear_velocity_is_body_frame(yaw, step, v_body):
    q = yaw_quat(yaw)
    v, w = body_twist(ORIGIN, q, step, q, 1.0)
    assert v == pytest.approx(v_body, abs=1e-12)
    assert w == pytest.approx(ORIGIN, abs=1e-12)


@pytest.mark.parametrize('yaw0, yaw1, wz', [
    (0.0, 0.1, 1.0),                   # CCW
    (0.0, -0.1, -1.0),                 # CW
    (math.pi - 0.05, -math.pi + 0.05, 1.0),  # CCW across +-pi
])
def test_yaw_rate(yaw0, yaw1, wz):
    v, w = body_twist(ORIGIN, yaw_quat(yaw0), ORIGIN, yaw_quat(yaw1), 0.1)
    assert w == pytest.approx((0.0, 0.0, wz), abs=1e-9)


def test_quaternion_sign_flip_is_no_rotation():
    q = yaw_quat(0.3)
    neg = tuple(-c for c in q)
    v, w = body_twist(ORIGIN, q, ORIGIN, neg, 0.01)
    assert w == pytest.approx(ORIGIN, abs=1e-9)


def test_estimator_first_pose_is_zero_then_measures():
    est = TwistEstimator(tau=0.05, max_gap=0.5)
    assert est.update(0.0, ORIGIN, Q0) == (ORIGIN, ORIGIN)
    v, w = est.update(0.01, (0.001, 0.0, 0.0), Q0)  # first difference is used as is
    assert v == pytest.approx((0.1, 0.0, 0.0))


def test_estimator_filters():
    est = TwistEstimator(tau=0.05, max_gap=0.5)
    est.update(0.0, ORIGIN, Q0)
    est.update(0.01, (0.001, 0.0, 0.0), Q0)          # 0.1 m/s
    v, _ = est.update(0.02, (0.003, 0.0, 0.0), Q0)   # 0.2 m/s raw
    a = 0.01 / (0.05 + 0.01)
    assert v[0] == pytest.approx(0.1 + a * 0.1)


def test_estimator_converges_on_arc():
    # 1 s of 240 Hz mocap: driving forward along a 0.3 m radius circle at
    # 0.5 rad/s, heading along the path, so the body twist is constant.
    est = TwistEstimator(tau=0.05, max_gap=0.5)
    r, rate = 0.3, 0.5
    for i in range(241):
        t = i / 240
        a = rate * t
        v, w = est.update(t, (r * math.sin(a), r * (1 - math.cos(a)), 0.0), yaw_quat(a))
    assert v == pytest.approx((r * rate, 0.0, 0.0), abs=1e-3)
    assert w == pytest.approx((0.0, 0.0, rate), abs=1e-9)


def test_estimator_restarts_after_gap():
    est = TwistEstimator(tau=0.05, max_gap=0.5)
    est.update(0.0, ORIGIN, Q0)
    est.update(0.01, (0.001, 0.0, 0.0), Q0)
    assert est.update(2.0, (5.0, 0.0, 0.0), Q0) == (ORIGIN, ORIGIN)  # not 2.5 m/s
    v, _ = est.update(2.01, (5.0, 0.002, 0.0), Q0)  # fresh first difference
    assert v == pytest.approx((0.0, 0.2, 0.0))


def test_estimator_ignores_stale_stamp():
    est = TwistEstimator(tau=0.05, max_gap=0.5)
    est.update(0.0, ORIGIN, Q0)
    before = est.update(0.01, (0.001, 0.0, 0.0), Q0)
    assert est.update(0.01, (9.0, 9.0, 0.0), Q0) == before
    assert est.update(0.005, (9.0, 9.0, 0.0), Q0) == before
    assert est.prev[0] == 0.01


@pytest.mark.parametrize('p, q', [
    ((math.nan, 0.0, 0.0), Q0),
    (ORIGIN, (0.0, 0.0, 0.0, 0.0)),
    (ORIGIN, (math.nan, 0.0, 0.0, 1.0)),
])
def test_estimator_rejects_bad_pose(p, q):
    est = TwistEstimator(tau=0.05, max_gap=0.5)
    est.update(0.0, ORIGIN, Q0)
    assert est.update(0.01, p, q) is None
    assert est.prev[0] == 0.0
