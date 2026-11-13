#!/usr/bin/env python3
"""OptiTrack relay: motion_capture_tracking ``/poses`` -> per-robot topics.

Copied from hero_mrs_control_stack (``hero_mrs_ros2/pose_filter.py``), with
odometry added. For every rigid body in each ``NamedPoseArray`` on ``/poses``
whose name does not start with one of ``exclude_prefixes`` (default ``cf``:
the Crazyflies estimate their own pose), it publishes::

    /<name>/pose   geometry_msgs/PoseStamped   the mocap pose, as received
    /<name>/odom   nav_msgs/Odometry           the same pose, plus a velocity

Publishers are created the first time a name is seen. Both topics keep the
mocap header (stamp, and frame ``world`` by default). ``child_frame_id`` is
the rigid-body name, the TF frame motion_capture_tracking broadcasts for it.

The robots send nothing back, so there is no wheel odometry. The odom
velocity is the finite difference of consecutive mocap poses, in the body
frame (REP-105: the twist is in ``child_frame_id``, so ``linear.x`` is
forward speed and ``linear.y`` is strafe), low-pass filtered with time
constant ``velocity_tau`` s. After a gap longer than ``max_gap`` s (body
lost, mocap paused) the estimate starts again. Covariances are left at zero.

``publish_pose:=false`` publishes odom only. Use it when hero_mrs_control_stack
runs its own ``pose_filter`` (multi_agent launch with ``support:=true``):
two relays would publish every ``/<name>/pose`` twice.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

ZERO = (0.0, 0.0, 0.0)


# Quaternions are (x, y, z, w) tuples, in ROS field order.

def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_normalized(q):
    """``q`` scaled to unit length, or None if it is zero or not finite."""
    n = math.sqrt(sum(c * c for c in q))
    if not math.isfinite(n) or n < 1e-9:
        return None
    return tuple(c / n for c in q)


def rotate(q, v):
    """Vector ``v`` rotated by unit quaternion ``q``."""
    x, y, z, _ = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), quat_conj(q))
    return (x, y, z)


def body_twist(p0, q0, p1, q1, dt):
    """Body-frame ``(v, w)`` that moves pose ``(p0, q0)`` to ``(p1, q1)`` in ``dt`` s.

    Positions are world-frame, quaternions unit. ``v`` is in the frame of
    ``q1``, ``w`` is the body angular velocity (rad/s about x, y, z).
    """
    v = rotate(quat_conj(q1), tuple((b - a) / dt for a, b in zip(p0, p1)))
    # Rotation from body frame 0 to body frame 1, in frame 0.
    x, y, z, w = quat_mul(quat_conj(q0), q1)
    if w < 0.0:  # q and -q are the same orientation: take the short way round
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-12:
        return v, ZERO
    k = 2.0 * math.atan2(s, w) / (s * dt)
    return v, (x * k, y * k, z * k)


class TwistEstimator:
    """Filtered body-frame twist of one rigid body from its stream of poses."""

    def __init__(self, tau, max_gap):
        self.tau = tau
        self.max_gap = max_gap
        self.prev = None     # (t, p, q) of the last pose used
        self.primed = False  # True once v, w hold a measurement
        self.v = ZERO
        self.w = ZERO

    def update(self, t, p, q):
        """Add the pose ``(p, q)`` seen at time ``t`` (s). Returns ``(v, w)``.

        Returns None, and changes nothing, if the pose is not finite. A
        stamp that is not newer than the last one returns the last estimate
        unchanged.
        """
        q = quat_normalized(q)
        if q is None or not all(math.isfinite(c) for c in p):
            return None
        if self.prev is not None:
            t0, p0, q0 = self.prev
            dt = t - t0
            if dt <= 0.0:
                return self.v, self.w
            if dt > self.max_gap:
                self.primed = False
                self.v = self.w = ZERO
            else:
                v, w = body_twist(p0, q0, p, q, dt)
                if self.primed:
                    a = dt / (self.tau + dt)
                    v = tuple(o + a * (n - o) for o, n in zip(self.v, v))
                    w = tuple(o + a * (n - o) for o, n in zip(self.w, w))
                self.v, self.w, self.primed = v, w, True
        self.prev = (t, p, q)
        return self.v, self.w


class _Body:
    """Publishers and velocity state for one rigid body."""

    def __init__(self, pose_pub, odom_pub, twist):
        self.pose_pub = pose_pub
        self.odom_pub = odom_pub
        self.twist = twist


class PoseFilterNode(Node):
    def __init__(self):
        super().__init__('pose_filter')
        # Imported here so the tests can import this module without it.
        from motion_capture_tracking_interfaces.msg import NamedPoseArray

        self.declare_parameter('exclude_prefixes', ['cf'])
        self.declare_parameter('publish_pose', True)
        self.declare_parameter('velocity_tau', 0.05)
        self.declare_parameter('max_gap', 0.5)
        self.exclude = tuple(self.get_parameter('exclude_prefixes').value or ())
        self.publish_pose = bool(self.get_parameter('publish_pose').value)
        self.velocity_tau = float(self.get_parameter('velocity_tau').value)
        self.max_gap = float(self.get_parameter('max_gap').value)

        # IMPORTANT: match mocap QoS (BEST_EFFORT)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.callback,
            qos
        )

        # One _Body per named pose, created lazily as names are seen.
        self.bodies = {}

    def _get_body(self, name):
        body = self.bodies.get(name)
        if body is None:
            pose_pub = None
            if self.publish_pose:
                pose_pub = self.create_publisher(PoseStamped, f'/{name}/pose', 10)
            odom_pub = self.create_publisher(Odometry, f'/{name}/odom', 10)
            body = _Body(pose_pub, odom_pub, TwistEstimator(self.velocity_tau, self.max_gap))
            self.bodies[name] = body
            topics = f'/{name}/pose and /{name}/odom' if self.publish_pose else f'/{name}/odom'
            self.get_logger().info(f'Publishing "{name}" on {topics}')
        return body

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:  # unstamped: fall back to arrival time
            t = self.get_clock().now().nanoseconds * 1e-9

        for item in msg.poses:
            # Crazyflies get their pose from onboard/SITL estimation, not the
            # mocap relay — skip any cf_* object so we don't stomp it.
            if item.name.startswith(self.exclude):
                continue
            body = self._get_body(item.name)

            if body.pose_pub is not None:
                out = PoseStamped()
                out.header = msg.header
                out.pose = item.pose
                body.pose_pub.publish(out)

            p, q = item.pose.position, item.pose.orientation
            twist = body.twist.update(t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            if twist is None:
                continue
            (vx, vy, vz), (wx, wy, wz) = twist
            odom = Odometry()
            odom.header = msg.header
            odom.child_frame_id = item.name
            odom.pose.pose = item.pose
            lin, ang = odom.twist.twist.linear, odom.twist.twist.angular
            lin.x, lin.y, lin.z = vx, vy, vz
            ang.x, ang.y, ang.z = wx, wy, wz
            body.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
