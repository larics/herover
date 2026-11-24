#!/usr/bin/env python3
"""Gamepad teleop for the herovers, mapped for a Logitech F710 in X mode.

Subscribes to ``joy`` (from ``joy_node``) and publishes ``cmd_vel``. Both
names are relative, so the namespace picks the robot
(``-r __ns:=/herover_1``). Default controls::

    LB (hold)       enable: nothing is sent unless it is held
    RB (hold)       turbo
    left stick      vx (up = forward), vy (left = strafe left)
    right stick     wz (left = turn CCW)

While enabled, a Twist goes out every 1/``rate`` s. When LB is released, or
``joy`` goes quiet for ``joy_timeout`` s (driver stopped, receiver
unplugged), zero Twists are sent for ``stop_repeat_s`` and then nothing, so
other cmd_vel publishers are not overridden. The zeros are repeated so that a
stop does not depend on a single message.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

# joy_node indices for the F710 with its back switch on X (XInput).
F710_DEFAULTS = {
    'axis_vx': 1,         # left stick vertical
    'axis_vy': 0,         # left stick horizontal
    'axis_wz': 3,         # right stick horizontal
    'enable_button': 4,   # LB
    'turbo_button': 5,    # RB; -1 disables turbo
    'speed': 0.10,        # m/s at full stick
    'turn': 0.5,          # rad/s at full stick
    'speed_turbo': 0.17,  # about the firmware's top speed (README: Units and speeds)
    'turn_turbo': 0.8,
}


def twist_from_joy(axes, buttons, p):
    """``(vx, vy, wz)`` for one Joy message, or None if enable is not held.

    ``p`` maps the keys of ``F710_DEFAULTS`` to their values. Indices outside
    the message read as a centred axis or a released button.
    """
    def axis(i):
        return max(-1.0, min(1.0, axes[i])) if 0 <= i < len(axes) else 0.0

    def pressed(i):
        return 0 <= i < len(buttons) and bool(buttons[i])

    if not pressed(p['enable_button']):
        return None
    turbo = pressed(p['turbo_button'])
    speed = p['speed_turbo'] if turbo else p['speed']
    turn = p['turn_turbo'] if turbo else p['turn']
    return (axis(p['axis_vx']) * speed, axis(p['axis_vy']) * speed, axis(p['axis_wz']) * turn)


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        self.p = {k: self.declare_parameter(k, v).value for k, v in F710_DEFAULTS.items()}
        rate = self.declare_parameter('rate', 20.0).value
        self.joy_timeout = self.declare_parameter('joy_timeout', 0.5).value
        stop_repeat_s = self.declare_parameter('stop_repeat_s', 0.5).value
        self.stop_ticks = max(1, round(stop_repeat_s * rate))

        self.cmd = None        # from the latest joy message; None = enable not held
        self.last_joy = None   # time.monotonic() of the latest joy message
        self.zeros_left = 0    # zero Twists still to send after driving

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'{self.resolve_topic_name("joy")} -> {self.resolve_topic_name("cmd_vel")}: '
            f'hold button {self.p["enable_button"]} to drive '
            f'({self.p["speed"]} m/s, {self.p["turn"]} rad/s)')

    def on_joy(self, msg):
        self.last_joy = time.monotonic()
        self.cmd = twist_from_joy(msg.axes, msg.buttons, self.p)

    def tick(self):
        fresh = self.last_joy is not None and time.monotonic() - self.last_joy < self.joy_timeout
        if self.cmd is not None and not fresh:
            self.get_logger().warning(f'no joy messages for {self.joy_timeout}s: stopping')
            self.cmd = None

        if self.cmd is not None:
            self.publish(*self.cmd)
            self.zeros_left = self.stop_ticks
        elif self.zeros_left > 0:
            self.publish(0.0, 0.0, 0.0)
            self.zeros_left -= 1

    def publish(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(msg)

    def stop_on_exit(self):
        """Send a few zeros if the robot may still be moving (Ctrl-C mid-drive)."""
        if self.cmd is None and self.zeros_left == 0:
            return
        try:
            for _ in range(3):
                self.publish(0.0, 0.0, 0.0)
        except Exception as e:  # context may already be shut down
            self.get_logger().warning(f'could not send stop on exit: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_on_exit()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
