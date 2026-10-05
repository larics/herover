#!/usr/bin/env python3

import time

import serial
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistSerialBridge(Node):
    def __init__(self):
        super().__init__('twist_serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('startup_delay', 2.0)
        self.declare_parameter('config_file', '')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.startup_delay = self.get_parameter('startup_delay').value
        self.config_file = self.get_parameter('config_file').value

        self.serial_conn = None
        self.last_sent = {}

        self.open_serial()

        robots = self.load_robots(self.config_file)
        if not robots:
            raise RuntimeError(f'No robots loaded from config: {self.config_file}')

        for robot in robots:
            self.subscribe_robot(robot['id'], robot['namespace'])

        self.get_logger().info(f'Sending serial commands to {self.port} at {self.baud} baud')

    def load_robots(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('robots', [])

    def subscribe_robot(self, robot_id, namespace):
        topic = f'/{namespace}/cmd_vel'
        self.create_subscription(
            Twist,
            topic,
            lambda msg, rid=robot_id: self.twist_callback(msg, rid),
            10,
        )
        self.get_logger().info(f'Robot {robot_id} listening on {topic}')

    def open_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(self.startup_delay)
            self.get_logger().info('Serial port opened')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

    def twist_callback(self, msg: Twist, robot_id: int):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        line = f'CMD {robot_id} {linear_x:.4f} {linear_y:.4f} {angular_z:.4f}\n'

        if line == self.last_sent.get(robot_id):
            return

        try:
            self.serial_conn.write(line.encode('utf-8'))
            self.serial_conn.flush()
            self.last_sent[robot_id] = line
            self.get_logger().info(f'Sent: {line.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def destroy_node(self):
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TwistSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
