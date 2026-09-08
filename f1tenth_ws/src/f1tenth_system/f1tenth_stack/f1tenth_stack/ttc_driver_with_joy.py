# Copyright 2025 JungHwa Lee
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""LiDAR distance control with a joystick deadman's switch."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan


class TTCDriverWithJoy(Node):
    """Drive only while the configured deadman's switch is held."""

    def __init__(self):
        super().__init__('ttc_driver_with_joy')

        self.declare_parameter('deadman_button_index', 5)
        self.declare_parameter('stop_distance_m', 1.0)
        self.declare_parameter('drive_speed_mps', 1.0)

        self.deadman_button_index = self.get_parameter(
            'deadman_button_index'
        ).value
        self.stop_distance_m = self.get_parameter('stop_distance_m').value
        self.drive_speed_mps = self.get_parameter('drive_speed_mps').value
        self.deadman_pressed = False

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )

    def joy_callback(self, message):
        was_pressed = self.deadman_pressed
        button_exists = 0 <= self.deadman_button_index < len(message.buttons)
        self.deadman_pressed = (
            button_exists and bool(message.buttons[self.deadman_button_index])
        )

        if was_pressed and not self.deadman_pressed:
            self.publish_drive(0.0)

    def scan_callback(self, message):
        center_index = len(message.ranges) // 2
        center_distance = (
            message.ranges[center_index] if message.ranges else float('nan')
        )

        can_drive = (
            self.deadman_pressed
            and math.isfinite(center_distance)
            and center_distance >= self.stop_distance_m
        )
        speed = self.drive_speed_mps if can_drive else 0.0
        self.publish_drive(speed)

    def publish_drive(self, speed):
        drive_message = AckermannDriveStamped()
        drive_message.drive.speed = float(speed)
        drive_message.drive.steering_angle = 0.0
        self.drive_publisher.publish(drive_message)


def main(args=None):
    rclpy.init(args=args)
    node = TTCDriverWithJoy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
