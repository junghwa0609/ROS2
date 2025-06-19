#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    """
    Safety Node that only uses /scan (LaserScan) data to avoid collisions.
    """
    def __init__(self):
        super().__init__('safety_node')

        # 퍼블리셔: /drive로 속도+조향 명령 보내기
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # 구독자: /scan 데이터 구독
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # 속도 제한
        self.max_speed = 1.5  # m/s

    def scan_callback(self, scan_msg):
        """
        LaserScan 메시지를 받아서 장애물과 거리 계산하고
        속도를 조정하여 /drive로 퍼블리시
        """
        ranges = np.array(scan_msg.ranges)
        print(ranges)

        # 무한대 (inf)나 NaN 제거
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            # 유효한 데이터가 없으면 그냥 멈추자
            self.brake()
            return

        # 가장 가까운 거리
        min_dist = np.min(valid_ranges)

        self.get_logger().info(f"Minimum distance to obstacle: {min_dist:.2f} m")

        drive_msg = AckermannDriveStamped()

        # 거리에 따라 속도 조정
        if min_dist < 0.5:
            # 0.5m 이내에 장애물 있으면 정지
            drive_msg.drive.speed = 0.0
            self.get_logger().warn("Obstacle too close! Braking!")
        elif min_dist < 1.5:
            # 0.5m ~ 1.5m 사이면 속도 낮게
            drive_msg.drive.speed = 0.5
        else:
            # 장애물 멀리 있으면 최대 1.5m/s로 주행
            drive_msg.drive.speed = self.max_speed

        # 조향각은 일단 0 (직진)
        drive_msg.drive.steering_angle = 0.0

        # 퍼블리시
        self.drive_pub.publish(drive_msg)

    def brake(self):
        """
        강제로 속도를 0으로 설정
        """
        #self.get_logger().info("braked")
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("Safety Node init")
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


