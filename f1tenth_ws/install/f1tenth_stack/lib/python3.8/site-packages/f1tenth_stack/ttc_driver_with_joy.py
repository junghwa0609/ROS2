# 파일명 예시: ttc_driver_with_joy.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class TTCDriverWithJoy(Node):
    def __init__(self):
        super().__init__('ttc_driver_with_joy')

        # /scan 구독 (LiDAR)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # /joy 구독 (조이스틱)
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # /drive 퍼블리셔
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.rb_pressed = False  # RB 버튼 상태 저장

    def joy_callback(self, msg):
        # RB 버튼 인덱스는 5번 
        self.rb_pressed = bool(msg.buttons[5])
        self.get_logger().info(f"RB Pressed: {self.rb_pressed}")

    def scan_callback(self, msg):
        if not self.rb_pressed:
            # RB 버튼 안 누르면 정지
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_publisher.publish(drive_msg)
            return

        ranges = msg.ranges
        center_index = len(ranges) // 2
        front_distance = ranges[center_index]

        speed = 1.0  # 기본 속도
        steering_angle = 0.0  # 직진

        # 장애물 가까우면 멈춤
        if front_distance < 1.0:
            speed = 0.0

        # 속도 1.5m/s 넘지 않게 제한
        speed = min(speed, 1.5)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle

        self.drive_publisher.publish(drive_msg)
        self.get_logger().info(f"Published speed: {speed:.2f}, steering: {steering_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TTCDriverWithJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# 이 코드는 ROS2 환경에서 LiDAR와 조이스틱을 사용하여 차량을 제어하는 예시입니다.
# 조이스틱의 RB 버튼을 누르면 차량이 주행하고, 버튼을 떼면 차량이 멈춥니다.
# 장애물과의 거리에 따라 속도를 조절합니다.
# 이 코드를 실행하기 위해서는 ROS2와 관련된 패키지들이 설치되어 있어야 합니다.
# 이 코드는 ROS2 환경에서 LiDAR와 조이스틱을 사용하여 차량을 제어하는 예시입니다.
