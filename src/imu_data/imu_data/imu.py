#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUDataNode(Node):
    def init(self):
        super().init('imu_data')
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.get_logger().info('Listening to /imu/data...')

    def imu_callback(self, msg: Imu):
        ori = msg.orientation
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity

        self.get_logger().info(
            f"\nOrientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}\n"
            f"Acceleration: x={acc.x:.3f}, y={acc.y:.3f}, z={acc.z:.3f}\n"
            f"Gyroscope:    x={gyro.x:.3f}, y={gyro.y:.3f}, z={gyro.z:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if name == 'main':
    main()
