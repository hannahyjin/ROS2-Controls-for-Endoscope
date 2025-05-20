#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import math, time

class IMUDataNode(Node):
    def __init__(self):
        super().__init__('imu_data')
        # filter parameter
        self.declare_parameter('alpha', 0.1)
        self.alpha = self.get_parameter('alpha').value
        # state for EMA filter
        self.filtered_acc = [0.0, 0.0, 0.0]
        self.filtered_gyro = [0.0, 0.0, 0.0]
        self.first_msg = True

        self.create_subscription(Imu, '/data', self.imu_callback, qos_profile=qos_profile_sensor_data)
        self.get_logger().info('Listening to /data with EMA filter (alpha=' + str(self.alpha) + ')')

    def imu_callback(self, msg: Imu):
        print('in callback')
        # orientation pass-through
        ori = msg.orientation
        # raw data
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity

        # initialize filter on first message
        if self.first_msg:
            self.filtered_acc = [acc.x, acc.y, acc.z]
            self.filtered_gyro = [gyro.x, gyro.y, gyro.z]
            self.first_msg = False

        # apply EMA filter
        a = self.alpha
        self.filtered_acc[0] = a * acc.x + (1 - a) * self.filtered_acc[0]
        self.filtered_acc[1] = a * acc.y + (1 - a) * self.filtered_acc[1]
        self.filtered_acc[2] = a * acc.z + (1 - a) * self.filtered_acc[2]
        self.filtered_gyro[0] = a * gyro.x + (1 - a) * self.filtered_gyro[0]
        self.filtered_gyro[1] = a * gyro.y + (1 - a) * self.filtered_gyro[1]
        self.filtered_gyro[2] = a * gyro.z + (1 - a) * self.filtered_gyro[2]

        # log filtered values
        self.get_logger().info(
            f"\nOrientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}\n"
            f"Filtered Acceleration: x={self.filtered_acc[0]:.3f}, y={self.filtered_acc[1]:.3f}, z={self.filtered_acc[2]:.3f}\n"
            f"Filtered Gyroscope:    x={self.filtered_gyro[0]:.3f}, y={self.filtered_gyro[1]:.3f}, z={self.filtered_gyro[2]:.3f}"
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

if __name__ == '__main__':
    main()
