#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class IMUDataNode(Node):
    def __init__(self):
        super().__init__('imu_data')
        # frequency control: one log per second
        self._last_time = time.time()

        # subscribe to IMU data
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.get_logger().info('IMUDataNode initialized, logging Euler angles at 1 Hz')

    def imu_callback(self, msg: Imu):
        now = time.time()
        # only output once per second
        if now - self._last_time < 1.0:
            return
        self._last_time = now

        # quaternion to Euler angles (roll, pitch, yaw)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # convert to degrees
        roll_deg  = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg   = math.degrees(yaw)

        # log only the angles
        self.get_logger().info(
            f"Euler angles (deg): Roll={roll_deg:.2f}, Pitch={pitch_deg:.2f}, Yaw={yaw_deg:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IMUDataNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
