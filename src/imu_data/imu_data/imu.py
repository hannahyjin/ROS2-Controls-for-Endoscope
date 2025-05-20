#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import math
import time

class IMUDataNode(Node):
    def __init__(self):
        super().__init__('imu_data')
        # frequency control: one log every 0.5 seconds
        self._last_time = time.time()

        # subscribe to IMU data with sensor_data QoS
        self.create_subscription(
            Imu,
            '/data',  # topic name where IMU publishes
            self.imu_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.get_logger().info('IMUDataNode initialized, logging Euler angles at 2 Hz')

    def imu_callback(self, msg: Imu):
        now = time.time()
        # only output once every 0.5 seconds
        if now - self._last_time < 0.5:
            return
        self._last_time = now

        # convert quaternion to Euler angles (roll, pitch, yaw)
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
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
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # log Euler angles
        self.get_logger().info(
            f"Euler (deg): Roll={roll_deg:.2f}, Pitch={pitch_deg:.2f}, Yaw={yaw_deg:.2f}"
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
