#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from collections import deque
import math
import time

class IMUDataNode(Node):
    def __init__(self):
        super().__init__('imu_data')
        # frequency control: one log per 0.5 seconds
        self._last_time = time.time()
        # calibration offsets (set on first callback)
        self._calibrated = False
        self._roll_offset = 0.0
        self._pitch_offset = 0.0
        self._yaw_offset = 0.0

        # subscribe to IMU data with sensor_data QoS
        self.create_subscription(
            Imu,
            '/data',  # topic name where IMU publishes
            self.imu_callback,
            qos_profile=qos_profile_sensor_data
        )
        self._buffer = deque(maxlen = 120)
        self.get_logger().info('IMUDataNode initialized, logging calibrated Euler angles at 2 Hz')

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

        # perform calibration on first valid reading
        if not self._calibrated:
            self._roll_offset = roll_deg
            self._pitch_offset = pitch_deg
            self._yaw_offset = yaw_deg
            self._calibrated = True
            self.get_logger().info(
                f"Calibrated: roll_offset={self._roll_offset:.2f}, "
                f"pitch_offset={self._pitch_offset:.2f}, "
                f"yaw_offset={self._yaw_offset:.2f}"
            )
            return

        # subtract offsets
        roll_cal  = roll_deg  - self._roll_offset
        pitch_cal = pitch_deg - self._pitch_offset
        yaw_cal   = yaw_deg   - self._yaw_offset

        # log calibrated angles
        self.get_logger().info(
            f"Euler (deg): Roll={roll_cal:.2f}, "
            f"Pitch={pitch_cal:.2f}, Yaw={yaw_cal:.2f}"
        )
        line = f"Euler (deg): Roll={roll_cal:.2f}, Pitch={pitch_cal:.2f}, Yaw={yaw_cal:.2f}"
        self._buffer.append(line)
        
#        with open('/imu_last120.log', 'w') as f:
#            f.write('\n'.join(self._buffer) + '\n')        


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
