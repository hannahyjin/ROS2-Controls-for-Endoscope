#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
 
class PumpControlNode(Node):
    def __init__(self):
        super().__init__('pump_control_node')
        self.PIN = 23  # BCM23
 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN, GPIO.OUT)
        GPIO.output(self.PIN, GPIO.LOW)
 
        self.create_subscription(
            Bool,
            'pump_cmd',
            self.cmd_callback,
            10
        )
        self.get_logger().info('PumpControlNode ready, listening on /pump_cmd')
 
    def cmd_callback(self, msg: Bool):
        state = GPIO.HIGH if msg.data else GPIO.LOW
        GPIO.output(self.PIN, state)
        action = 'ON' if msg.data else 'OFF'
        self.get_logger().info(f'Pump turned {action}')
 
    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()
        self.get_logger().info('GPIO cleaned up')
 
def main(args=None):
    rclpy.init(args=args)
    node = PumpControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
