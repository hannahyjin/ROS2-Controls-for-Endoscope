#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
from time import time


class PumpControlNode(Node):
    def __init__(self):
        super().__init__('pump_control_node')


        # GPIO pin mappings
        self.PUMP_FRONT_INFLATE = 17  # A
        self.PUMP_FRONT_DEFLATE = 27  # B
        self.PUMP_REAR_INFLATE = 22   # X
        self.PUMP_REAR_DEFLATE = 23   # Y


        # Controller button indices
        self.A_BUTTON = 0
        self.B_BUTTON = 1
        self.X_BUTTON = 2
        self.Y_BUTTON = 3


        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        for pin in [self.PUMP_FRONT_INFLATE, self.PUMP_FRONT_DEFLATE,
                    self.PUMP_REAR_INFLATE, self.PUMP_REAR_DEFLATE]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)


        # ROS setup
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_timer(0.1, self.safety_timer)


        self.last_input_time = time()
        self.pressed_buttons = set()


        self.get_logger().info("PumpControlNode running with safety timeout and per-pair exclusivity")


    def joy_callback(self, msg):
        b = msg.buttons
        self.last_input_time = time()


        self.pressed_buttons.clear()


        if b[self.A_BUTTON]:
            self.pressed_buttons.add('A')
            print("Pressed A to inflate front pump")
        if b[self.B_BUTTON]:
            self.pressed_buttons.add('B')
            print("Pressed B to deflate front pump")
        if b[self.X_BUTTON]:
            self.pressed_buttons.add('X')
            print("Pressed X to inflate back pump")
        if b[self.Y_BUTTON]:
            self.pressed_buttons.add('Y')
            print("Pressed Y to deflate back pump")


        # Front pair logic (A = inflate, B = deflate)
        if 'A' in self.pressed_buttons and 'B' not in self.pressed_buttons:
            GPIO.output(self.PUMP_FRONT_INFLATE, GPIO.HIGH)
            GPIO.output(self.PUMP_FRONT_DEFLATE, GPIO.LOW)
        elif 'B' in self.pressed_buttons and 'A' not in self.pressed_buttons:
            GPIO.output(self.PUMP_FRONT_INFLATE, GPIO.LOW)
            GPIO.output(self.PUMP_FRONT_DEFLATE, GPIO.HIGH)
        else:
            GPIO.output(self.PUMP_FRONT_INFLATE, GPIO.LOW)
            GPIO.output(self.PUMP_FRONT_DEFLATE, GPIO.LOW)


        # Rear pair logic (X = inflate, Y = deflate)
        if 'X' in self.pressed_buttons and 'Y' not in self.pressed_buttons:
            GPIO.output(self.PUMP_REAR_INFLATE, GPIO.HIGH)
            GPIO.output(self.PUMP_REAR_DEFLATE, GPIO.LOW)
        elif 'Y' in self.pressed_buttons and 'X' not in self.pressed_buttons:
            GPIO.output(self.PUMP_REAR_INFLATE, GPIO.LOW)
            GPIO.output(self.PUMP_REAR_DEFLATE, GPIO.HIGH)
        else:
            GPIO.output(self.PUMP_REAR_INFLATE, GPIO.LOW)
            GPIO.output(self.PUMP_REAR_DEFLATE, GPIO.LOW)


    def safety_timer(self):
        if time() - self.last_input_time > 1.0:
            # Turn off all pumps if no recent joystick input
            GPIO.output(self.PUMP_FRONT_INFLATE, GPIO.LOW)
            GPIO.output(self.PUMP_FRONT_DEFLATE, GPIO.LOW)
            GPIO.output(self.PUMP_REAR_INFLATE, GPIO.LOW)
            GPIO.output(self.PUMP_REAR_DEFLATE, GPIO.LOW)


    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up")


def main(args=None):
    rclpy.init(args=args)
    node = PumpControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
