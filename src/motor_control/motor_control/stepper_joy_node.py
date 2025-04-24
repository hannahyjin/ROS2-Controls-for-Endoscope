#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
from time import sleep

class StepperJoyNode(Node):
    def __init__(self):
        super().__init__('stepper_joy_node')
        # ── CONFIG ──
        self.DIR_PIN    = 20
        self.STEP_PIN   = 21
        self.LB_BUTTON    = 4
        self.RB_BUTTON    = 5
        self.STEP_DELAY = 0.001
        # ────────────
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN,  GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        self._direction = None
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_timer(self.STEP_DELAY * 2, self.step_timer)
        print("Initialized")

    def joy_callback(self, msg: Joy):
        if msg.buttons[self.LB_BUTTON]:
            self._direction = GPIO.LOW    # CW
            print("Pressed LB")
        elif msg.buttons[self.RB_BUTTON]:
            self._direction = GPIO.HIGH   # CCW
            print("Pressed RB")
        else:
            self._direction = None

    def step_timer(self):
        if self._direction is None:
            return
        GPIO.output(self.DIR_PIN, self._direction)
        GPIO.output(self.STEP_PIN, GPIO.HIGH)
        sleep(self.STEP_DELAY)
        GPIO.output(self.STEP_PIN, GPIO.LOW)

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = StepperJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == 'main':
    main()

