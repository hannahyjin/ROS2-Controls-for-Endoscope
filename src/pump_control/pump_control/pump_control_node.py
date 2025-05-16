#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
from time import time

class PumpControlNode(Node):
    def __init__(self):
        super().__init__('pump_control_node')

        # GPIO pin mappings (BCM): solenoid and pump for each action
        self.SOL_PINS = {  # solenoid valves
            'Y': 14,  # front inflate solenoid
            'A': 17,  # front deflate solenoid
            'X': 22,  # back inflate solenoid
            'B': 24,  # back deflate solenoid
        }
        self.PUMP_PINS = {  # air pumps
            'Y': 15,  # front inflate pump
            'A': 18,  # front deflate pump
            'X': 23,  # back inflate pump
            'B': 27,  # back deflate pump
        }
        # Controller button indices
        self.BUTTONS = {'A': 0, 'B': 1, 'X': 2, 'Y': 3}

        # Timing parameters
        self.SOL_DELAY     = 0.5  # seconds before activating pump
        self.STOP_DURATION = 5.0  # total seconds to hold before auto-stop

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        for pin in list(self.SOL_PINS.values()) + list(self.PUMP_PINS.values()):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # State tracking for edge detection
        self.prev = {k: 0 for k in self.BUTTONS}
        self._pressed_key = None
        self._press_time = 0

        # ROS subscription and control timer
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_timer(0.1, self.control_timer)

        self.get_logger().info(
            f"PumpControlNode ready: solenoid→pump sequencing, hold ≤{self.STOP_DURATION}s"
        )

    def joy_callback(self, msg: Joy):
        now = time()
        btns = msg.buttons

        # Detect rising edge in defined priority order
        for key in ['Y', 'A', 'X', 'B']:
            idx = self.BUTTONS[key]
            if btns[idx] and not self.prev[key]:
                self._pressed_key = key
                self._press_time = now
                self.get_logger().info(
                    f"Pressed {key} → {self._action(key)} (solenoid first)"
                )
                break
        # Update previous button states and handle release
        for key, idx in self.BUTTONS.items():
            if not btns[idx] and self.prev[key] and self._pressed_key == key:
                # On release, reset
                self._clear_all()
                self._pressed_key = None
                self._press_time = 0
            self.prev[key] = btns[idx]

    def control_timer(self):
        if not self._pressed_key:
            return
        now = time()
        elapsed = now - self._press_time
        key = self._pressed_key
        sol_pin = self.SOL_PINS[key]
        pump_pin = self.PUMP_PINS[key]
        if elapsed < self.SOL_DELAY:
            # Solenoid only
            GPIO.output(sol_pin, GPIO.HIGH)
            GPIO.output(pump_pin, GPIO.LOW)
        elif elapsed < self.STOP_DURATION:
            # Solenoid + pump
            GPIO.output(sol_pin, GPIO.HIGH)
            GPIO.output(pump_pin, GPIO.HIGH)
        else:
            # Timeout: reset all
            self._clear_all()
            self._pressed_key = None
            self._press_time = 0

    def _clear_all(self):
        # Turn off all solenoids and pumps
        for pin in self.SOL_PINS.values():
            GPIO.output(pin, GPIO.LOW)
        for pin in self.PUMP_PINS.values():
            GPIO.output(pin, GPIO.LOW)

    def _action(self, key):
        names = {
            'Y': 'front inflate',
            'A': 'front deflate',
            'X': 'back inflate',
            'B': 'back deflate',
        }
        return names[key]

    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info('GPIO cleaned up')
        super().destroy_node()


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
