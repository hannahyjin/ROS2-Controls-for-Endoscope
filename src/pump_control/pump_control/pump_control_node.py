#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
from time import time

class PumpControlNode(Node):
    def __init__(self):
        super().__init__('pump_control_node')

        # GPIO pin mappings for pump control (BCM)
        self.PINS = {
            'Y': 14,  # front inflate pin 11
            'A': 17,  # front deflate pin 13
            'X': 22,  # rear inflate pin 15
            'B': 24,  # rear deflate pin 16
        }
        # Controller button indices
        self.BUTTONS = {'A': 0, 'B': 1, 'X': 2, 'Y': 3}

        # Duration pump remains active after press
        self.STOP_DURATION = 5.0  # seconds

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        for pin in self.PINS.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # State tracking
        max_idx = max(self.BUTTONS.values())
        self.prev_buttons = [0] * (max_idx + 1)
        self._pressed_key = None
        self._press_time = 0

        # Subscribe to joystick
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info(
            "PumpControlNode: edge-triggered, prints once, holds for 2s"
        )

    def joy_callback(self, msg: Joy):
        now = time()
        btns = msg.buttons

        # Detect new press (edge) among mapped keys in priority order
        current = None
        for key in ['Y', 'A', 'X', 'B']:
            idx = self.BUTTONS[key]
            if btns[idx]:
                current = key
                break

        # Edge detection
        if current is not None and self.prev_buttons[self.BUTTONS[current]] == 0:
            # New key down
            self._pressed_key = current
            self._press_time = now
            self.get_logger().info(f"Pressed {current} to {self._action_map(current)}")

        # If button released, clear state
        if current is None:
            self._pressed_key = None
            self._press_time = 0
            self._set_all(False)

        # Handle active pump based on timing
        
        #this is modified code because the solenoid valve has to be opened first before the air pump, both of them being opened at the same time closes the solenoid valve and doesn't pump air. May have to seperate the solenoid valve and the air pump on different mosfets to gain control of solenoid valve and allow it to release first before the air pump
 #       if self._pressed_key:
      #      elapsed = now - self._press_time
     #       elapsed_solen = now - self._press_time
        #    if elapsed < (0.5):
    #            self._set_gpio(27, True) #this is if the solenoid val is in a different mosfet, connected to this pin
        #    	if elapsed < self.STOP_DURATION:
                # Keep that pump on                
       #            self._set_gpio(self._pressed_key, True)
     #       else:
                # Timeout reached: turn off but do not retrigger until release
     #           self._set_all(False)

        if self._pressed_key:
            elapsed = now - self._press_time
            if elapsed < self.STOP_DURATION:
                # Keep that pump on
                self._set_gpio(self._pressed_key, True)
            else:
                # Timeout reached: turn off but do not retrigger until release
                self._set_all(False)

        # Update previous button states
        for k, idx in self.BUTTONS.items():
            self.prev_buttons[idx] = btns[idx]

    def _set_gpio(self, key, state):
        # Turn only that key's pin HIGH, others LOW
        for k, pin in self.PINS.items():
            GPIO.output(pin, GPIO.HIGH if (k == key and state) else GPIO.LOW)

    def _set_all(self, state):
        level = GPIO.HIGH if state else GPIO.LOW
        for pin in self.PINS.values():
            GPIO.output(pin, level)

    def _action_map(self, key):
        return {
            'Y': 'inflate front pump',
            'A': 'deflate front pump',
            'X': 'inflate back pump',
            'B': 'deflate back pump',
        }[key]

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up")


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
