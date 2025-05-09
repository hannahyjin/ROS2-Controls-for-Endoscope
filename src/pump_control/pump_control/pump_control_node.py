#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
from time import time

class PumpControlNode(Node):
    def __init__(self):
        super().__init__('pump_control_node')

        # GPIO pin mappings (BCM): Y=front inflate, A=front deflate, X=rear inflate, B=rear deflate
        self.PINS = {'Y':17, 'A':27, 'X':22, 'B':23}
        # Button to key mapping
        self.BUTTONS = {'A':0, 'B':1, 'X':2, 'Y':3}
        # How long pump stays on after press
        self.DURATION = 2.0

        GPIO.setmode(GPIO.BCM)
        for pin in self.PINS.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Track edge state
        self.prev = {btn:0 for btn in self.BUTTONS}
        self.active_key = None
        self.start_time = 0

        # ROS subscriber
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('PumpControlNode ready: edge-triggered, single print, 2s hold')

    def joy_callback(self, msg: Joy):
        now = time()
        btns = msg.buttons

        # detect edges
        for key, idx in self.BUTTONS.items():
            # falling edge from 0->1
            if btns[idx] and not self.prev[key]:
                # new press
                self.active_key = key
                self.start_time = now
                self.get_logger().info(f"Pressed {key} to {self._action(key)}")
                break
        # update prev states
        for key, idx in self.BUTTONS.items():
            self.prev[key] = btns[idx]

        # turn on/off according to active_key and duration
        if self.active_key:
            if now - self.start_time < self.DURATION:
                # pump on
                for k,p in self.PINS.items():
                    GPIO.output(p, GPIO.HIGH if k==self.active_key else GPIO.LOW)
                return
            else:
                # timeout: turn off and clear
                for p in self.PINS.values(): GPIO.output(p, GPIO.LOW)
                self.active_key = None
        else:
            # no active: ensure off
            for p in self.PINS.values(): GPIO.output(p, GPIO.LOW)

    def _action(self, k):
        return {'Y':'inflate front','A':'deflate front','X':'inflate rear','B':'deflate rear'}[k]

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

if __name__=='__main__':
    main()
