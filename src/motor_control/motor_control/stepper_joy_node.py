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
        self.DIR1_PIN    = 20
        self.STEP1_PIN   = 21
        self.DIR2_PIN    = 10
        self.STEP2_PIN   = 11
        self.LB_BUTTON    = 4
        self.RB_BUTTON    = 5
        self.DPAD_UP     = 13
        self.DPAD_DOWN   = 14
        self.STEP_DELAY = 0.001
        # ────────────
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR1_PIN,  GPIO.OUT)
        GPIO.setup(self.STEP1_PIN, GPIO.OUT)
        GPIO.setup(self.DIR2_PIN,  GPIO.OUT)
        GPIO.setup(self.STEP2_PIN, GPIO.OUT)
        self._mode = None
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_timer(self.STEP_DELAY * 2, self.step_timer)
        print("Initialized")

    def joy_callback(self, msg: Joy):
    	b = msg.buttons
    	if b[self.LB_BUTTON]:
            new_mode = 'LB'
            print("Pressed LB")
    	elif b[self.RB_BUTTON]:
            new_mode = 'RB'
            print("Pressed RB")
    	elif b[self.DPAD_UP]:
    	    new_mode = 'UP'
    	    print('Pressed DPad Up')
    	elif b[self.DPAD_DOWN]:
    	    new_mode = 'DOWN'
    	    print('Pressed DPad Down')
    	else:
            new_mode = None
            
    	if new_mode != self._mode:
            self._mode = new_mode

    def step_timer(self):
        if self._mode is None:
            return
            
        if self._mode == 'LB':
            d1 = GPIO.LOW #CCW
            d2 = GPIO.HIGH #CW
        elif self._mode == 'RB':
            d1 = GPIO.HIGH
            d2 = GPIO.LOW
        elif self._mode == 'UP':
            d1 = GPIO.LOW
            d2 = GPIO.LOW
        else:
            d1 = GPIO.HIGH
            d2 = GPIO.HIGH
     
        GPIO.output(self.DIR1_PIN, d1)
        GPIO.output(self.DIR2_PIN, d2)
        
        GPIO.output(self.STEP1_PIN, GPIO.HIGH)
        GPIO.output(self.STEP2_PIN, GPIO.HIGH)
        sleep(self.STEP_DELAY)
        GPIO.output(self.STEP1_PIN, GPIO.LOW)
        GPIO.output(self.STEP2_PIN, GPIO.LOW)

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

