#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mprls_imu_node.msg import MPRLSPressures
import RPi.GPIO as GPIO
from time import sleep, time
 
class CombinedControlNode(Node):
    def __init__(self):
        super().__init__('combined_control_node')
 
        # ── MOTOR CONFIG ───────────────────────────
        self.DIR1_PIN    = 20
        self.STEP1_PIN   = 21
        self.DIR2_PIN    = 10
        self.STEP2_PIN   = 11
        self.MOTOR_STOP_DURATION = 1.0
        self.STEP_DELAY   = 0.001
 
        # ── AUTONOMOUS CYCLE CONFIG ────────────────
        self.START_BUTTON   = 7
        self._auto_active   = False
        self._auto_phase    = 0
        self._phase_start   = 0
        self.SOL_DELAY      = 0.5
        self.AUTO_PHASES    = 6
 
        # ── PUMP CONFIG ────────────────────────────
        self.SOL_PINS = {
            'Y': 14, 'A': 17, 'X': 22, 'B': 24,
        }
        self.PUMP_PINS = {
            'Y': 15, 'A': 18, 'X': 23, 'B': 27,
        }
        self.SOL_DELAY = 0.5
 
        # ── PRESSURE THRESHOLDS ────────────────────
        self.MAX_PRESSURE = 1050
        self.MIN_PRESSURE = 900
        self.pressure_sensor_1 = 0.0  # back balloons
        self.pressure_sensor_2 = 0.0  # front balloons
 
        # ── JOYSTICK MAPPING ──────────────────────
        self.LB_BUTTON  = 4
        self.RB_BUTTON  = 5
        self.DPAD_UP    = 13
        self.DPAD_DOWN  = 14
        self.BUTTONS = {'A':0,'B':1,'X':2,'Y':3}
 
        # ── GPIO SETUP ─────────────────────────────
        GPIO.setmode(GPIO.BCM)
        for p in (self.DIR1_PIN, self.STEP1_PIN, self.DIR2_PIN, self.STEP2_PIN):
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)
        for p in list(self.SOL_PINS.values()) + list(self.PUMP_PINS.values()):
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)
 
        # ── STATE TRACKING ──────────────────────────
        self._motor_mode = None
        self._motor_btn = None
        self._motor_time = 0
        self._motor_disabled = False
        self._pump_key = None
        self._pump_time = 0
 
        # ── ROS2 SETUP ─────────────────────────────
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(MPRLSPressures, '/mprls_pressures', self.pressure_callback, 10)
        self.create_timer(self.STEP_DELAY*2, self.step_timer)
        self.create_timer(0.1, self.pump_timer)
        self.create_timer(0.1, self.auto_cycle)
        self.get_logger().info('CombinedControlNode ready')
 
    def pressure_callback(self, msg):
        self.pressure_sensor_1 = msg.pressure_sensor_1
        self.pressure_sensor_2 = msg.pressure_sensor_2
 
    def joy_callback(self, msg: Joy):
        b = msg.buttons
        now = time()
 
        if b[self.START_BUTTON]:
            if not self._auto_active:
                self._auto_active = True
                self._motor_disabled = False
                self._auto_phase = 0
                self._phase_start = now
                self.get_logger().info('Autonomous cycle START')
        else:
            if self._auto_active:
                self._auto_active = False
                self._clear_pumps()
                self._pump_key = None
                self._motor_mode = None
                self.get_logger().info('Autonomous cycle STOP')
 
        new_mode = None
        btn = None
        if b[self.LB_BUTTON]: new_mode, btn = 'LB', self.LB_BUTTON
        elif b[self.RB_BUTTON]: new_mode, btn = 'RB', self.RB_BUTTON
        elif b[self.DPAD_UP]: new_mode, btn = 'UP', self.DPAD_UP
        elif b[self.DPAD_DOWN]: new_mode, btn = 'DOWN', self.DPAD_DOWN
 
        if not self._auto_active:
            if btn is not None:
                if not self._motor_disabled and btn != self._motor_btn:
                    self._motor_btn  = btn
                    self._motor_time = now
                    self._motor_mode = new_mode
                    self.get_logger().info(f'Motor mode: {new_mode}')
                elif self._motor_btn == btn and not self._motor_disabled:
                    if now - self._motor_time >= self.MOTOR_STOP_DURATION * 3:
                        self.get_logger().warn('Motor disabled until release')
                        self._motor_disabled = True
                        self._motor_mode = None
            else:
                self._motor_btn = None
                self._motor_time = 0
                self._motor_mode = None
                self._motor_disabled = False
 
        if not self._auto_active:
            for key, idx in self.BUTTONS.items():
                if b[idx] and key != self._pump_key:
                    self._pump_key  = key
                    self._pump_time = now
                    self.get_logger().info(f'Pump action: {key}')
                    break
                if not b[idx] and self._pump_key == key:
                    self._clear_pumps()
                    self._pump_key = None
                    self._pump_time = 0
 
    def auto_cycle(self):
        if not self._auto_active:
            return
        now = time()
        ph = self._auto_phase
        elapsed = now - self._phase_start
 
        if ph in (0, 1, 3, 4):
            if ph == 0:
                keys = ['Y']; pressure = self.pressure_sensor_2; inflating = True
            elif ph == 1:
                keys = ['B']; pressure = self.pressure_sensor_1; inflating = False
            elif ph == 3:
                keys = ['X']; pressure = self.pressure_sensor_1; inflating = True
            else:
                keys = ['A']; pressure = self.pressure_sensor_2; inflating = False
 
            if elapsed < self.SOL_DELAY:
                for k in keys:
                    GPIO.output(self.SOL_PINS[k], GPIO.HIGH)
                    GPIO.output(self.PUMP_PINS[k], GPIO.LOW)
            else:
                if (inflating and pressure < self.MAX_PRESSURE) or (not inflating and pressure > self.MIN_PRESSURE):
                    for k in keys:
                        GPIO.output(self.SOL_PINS[k], GPIO.HIGH)
                        GPIO.output(self.PUMP_PINS[k], GPIO.HIGH)
                else:
                    for k in keys:
                        GPIO.output(self.SOL_PINS[k], GPIO.LOW)
                        GPIO.output(self.PUMP_PINS[k], GPIO.LOW)
                    self._auto_phase = (ph + 1) % self.AUTO_PHASES
                    self._phase_start = now
 
        elif ph == 2:
            if elapsed < self.MOTOR_STOP_DURATION:
                self._motor_mode = 'DOWN'
            else:
                self._motor_mode = None
                self._auto_phase = (ph + 1) % self.AUTO_PHASES
                self._phase_start = now
 
        elif ph == 5:
            if elapsed < self.MOTOR_STOP_DURATION:
                self._motor_mode = 'UP'
            else:
                self._motor_mode = None
                self._auto_phase = (ph + 1) % self.AUTO_PHASES
                self._phase_start = now
 
    def step_timer(self):
        if self._auto_active and self._auto_phase not in (2, 5):
            return
        mode = self._motor_mode
        if self._auto_active:
            if self._auto_phase == 5:
                mode = 'UP'
            elif self._auto_phase == 2:
                mode = 'DOWN'
        if not mode:
            return
        if mode == 'LB': d1, d2 = GPIO.LOW, GPIO.HIGH
        elif mode == 'RB': d1, d2 = GPIO.HIGH, GPIO.LOW
        elif mode == 'UP': d1, d2 = GPIO.LOW, GPIO.LOW
        else: d1, d2 = GPIO.HIGH, GPIO.HIGH
        GPIO.output(self.DIR1_PIN, d1)
        GPIO.output(self.DIR2_PIN, d2)
        GPIO.output(self.STEP1_PIN, GPIO.HIGH)
        GPIO.output(self.STEP2_PIN, GPIO.HIGH)
        sleep(self.STEP_DELAY)
        GPIO.output(self.STEP1_PIN, GPIO.LOW)
        GPIO.output(self.STEP2_PIN, GPIO.LOW)
 
    def pump_timer(self):
        if self._auto_active:
            return
        key = self._pump_key
        if not key:
            return
        now = time()
        elapsed = now - self._pump_time
        sol = self.SOL_PINS[key]
        pump = self.PUMP_PINS[key]
        if elapsed < self.SOL_DELAY:
            GPIO.output(sol, GPIO.HIGH)
            GPIO.output(pump, GPIO.LOW)
            return
 
        if key in ['X', 'B']:  # back
            pressure = self.pressure_sensor_1
        else:  # front
            pressure = self.pressure_sensor_2
 
        inflating = key in ['X', 'Y']
        if (inflating and pressure < self.MAX_PRESSURE) or (not inflating and pressure > self.MIN_PRESSURE):
            GPIO.output(sol, GPIO.HIGH)
            GPIO.output(pump, GPIO.HIGH)
        else:
            GPIO.output(sol, GPIO.LOW)
            GPIO.output(pump, GPIO.LOW)
            self.get_logger().warn(f'Pump actions "{key}" stopped due to pressure threshold: {pressure:.2f} hPA')
            self._clear_pumps()
            self._pump_key = None
            self._pump_time = 0
 
    def _clear_pumps(self):
        for p in list(self.SOL_PINS.values()) + list(self.PUMP_PINS.values()):
            GPIO.output(p, GPIO.LOW)
 
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()
 
def main(args=None):
    rclpy.init()
    node = CombinedControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
