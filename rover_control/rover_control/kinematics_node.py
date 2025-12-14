import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
import math
import time

class RoverKinematics(Node):
    def __init__(self):
        super().__init__('rover_kinematics')
        
        # --- 1. CONFIGURATION ---
        self.declare_parameter('wheelbase', 1.13)
        self.declare_parameter('track_width', 0.84)
        self.declare_parameter('max_steer_rad', 0.785)
        self.declare_parameter('timeout_sec', 0.5)

        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.max_steer = self.get_parameter('max_steer_rad').value
        self.timeout = self.get_parameter('timeout_sec').value

        # --- 2. PUBLISHERS ---
        self.pub_fl_s = self.create_publisher(Int32, 'wheel_front_left_speed', 10)
        self.pub_fl_a = self.create_publisher(Int32, 'wheel_front_left_angle', 10)
        self.pub_fr_s = self.create_publisher(Int32, 'wheel_front_right_speed', 10)
        self.pub_fr_a = self.create_publisher(Int32, 'wheel_front_right_angle', 10)
        self.pub_rl_s = self.create_publisher(Int32, 'wheel_rear_left_speed', 10)
        self.pub_rl_a = self.create_publisher(Int32, 'wheel_rear_left_angle', 10)
        self.pub_rr_s = self.create_publisher(Int32, 'wheel_rear_right_speed', 10)
        self.pub_rr_a = self.create_publisher(Int32, 'wheel_rear_right_angle', 10)

        # --- 3. SUBSCRIBER ---
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Watchdog
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_loop)

    def joy_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        try:
            # Axis 1 (Left Stick Up/Down) -> Forward/Back
            vx = msg.axes[1] 
            # Axis 0 (Left Stick Left/Right) -> Turn
            wz = msg.axes[0]
        except IndexError:
            return
        self.compute_kinematics(vx, wz)

    def watchdog_loop(self):
        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_diff > self.timeout:
            self.compute_kinematics(0.0, 0.0)

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def rad_to_servo_int(self, angle_rad):
        degrees = math.degrees(angle_rad)
        return int(180 - degrees)

    def speed_to_pwm_int(self, speed_float):
        speed_float = max(-1.0, min(1.0, speed_float))
        return int(speed_float * 63)

    def compute_kinematics(self, vx, wz):
        # --- 1. STOP ---
        if abs(vx) < 0.01 and abs(wz) < 0.01:
            steering = [0.0, 0.0, 0.0, 0.0]
            velocity = [0.0, 0.0, 0.0, 0.0]

        # --- 2. SPOT TURN ---
        elif abs(vx) < 0.01:
            angle = math.atan2(self.L, self.W)
            speed = wz 
            safe_angle = self.clamp(angle, self.max_steer)
            steering = [-safe_angle, safe_angle, -safe_angle, safe_angle]
            velocity = [speed, -speed, speed, -speed]

        # --- 3. ACKERMANN DRIVE ---
        else:
            wz = wz if abs(wz) > 0.001 else 0.00001
            R = vx / wz
            
            # Angles
            fl_a = math.atan((self.L/2) / (R - self.W/2))
            fr_a = math.atan((self.L/2) / (R + self.W/2))
            
            fl_a = self.clamp(fl_a, self.max_steer)
            fr_a = self.clamp(fr_a, self.max_steer)
            
            # Differential Speeds (Outer wheels faster)
            dist_left = math.hypot(R - self.W/2, self.L/2)
            dist_right = math.hypot(R + self.W/2, self.L/2)
            
            # Normalize direction
            fl_s = vx * (dist_left / abs(R))
            fr_s = vx * (dist_right / abs(R))
            rl_s = vx * (dist_left / abs(R))
            rr_s = vx * (dist_right / abs(R))
            
            steering = [fl_a, fr_a, -fl_a, -fr_a]
            velocity = [fl_s, fr_s, fl_s, fr_s]

        # --- HARDWARE OUTPUT ---
        fl_a_int = self.rad_to_servo_int(steering[0])
        fr_a_int = self.rad_to_servo_int(steering[1])
        rl_a_int = self.rad_to_servo_int(steering[2])
        rr_a_int = self.rad_to_servo_int(steering[3])

        fl_s_int = self.speed_to_pwm_int(velocity[0])
        fr_s_int = self.speed_to_pwm_int(velocity[1])
        rl_s_int = self.speed_to_pwm_int(velocity[2])
        rr_s_int = self.speed_to_pwm_int(velocity[3])

        self.pub_fl_s.publish(Int32(data=fl_s_int))
        self.pub_fl_a.publish(Int32(data=fl_a_int))
        self.pub_fr_s.publish(Int32(data=fr_s_int))
        self.pub_fr_a.publish(Int32(data=fr_a_int))
        self.pub_rl_s.publish(Int32(data=rl_s_int))
        self.pub_rl_a.publish(Int32(data=rl_a_int))
        self.pub_rr_s.publish(Int32(data=rr_s_int))
        self.pub_rr_a.publish(Int32(data=rr_a_int))

def main():
    rclpy.init()
    rclpy.spin(RoverKinematics())
    rclpy.shutdown()