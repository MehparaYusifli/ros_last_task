#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import math

class RoverKinematics(Node):
    def __init__(self):
        super().__init__('rover_kinematics')

        # --- 1. CONFIGURATION ---
        self.declare_parameter('wheelbase', 1.13)
        self.declare_parameter('track_width', 0.84)
        self.declare_parameter('max_steer_rad', 0.785) # 45 degrees
        self.declare_parameter('timeout_sec', 0.5)

        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.max_steer = self.get_parameter('max_steer_rad').value
        self.timeout = self.get_parameter('timeout_sec').value

        # --- 2. PUBLISHERS (8 Topics - Int32 for Hardware) ---
        self.pub_fl_s = self.create_publisher(Int32, 'wheel_front_left_speed', 10)
        self.pub_fl_a = self.create_publisher(Int32, 'wheel_front_left_angle', 10)
        self.pub_fr_s = self.create_publisher(Int32, 'wheel_front_right_speed', 10)
        self.pub_fr_a = self.create_publisher(Int32, 'wheel_front_right_angle', 10)
        self.pub_rl_s = self.create_publisher(Int32, 'wheel_rear_left_speed', 10)
        self.pub_rl_a = self.create_publisher(Int32, 'wheel_rear_left_angle', 10)
        self.pub_rr_s = self.create_publisher(Int32, 'wheel_rear_right_speed', 10)
        self.pub_rr_a = self.create_publisher(Int32, 'wheel_rear_right_angle', 10)

        # --- 3. SUBSCRIBER ---
        self.subscription_ = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # --- 4. SAFETY WATCHDOG ---
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.watchdog_loop)

    def joy_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        # 1. READ INPUTS
        # Axis 1 = Up/Down (Speed)
        # Axis 0 = Left/Right (Steer) - Standard Xbox/Logitech mapping
        try:
            speed_axis = msg.axes[1] 
            steer_axis = msg.axes[0]
        except IndexError:
            return

        # 2. CALCULATE TARGETS
        # FIX #1: Use 63, not 255 (Matches your hardware test script)
        velocity = speed_axis * 63.0  
        
        # Steering: Map -1.0..1.0 to -max_steer..max_steer
        delta = steer_axis * self.max_steer

        # 3. ACKERMANN GEOMETRY (4-WHEEL STEERING VERSION)
        if abs(delta) < 0.01:
            # Straight
            fl_angle = 0.0
            fr_angle = 0.0
            rl_angle = 0.0
            rr_angle = 0.0
        else:
            R = self.L / math.tan(abs(delta))

            if delta > 0: # LEFT TURN
                # Front
                fl_angle = math.atan(self.L / (R - self.W / 2))
                fr_angle = math.atan(self.L / (R + self.W / 2))
                # Rear (Opposite to front for tighter turn)
                rl_angle = -math.atan(self.L / (R - self.W / 2))
                rr_angle = -math.atan(self.L / (R + self.W / 2))
            else: # RIGHT TURN
                # Front
                fl_angle = -math.atan(self.L / (R + self.W / 2))
                fr_angle = -math.atan(self.L / (R - self.W / 2))
                # Rear (Opposite to front)
                rl_angle = math.atan(self.L / (R + self.W / 2))
                rr_angle = math.atan(self.L / (R - self.W / 2))

        # 4. PUBLISH COMMANDS
        self.publish_commands(velocity, fl_angle, fr_angle, rl_angle, rr_angle)

    def publish_commands(self, velocity, fl_rad, fr_rad, rl_rad, rr_rad):
        # FIX #2: Correct Hardware Mapping (180 - degrees)
        # This matches the 'joy_wheel_converter' logic you proved works.
        
        def rad_to_servo(rad):
            deg = math.degrees(rad)
            return int(180 - deg) # The critical hardware formula

        fl_int = rad_to_servo(fl_rad)
        fr_int = rad_to_servo(fr_rad)
        rl_int = rad_to_servo(rl_rad)
        rr_int = rad_to_servo(rr_rad)
        
        vel_int = int(velocity)

        # Create Message
        s_msg = Int32()
        s_msg.data = vel_int
        
        # Front Left
        self.pub_fl_s.publish(s_msg)
        self.pub_fl_a.publish(Int32(data=fl_int))
        
        # Front Right
        self.pub_fr_s.publish(s_msg)
        self.pub_fr_a.publish(Int32(data=fr_int))
        
        # Rear Left
        self.pub_rl_s.publish(s_msg)
        self.pub_rl_a.publish(Int32(data=rl_int))
        
        # Rear Right
        self.pub_rr_s.publish(s_msg)
        self.pub_rr_a.publish(Int32(data=rr_int))

    def watchdog_loop(self):
        time_diff = self.get_clock().now() - self.last_cmd_time
        
        if time_diff.nanoseconds / 1e9 > self.timeout:
            # STOP ROVER
            zero_msg = Int32()
            zero_msg.data = 0
            
            # Reset speed to 0, keep angles as is (or reset angles to 180 if preferred)
            self.pub_fl_s.publish(zero_msg)
            self.pub_fr_s.publish(zero_msg)
            self.pub_rl_s.publish(zero_msg)
            self.pub_rr_s.publish(zero_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoverKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()