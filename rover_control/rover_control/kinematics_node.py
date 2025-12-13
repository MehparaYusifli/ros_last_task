import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32  # Changing to Int32 because hardware requires simple integers
import math
import time

class RoverKinematics(Node):
    def __init__(self):
        # Initialize the ROS 2 Node with the name 'rover_kinematics'
        super().__init__('rover_kinematics')
        
        # --- 1. CONFIGURATION PARAMETERS ---
        # We declare these so we can tune them in the launch file without editing code.
        
        # 'wheelbase': Distance from front axle to rear axle (meters)
        self.declare_parameter('wheelbase', 1.13)
        # 'track_width': Distance from left wheel to right wheel (meters)
        self.declare_parameter('track_width', 0.84)
        # 'max_steer_rad': Limit the servos to 45 degrees (0.785 rad) to prevent breaking them
        self.declare_parameter('max_steer_rad', 0.785)
        # 'timeout_sec': If joystick disconnects for 0.5s, stop the rover
        self.declare_parameter('timeout_sec', 0.5)

        # Load the parameters into variables for easy use
        self.L = self.get_parameter('wheelbase').value
        self.W = self.get_parameter('track_width').value
        self.max_steer = self.get_parameter('max_steer_rad').value
        self.timeout = self.get_parameter('timeout_sec').value

        # --- 2. PUBLISHERS (THE OUTPUT) ---
        # The hardware requires 8 separate topics (one for each motor/servo).
        # We use Int32 because the Arduino/Motor Driver expects raw numbers, not standard ROS messages.
        
        # Front Left Wheel
        self.pub_fl_s = self.create_publisher(Int32, 'wheel_front_left_speed', 10)
        self.pub_fl_a = self.create_publisher(Int32, 'wheel_front_left_angle', 10)
        
        # Front Right Wheel
        self.pub_fr_s = self.create_publisher(Int32, 'wheel_front_right_speed', 10)
        self.pub_fr_a = self.create_publisher(Int32, 'wheel_front_right_angle', 10)
        
        # Rear Left Wheel
        self.pub_rl_s = self.create_publisher(Int32, 'wheel_rear_left_speed', 10)
        self.pub_rl_a = self.create_publisher(Int32, 'wheel_rear_left_angle', 10)
        
        # Rear Right Wheel
        self.pub_rr_s = self.create_publisher(Int32, 'wheel_rear_right_speed', 10)
        self.pub_rr_a = self.create_publisher(Int32, 'wheel_rear_right_angle', 10)

        # --- 3. SUBSCRIBER (THE INPUT) ---
        # We listen to '/cmd_vel'. This topic comes from the joystick node (teleop_twist_joy).
        # It contains linear.x (forward speed) and angular.z (turning speed).
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # --- 4. SAFETY WATCHDOG ---
        # We track exactly when the last command arrived.
        self.last_cmd_time = self.get_clock().now()
        # We create a timer that runs 10 times a second (0.1s) to check connection health.
        self.create_timer(0.1, self.watchdog_loop)

    def cmd_callback(self, msg):
        """
        Triggered whenever the joystick sends a command.
        """
        # 1. Update the timestamp (We are alive!)
        self.last_cmd_time = self.get_clock().now()
        # 2. Process the math immediately
        self.compute_kinematics(msg)

    def watchdog_loop(self):
        """
        Runs constantly to ensure the joystick is still connected.
        If we don't hear from the joystick for 0.5 seconds, we panic and stop.
        """
        # Calculate time passed since last command
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9 # Convert to seconds
        
        if time_diff > self.timeout:
            # SAFETY TRIGGER: Send a "Stop" command (all zeros)
            stop_msg = Twist()
            self.compute_kinematics(stop_msg)
            # (Optional) Log warning: self.get_logger().warn("Watchdog: Stopping rover.")

    def clamp(self, value, limit):
        """
        Simple helper: Ensures 'value' stays between '-limit' and '+limit'.
        Used to protect physical servos from turning too far.
        """
        return max(-limit, min(limit, value))

    def rad_to_servo_int(self, angle_rad):
        """
        CONVERSION HELPER: Radians -> Hardware Integer
        The math logic uses Radians (standard science unit).
        The hardware wants an Integer between 90 and 270.
        
        Mapping logic derived from your test script:
        - Center (0 rad) -> 180
        - Left (+0.78 rad) -> 135
        - Right (-0.78 rad) -> 225
        """
        # Step 1: Convert Radians to Degrees
        degrees = math.degrees(angle_rad)
        
        # Step 2: Apply the offset formula from your script
        # Formula: 180 - degrees
        servo_val = 180 - degrees
        
        # Step 3: Return as Integer
        return int(servo_val)

    def speed_to_pwm_int(self, speed_ms):
        """
        CONVERSION HELPER: Meters/Sec -> PWM Integer
        The math logic uses m/s.
        The hardware wants a PWM signal between -63 and +63.
        """
        # Step 1: Safety clamp (Assume max speed is 1.0 m/s)
        speed_ms = max(-1.0, min(1.0, speed_ms))
        
        # Step 2: Scale to hardware range
        # If speed is 1.0, result is 63. If -1.0, result is -63.
        pwm_val = speed_ms * 63
        
        # Step 3: Return as Integer
        return int(pwm_val)

    def compute_kinematics(self, msg):
        """
        THE MAIN LOGIC FUNCTION
        1. Takes Joystick Input (msg)
        2. Calculates Ackermann Geometry (Radians/Meters)
        3. Converts to Hardware Format (Integers)
        4. Publishes to topics
        """
        vx = msg.linear.x   # Forward Speed
        wz = msg.angular.z  # Turning Speed

        # --- SCENARIO 1: STOP (Deadzone) ---
        if abs(vx) < 0.01 and abs(wz) < 0.01:
            steering = [0.0, 0.0, 0.0, 0.0]
            velocity = [0.0, 0.0, 0.0, 0.0]

        # --- SCENARIO 2: SPOT TURN (Spin in place) ---
        elif abs(vx) < 0.01:
            # Calculate angle to turn wheels tangent to the circle
            angle = math.atan2(self.L, self.W)
            # Calculate speed based on turn rate
            speed = wz * math.hypot(self.L/2, self.W/2)
            
            # Clamp angle for safety
            safe_angle = self.clamp(angle, self.max_steer)
            
            # Set wheels to X-Pattern
            steering = [-safe_angle, safe_angle, -safe_angle, safe_angle]
            # Spin Left wheels fwd, Right wheels back
            velocity = [speed, -speed, speed, -speed]

        # --- SCENARIO 3: ACKERMANN DRIVE (Car-like turn) ---
        else:
            # Protect against division by zero
            wz = wz if abs(wz) > 0.001 else 0.00001
            # Calculate Turning Radius (R)
            R = vx / wz
            
            # Calculate Inner/Outer wheel angles (Geometry Math)
            fl_a = math.atan((self.L/2) / (R - self.W/2))
            fr_a = math.atan((self.L/2) / (R + self.W/2))
            
            # Apply Mechanical Safety Clamps
            fl_a = self.clamp(fl_a, self.max_steer)
            fr_a = self.clamp(fr_a, self.max_steer)
            
            # Calculate Wheel Speeds (Outer wheels move faster)
            fl_s = wz * math.hypot(R - self.W/2, self.L/2)
            fr_s = wz * math.hypot(R + self.W/2, self.L/2)
            
            # Rear wheels steer opposite to front (4-Wheel Steering)
            steering = [fl_a, fr_a, -fl_a, -fr_a]
            velocity = [fl_s, fr_s, fl_s, fr_s]

        # --- HARDWARE TRANSLATION (The Bridge) ---
        # Now we convert the "Smart Math" (Radians) into "Dumb Numbers" (Integers)
        
        # 1. Convert Angles (Rad -> 90-270 range)
        fl_a_int = self.rad_to_servo_int(steering[0])
        fr_a_int = self.rad_to_servo_int(steering[1])
        rl_a_int = self.rad_to_servo_int(steering[2])
        rr_a_int = self.rad_to_servo_int(steering[3])

        # 2. Convert Speeds (m/s -> -63 to 63 range)
        fl_s_int = self.speed_to_pwm_int(velocity[0])
        fr_s_int = self.speed_to_pwm_int(velocity[1])
        rl_s_int = self.speed_to_pwm_int(velocity[2])
        rr_s_int = self.speed_to_pwm_int(velocity[3])

        # --- PUBLISH TO MOTORS ---
        # Send the calculated integers to the specific topics the hardware listens to.
        
        self.pub_fl_s.publish(Int32(data=fl_s_int))
        self.pub_fl_a.publish(Int32(data=fl_a_int))
        
        self.pub_fr_s.publish(Int32(data=fr_s_int))
        self.pub_fr_a.publish(Int32(data=fr_a_int))
        
        self.pub_rl_s.publish(Int32(data=rl_s_int))
        self.pub_rl_a.publish(Int32(data=rl_a_int))
        
        self.pub_rr_s.publish(Int32(data=rr_s_int))
        self.pub_rr_a.publish(Int32(data=rr_a_int))

def main():
    # Start ROS
    rclpy.init()
    # Keep the node running
    rclpy.spin(RoverKinematics())
    # Stop ROS
    rclpy.shutdown()