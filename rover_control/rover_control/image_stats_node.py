import rclpy                                        
from rclpy.node import Node                         
from rclpy.qos import QoSProfile, ReliabilityPolicy # Import Quality of Service settings for network tuning
from sensor_msgs.msg import Image                   # Standard ROS message for video frames
from std_msgs.msg import String                     # Standard ROS message for text/JSON data
import time                                         # Required for calculating Frames Per Second (FPS)
import json                                         # Required to format data so the Website (JavaScript) can read it

class ImageStats(Node):
    def __init__(self):
        # Initialize the node with the name 'image_stats'
        super().__init__('image_stats')
        
        # --- 1. QoS CONFIGURATION (CRITICAL ARCHITECTURE CHOICE) ---
        # Standard ROS communication uses "Reliable" policy (similar to TCP).
        # This means if a packet is dropped, the network pauses to request a resend.
        # For live video driving a robot, this is bad because it causes "buffering lag".
        
        # We switch to "Best Effort" (similar to UDP). 
        # Logic: If a video frame is corrupted or lost, ignore it and display the next one.
        # Result: The operator always sees the CURRENT reality, even if the quality drops.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribe to the raw camera feed using our custom low-latency QoS profile
        self.create_subscription(Image, '/camera/image_raw', self.calculate, qos)
        
        # Publisher for the Dashboard
        # We publish a simple String because we are sending a JSON object.
        self.pub_telemetry = self.create_publisher(String, '/camera/stats', 10)
        
        # Initialize counters for calculation
        self.frames = 0       # How many frames received in the current second
        self.bytes = 0        # How many bytes of data received in the current second
        self.start = time.time() # The timestamp when we started counting

    def calculate(self, msg):
        """
        Callback function executed EVERY TIME a new video frame arrives.
        msg: The image data containing pixels, height, width, etc.
        """
        # 1. Update Counters
        self.frames += 1            # Increment frame count
        self.bytes += len(msg.data) # Add the size of the current image to total bandwidth
        
        # 2. Check time elapsed
        # We want to report statistics exactly once per second (1.0 Hz)
        elapsed = time.time() - self.start
        
        if elapsed >= 1.0:
            # --- 3. PERFORM CALCULATIONS ---
            
            # Calculate Frequency: Frames / Time (should be around 30.0)
            fps = self.frames / elapsed
            
            # Calculate Bandwidth: 
            # (Bytes * 8) = Bits
            # / 1000 = Kilobits
            # / elapsed = Kilobits per second (Kbps)
            kbps = (self.bytes * 8) / 1000 / elapsed
            
            # --- 4. FORMAT DATA FOR OPENMCT (TASK 3) ---
            # OpenMCT is a web dashboard. It understands JSON (JavaScript Object Notation).
            # We create a Python dictionary to structure our data.
            telemetry = {
                "timestamp": time.time(),                # Current time for the x-axis of the graph
                "id": "main_cam",                        # Unique ID for the data source
                "fps": round(fps, 2),                    # Round FPS to 2 decimals for readability
                "bandwidth_kbps": int(kbps),             # Bandwidth as an integer
                "resolution": f"{msg.width}x{msg.height}" # Video resolution (e.g. "640x480")
            }
            
            # Serialize: Convert the Python dictionary into a JSON String
            msg_out = String()
            msg_out.data = json.dumps(telemetry)
            
            # Publish the JSON string to the '/camera/stats' topic
            self.pub_telemetry.publish(msg_out)
            
            # --- 5. RESET ---
            # Clear the counters to start measuring the next second
            self.frames = 0
            self.bytes = 0
            self.start = time.time()

def main():
    # Start the ROS 2 system
    rclpy.init()
    # Keep the node running and listening for images
    rclpy.spin(ImageStats())
    # Clean up and shut down
    rclpy.shutdown()