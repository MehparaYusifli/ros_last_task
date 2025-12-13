from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. DECLARE ARGUMENTS (Configuration) ---
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Set to false to run without camera (e.g., motor testing)'
    )
    
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop', default_value='true',
        description='Set to false to run without joystick (e.g., autonomy mode)'
    )

    # Capture the values
    use_camera = LaunchConfiguration('use_camera')
    use_teleop = LaunchConfiguration('use_teleop')

    return LaunchDescription([
        use_camera_arg,
        use_teleop_arg,

        # --- 2. INPUT LAYER (Joystick) ---
        # Reads the physical controller
        Node(
            condition=IfCondition(use_teleop),
            package='joy', executable='joy_node'
        ),
        # Converts buttons to Twist messages
        Node(
            condition=IfCondition(use_teleop),
            package='teleop_twist_joy', executable='teleop_node',
            parameters=[{
                'axis_linear.x': 1, 
                'axis_angular.yaw': 3, 
                'scale_linear.x': 1.0, 
                'scale_angular.yaw': 1.0,
                'enable_button': -1 # Always active (Deadman switch recommended for V2)
            }]
        ),

        # --- 3. CONTROL LAYER (The Brain) ---
        # Runs our Kinematics logic
        Node(
            package='rover_control', executable='kinematics',
            # PLATINUM FEATURE: Override dimensions here if robot changes!
            parameters=[{'wheelbase': 1.13, 'track_width': 0.84, 'max_steer_rad': 0.785}]
        ),

        # --- 4. VISION LAYER (The Eyes) ---
        # Hardware Driver (Standard USB Cam)
        Node(
            condition=IfCondition(use_camera),
            package='usb_cam', executable='usb_cam_node_exe',
            parameters=[{'video_device': '/dev/video0', 'framerate': 30.0}],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),
        # Web Video Server (Industry Standard C++ Streamer)
        Node(
            condition=IfCondition(use_camera),
            package='web_video_server', executable='web_video_server',
            parameters=[{'port': 8080}]
        ),
        # Telemetry Node (Our custom logic)
        Node(
            condition=IfCondition(use_camera),
            package='rover_control', executable='image_stats'
        ),

        # --- 5. CONNECTIVITY LAYER (The Bridge) ---
        # Allows OpenMCT to talk to ROS via JSON/Websockets
        Node(package='rosbridge_server', executable='rosbridge_websocket')
    ])