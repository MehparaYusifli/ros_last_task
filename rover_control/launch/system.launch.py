from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Set to false to run without camera'
    )
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop', default_value='true',
        description='Set to false to run without joystick'
    )
    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge', default_value='true',
        description='Set to false to run without rosbridge'
    )

    use_camera = LaunchConfiguration('use_camera')
    use_teleop = LaunchConfiguration('use_teleop')
    use_bridge = LaunchConfiguration('use_bridge')

    return LaunchDescription([
        use_camera_arg,
        use_teleop_arg,
        use_bridge_arg,

        # --- INPUT (JOYSTICK) ---
        # Includes DEADZONE and AUTOREPEAT settings
        Node(
            condition=IfCondition(use_teleop),
            package='joy', 
            executable='joy_node',
            name='joy_driver',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,             # Fixes drift
                'autorepeat_rate': 20.0,      # Fixes watchdog timeout
            }]
        ),

        # --- CONTROL (KINEMATICS) ---
        Node(
            package='rover_control', 
            executable='kinematics',
            parameters=[{'wheelbase': 1.13, 'track_width': 0.84, 'max_steer_rad': 0.785}]
        ),

        # --- VISION ---
        Node(
            condition=IfCondition(use_camera),
            package='usb_cam', executable='usb_cam_node_exe',
            parameters=[{'video_device': '/dev/video0', 'framerate': 30.0}],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),
        Node(
            condition=IfCondition(use_camera),
            package='web_video_server', executable='web_video_server',
            parameters=[{'port': 8080}]
        ),
        Node(
            condition=IfCondition(use_camera),
            package='rover_control', executable='image_stats'
        ),

        # --- BRIDGE ---
        Node(
            condition=IfCondition(use_bridge),
            package='rosbridge_server', executable='rosbridge_websocket'
        )
    ])