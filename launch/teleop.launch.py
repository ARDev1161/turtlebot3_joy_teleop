from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
        Node(
            package='spacenav', executable='spacenav_node', name='spacenav_node'
        ),
        # Teleop for joystick only
        Node(
            package='turtlebot3_joy_teleop', executable='teleop_node', name='joy_teleop',
            parameters=[
                {'cmd_vel_topic': 'cmd_vel'},
                {'use_timestamp': True},
                {'scale_linear': 1.0},
                {'scale_angular': 1.5},
                {'require_enable': True}
            ]
        ),
    ])
