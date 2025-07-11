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
        Node(
            package='turtlebot3_joy_teleop', executable='teleop_node', name='teleop_node',
            parameters=[
                {'joy_topic': 'joy'},
                {'spacenav_topic': 'spacenav/joy'},
                {'cmd_vel_topic': 'cmd_vel'},
                {'use_timestamp': True},
                {'enable_button': 5},
                {'axis_linear': 1},
                {'axis_angular': 0},
                {'scale_linear': 1.0},
                {'scale_angular': 1.5},
                {'require_enable': False}
            ]
        ),
    ])
