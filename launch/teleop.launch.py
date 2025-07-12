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
        # Teleop for joystick
        Node(
            package='turtlebot3_joy_teleop', executable='teleop_node', name='joy_teleop',
            parameters=[
                {'joy_topic': 'joy'},
                {'cmd_vel_topic': 'cmd_vel'},
                {'use_timestamp': True},
                {'enable_button': 4},
                {'axis_linear': 1},
                {'axis_angular': 0},
                {'scale_linear': 1.0},
                {'scale_angular': 1.5},
                {'require_enable': True}
            ]
        ),
        # Teleop for spacenav
        Node(
            package='turtlebot3_joy_teleop', executable='teleop_node', name='spnav_teleop',
            parameters=[
                {'joy_topic': 'spacenav/joy'},
                {'cmd_vel_topic': 'cmd_vel'},
                {'use_timestamp': True},
                {'enable_button': 0},
                {'axis_linear': 0},
                {'axis_angular': 5},
                {'scale_linear': 1.0},
                {'scale_angular': 1.5},
                {'require_enable': True}
            ]
        ),
    ])
