from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'use_sim_time': ('false', bool),
        'follow_distance': ('0.35', float),
        'max_front_length': ('0.50', float),
        'forward_speed': ('0.10', float),
        'lateral_speed': ('0.10', float),
        'distance_gain': ('0.80', float),
        'angle_gain': ('1.20', float),
        'max_angular_speed': ('0.30', float),
        'detection_limit': ('1.20', float),
        'back_right_limit_factor': ('1.50', float),
    }

    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, (value, _) in defaults.items()
    ]

    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=kind)
        for name, (_, kind) in defaults.items()
    }

    return LaunchDescription(arguments + [
        Node(
            package='my_robot_control',
            executable='my_robot_wallfollower_holonomic_improved_exec',
            name='improved_holonomic_wall_follower',
            output='screen',
            parameters=[parameters],
        ),
    ])