import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    """Generate launch description with multiple components."""
    # for joy
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),  # device of joy

        ComposableNodeContainer(
            name='main_arm_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='joy',
                    plugin='joy::Joy',
                    name='joy_node',
                    parameters=[{
                        'dev': joy_dev,
                        'deadzone': 0.3,
                        'autorepeat_rate': 20.0,
                    }]),
                ComposableNode(
                    package='catch23_robot_controller',
                    plugin='arm_controller::C620Test',
                    name='c620_test'),
            ],
            output='both',
        )
    ])
