import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import yaml

from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.event_handlers import OnProcessStart
from launch.event_handlers.on_shutdown import OnShutdown
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import matches_node_name


def gen_serial_settings():
    share_dir_kondo = get_package_share_directory('kondo_drivers')
    params_declare = DeclareLaunchArgument(
        'serial_params_file',
        default_value=os.path.join(share_dir_kondo, 'params', 'params.yaml'),
        description='File path to the ROS2 parameters file to use'
    )

    node_name = "serial_driver_node"
    bridge_node = LifecycleNode(
        package='serial_driver',
        executable='serial_bridge',
        name=node_name,
        namespace=TextSubstitution(text=''),
        parameters=[LaunchConfiguration('serial_params_file')],
        output='screen',
    )

    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=bridge_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )

    shutdown_event_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name(node_name),
                        transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN,
                    )
                )
            ]
        )
    )
    return params_declare, bridge_node, configure_event_handler, activate_event_handler, shutdown_event_handler


def generate_launch_description():
    """Generate launch description with multiple components."""
    # for joy
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    params_declare, bridge_node, configure_event_handler, activate_event_handler, shutdown_event_handler = gen_serial_settings()

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),  # device of joy
        params_declare,
        bridge_node,
        configure_event_handler,
        activate_event_handler,
        shutdown_event_handler,
        ComposableNodeContainer(
            name='main_arm_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package="kondo_drivers",
                    plugin="kondo_drivers::KondoB3mDriverNode",
                    name="kondo_b3m_driver_component",
                ),
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
                    plugin='arm_controller::ArmControllerNode',
                    name='main_arm_controller_component'),
                ComposableNode(
                    package='catch23_robot_controller',
                    plugin='arm_controller::StatePublisherNode',
                    name='state_publisher_component'),
            ],
            output='both',
        )
    ])
