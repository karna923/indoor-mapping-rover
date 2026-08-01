import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_rover_description = get_package_share_directory('rover_description')
    slam_params_file = os.path.join(pkg_rover_description, 'config', 'slam_toolbox_params.yaml')

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}]
    )

    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == slam_toolbox_node,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        slam_toolbox_node,
        configure_on_start,
        activate_on_configure,
    ])
