import time
import launch
import launch.actions
import launch.events
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, SetEnvironmentVariable
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("dvl_a50"), "config", "dvl_a50.yml"
    )
    node = LifecycleNode(
        namespace="sensors",
        package="dvl_a50",
        executable="dvl_a50_node",
        name="dvl_a50",
        parameters=[config],
    )

    # Not supported in xml or yaml launch files!
    # Lifecycle transition events from https://answers.ros.org/question/304370/ros2-launch-how-to-correctly-create-a-lifecycle-launch-file/
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # When the node reaches the 'active' state, log a message
    on_activated = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="active",
            entities=[
                LogInfo(msg=f"'sonar_blueview' reached the 'ACTIVE' state"),
            ],
        )
    )

    ld.add_action(configure_event)
    ld.add_action(activate_event)
    ld.add_action(node)
    ld.add_action(on_activated)

    return ld
