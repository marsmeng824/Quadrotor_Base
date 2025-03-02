import launch
import launch_ros.actions
from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited, ShutdownProcess

def generate_launch_description():
    # switch on takeoff node
    takeoff_node = launch_ros.actions.Node(
        package='f450_simulation',  #  package where the node locates
        executable='takeoff',  # takeoff procedure
        name='takeoff'
    )

    # switch on circle node
    circle_node = launch_ros.actions.Node(
        package='f450_simulation',  #  package where the node locates
        executable='trajectory',  # circle procedure
        name='circle'
    )

    # **1. after 30 seconds stop takeoff_node**
    stop_takeoff = TimerAction(
        period=30.0,
        actions=[
            EmitEvent(event=ShutdownProcess(process_matcher=lambda p: p is takeoff_node))
        ],
    )

    # **2. after exit takeoff_node，switch on circle_node**
    start_circle = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=takeoff_node,
            on_exit=[circle_node]
        )
    )

    return launch.LaunchDescription([
        takeoff_node,
        stop_takeoff,  
        start_circle,  
    ])
