"""LiDAR version, container 2: the map builder and the car simulator.

The mapper builds the map from the phone's LiDAR stream, finds the posts and the wall,
plans the slalom and publishes it; the simulator drives it and sends the drive back;
the mapper draws the car on the LiDAR map. The explorer drives the same track WITHOUT
knowing it (it only gets the map as its "world" and sees it through a virtual sensor).
The launch stops when all three nodes have finished.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("gap_cm", default_value="10.0"),
        DeclareLaunchArgument("corridor_width_cm", default_value="100.0"),
        DeclareLaunchArgument("max_steer_deg", default_value="30.0"),
        DeclareLaunchArgument("wind_cm_s", default_value="300.0",
                              description="wind along the corridor (in the face there, in the back back); 0 = calm"),
    ]
    mapper = Node(
        package="minicar_processor", executable="lidar_mapper_node", output="screen",
        parameters=[{"output_dir": "/results/lidar", "gap_cm": LaunchConfiguration("gap_cm"),
                     "corridor_width_cm": LaunchConfiguration("corridor_width_cm")}],
    )
    simulator = Node(
        package="minicar_simulator", executable="simulator_node", output="screen",
        parameters=[{"output_dir": "/results/lidar", "wait_for_limits": True,
                     "post_radius_cm": 3.5, "max_steer_deg": LaunchConfiguration("max_steer_deg"),
                     "wind_cm_s": LaunchConfiguration("wind_cm_s")}],
    )
    explorer = Node(
        package="minicar_simulator", executable="explorer_node", output="screen",
        parameters=[{"output_dir": "/results/lidar"}],
    )
    # stop the launch when all three nodes have finished
    finished = set()

    def one_finished(event, context):
        finished.add(event.action.name)
        if len(finished) == 3:
            return [EmitEvent(event=Shutdown())]
        return []

    stop_handlers = [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=one_finished))
                     for n in (mapper, simulator, explorer)]
    return LaunchDescription(args + [mapper, simulator, explorer] + stop_handlers)
