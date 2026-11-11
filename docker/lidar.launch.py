"""LiDAR version, container 2: the map builder and the car simulator.

The mapper builds the map from the phone's LiDAR stream, finds the posts and the wall,
plans the slalom and publishes it; the simulator drives it. The launch stops when the
simulator has finished.
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
    ]
    mapper = Node(
        package="minicar_processor", executable="lidar_mapper_node", output="screen",
        parameters=[{"output_dir": "/results/lidar", "gap_cm": LaunchConfiguration("gap_cm"),
                     "corridor_width_cm": LaunchConfiguration("corridor_width_cm")}],
    )
    simulator = Node(
        package="minicar_simulator", executable="simulator_node", output="screen",
        parameters=[{"output_dir": "/results/lidar", "wait_for_limits": True,
                     "post_radius_cm": 3.5, "max_steer_deg": LaunchConfiguration("max_steer_deg")}],
    )
    stop_when_simulator_ends = RegisterEventHandler(
        OnProcessExit(target_action=simulator, on_exit=[EmitEvent(event=Shutdown())])
    )
    return LaunchDescription(args + [mapper, simulator, stop_when_simulator_ends])
