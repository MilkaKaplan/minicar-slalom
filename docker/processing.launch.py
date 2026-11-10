"""Container 2 runs two ROS 2 nodes: the processor and the car simulator.

The processor exits after one image (+ a few seconds); the whole launch stops
when the simulator has finished driving.

Experiments without changing code, e.g. a car that cannot turn as sharply:
    docker compose run --rm processor ros2 launch /ros2_ws/processing.launch.py max_steer_deg:=10.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("gap_cm", default_value="10.0",
                              description="planned gap from post edge to car side"),
        DeclareLaunchArgument("max_steer_deg", default_value="30.0",
                              description="max steering angle of the simulated car"),
        DeclareLaunchArgument("speed_cm_s", default_value="20.0"),
    ]
    processor = Node(
        package="minicar_processor", executable="processor_node", output="screen",
        parameters=[{"output_dir": "/results", "exit_after": 1, "linger_s": 10.0,
                     "gap_cm": LaunchConfiguration("gap_cm")}],
    )
    simulator = Node(
        package="minicar_simulator", executable="simulator_node", output="screen",
        parameters=[{"output_dir": "/results",
                     "max_steer_deg": LaunchConfiguration("max_steer_deg"),
                     "speed_cm_s": LaunchConfiguration("speed_cm_s")}],
    )
    stop_when_simulator_ends = RegisterEventHandler(
        OnProcessExit(target_action=simulator, on_exit=[EmitEvent(event=Shutdown())])
    )
    return LaunchDescription(args + [processor, simulator, stop_when_simulator_ends])
