"""Gazebo Harmonic + ros2_control + the gait node.

    ros2 launch hexapod_description sim.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive with the keyboard
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from hexapod_gait import robot_config as cfg


def generate_launch_description():
    pkg = FindPackageShare("hexapod_description")
    world = PathJoinSubstitution([pkg, "worlds", "course.sdf"])
    xacro_file = PathJoinSubstitution([pkg, "urdf", "hexapod.urdf.xacro"])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": ["-r -v 3 ", world]}.items())

    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    # Spawn a little above the floor in the boot pose; the legs fold to the
    # stand pose during the gait node's start-up ease.
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "hexapod", "-z", "0.12"])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        parameters=[{"config_file": PathJoinSubstitution([pkg, "config", "bridge.yaml"]),
                     "use_sim_time": True}])

    joint_states = Node(package="controller_manager", executable="spawner",
                        arguments=["joint_state_broadcaster"])
    legs = Node(package="controller_manager", executable="spawner",
                arguments=["leg_controller"])
    gait = Node(package="hexapod_gait", executable="gait_node", output="screen",
                parameters=[{"use_sim_time": True}])

    # arm_controller only exists in controllers.yaml when robot_config.ARM is
    # on, so spawning it unconditionally would fail on a plain hexapod. One
    # switch drives the URDF, the controller list and this.
    controllers = [legs]
    if cfg.ARM:
        # Holds the arm in its rest pose until an arm node drives it.
        controllers.append(Node(package="controller_manager", executable="spawner",
                                arguments=["arm_controller"]))
    controllers.append(gait)

    return LaunchDescription([
        gazebo,
        state_publisher,
        bridge,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_states])),
        RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=controllers)),
    ])
