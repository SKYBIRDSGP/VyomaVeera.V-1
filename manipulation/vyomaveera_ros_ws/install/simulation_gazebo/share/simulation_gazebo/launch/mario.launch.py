#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    package_name = "simulation_gazebo"
    urdf_file = "manipulator.urdf"

    # Paths
    robot_desc_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        urdf_file
    )

    controller_yaml = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "manipulator.yaml"
    )

    # Robot description
    robot_description_content = Command(["xacro ", robot_desc_path])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher (REQUIRED)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description
        ],
    )

    # Spawn robot into Gazebo Sim
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_entity",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "mario",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
    )

    # Joint State Broadcaster (spawner only!)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Arm / position controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controller_yaml,
        ],
        output="screen",
    )

    # ---- Sequencing ----
    # Spawn JSB only AFTER robot exists in Gazebo
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Spawn arm controller only AFTER JSB is active
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        spawn_robot,
        delay_jsb,
        delay_controller,
    ])
