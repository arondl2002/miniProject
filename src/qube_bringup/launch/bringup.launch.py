import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("qube_driver"), "launch", "qube_driver.launch.py")
        )
    )

    rviz_config = os.path.join(get_package_share_directory("qube_description"), "rviz", "config.rviz")
    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        arguments = ["-d", rviz_config],
        output = "screen"
    )

    urdf_file = os.path.join(get_package_share_directory("qube_bringup"), "urdf", "controlled_qube.urdf.xacro")
    robot_description = xacro.process_file(urdf_file).toprettyxml()
    params = {"robot_description": robot_description}

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [params]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        qube_driver_launch
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_description",
            default_value = os.path.join(get_package_share_directory("qube_bringup"), "urdf", "controlled_qube.urdf.xacro"),
            description = "Full path to the robot URDF file"
        ),
    ] + nodes_to_start)
