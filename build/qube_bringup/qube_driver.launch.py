import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("qube_driver"), "launch", "qube_driver.launch.py")
        )
    )

    rviz_ConfigFile = os.path.join(get_package_share_directory("qube_description"), "rviz", "view_cube.rviz")
    rvizNode = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        arguments = ["-d", rviz_ConfigFile],
        output = "screen"
    )

    robot_state_publisher_Node = Node(
        package = "robot_state_publisher",
        executable = "robot_state publisher"
        parameters = [{"robotDescription": LaunchConfiguration("robotDescription")}],
        output = "screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robotDescription",
            default_value = os.path.join(get_package_share_directory("qube_bringup"), "urdf", "controlled_qube.urdf.xacro"),
            description = "Full path to the robot URDF file"
        ),
        qube_driver_launch,
        robot_state_publisher_Node,
        rvizNode
    ])