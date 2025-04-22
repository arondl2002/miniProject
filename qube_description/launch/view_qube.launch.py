from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Retrieve path to urdf file
    urdf_file = os.path.join(get_package_share_directory("qube_description"), "urdf", "qube.urdf.xacro")
    # Parse file as xml
    robot_description = xacro.process_file(urdf_file).toxml()

    # Retrieve rviz config file
    rviz_config = os.path.join(get_package_share_directory("qube_description"), "rviz", "config.rviz")

    # Return a list of declared nodes to start
    return LaunchDescription([
        # Declare robot_state_publisher node
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            name="robot_state_publisher",
            parameters = [{'robot_description': robot_description}]
        ),
        # Declare joint_state_publisher node
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher"
        ),
        # Declare rviz node
        Node(
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            arguments = ["-d", rviz_config]
        )
    ])
