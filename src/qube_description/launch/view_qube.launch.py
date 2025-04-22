from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory("qube_description"), "urdf", "qube.urdf.xacro")
    robot_description = xacro.process_file(urdf_file).toxml()
    params = {'robot_description': robot_description}

    rviz_config = os.path.join(get_package_share_directory("qube_description"), "rviz", "config.rviz")

    return LaunchDescription([
        #Launch Node
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            name="robot_state_publisher",
            parameters = [params]
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher"
        ),
        Node(
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            arguments = ["-d", rviz_config]
        )
    ])
