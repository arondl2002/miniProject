from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory("qube_description"), "urdf", "qube.urdf.xacro")
    return LaunchDescription([
        #Launch Node
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            arguments = [urdf_file],
            output = "screen"
        ),
        Node(
            package = "rviz2",
            executable = "rviz2",
            output = "screen"
        )
    ])