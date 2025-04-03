from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory("qube_description"), "urdf", "qube.urdf.xacro")
    robot_description = xacro.process_file(urdf_file).toprettyxml()
    params = {'robot_description': robot_description}
    return LaunchDescription([
        #Launch Node
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            parameters = [params],
            output = "screen"
        ),
        Node(
            package = "rviz2",
            executable = "rviz2",
            output = "screen"
        )
    ])
