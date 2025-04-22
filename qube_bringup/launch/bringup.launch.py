import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Getting external launch file from qube_driver
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("qube_driver"), "launch", "qube_driver.launch.py")
        )
    )

    # Get rviz config from qube_description directory 
    rviz_config = os.path.join(get_package_share_directory("qube_description"), "rviz", "config.rviz")
    # Declare rviz node, passing the config as argument
    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        arguments = ["-d", rviz_config],
    )

    # Retrieving urdf file
    urdf_file = os.path.join(get_package_share_directory("qube_bringup"), "urdf", "controlled_qube.urdf.xacro")
    # Parse file as xacro
    robot_description = xacro.process_file(urdf_file).toprettyxml()

    # Declare robot_state_publisher_node, passing robot_description as parameter
    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )

    # Declare joint_state_publisher_node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
    )

    # Declare qube_controller_node 
    qube_controller_node = Node(
        package="qube_controller",
        executable="qube_controller",
        name="qube_controller_node"
    )

    # List containing nodes + launch variables to start on launch
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        qube_controller_node,
        qube_driver_launch
    ]

    # Return a launch description generated from node list
    return LaunchDescription(nodes_to_start)
