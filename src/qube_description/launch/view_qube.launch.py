from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Henter "path"en til URDF (robotbeskrivelsen), samt prosessere Xacro-filen
    urdf_file = os.path.join(get_package_share_directory("qube_description"), "urdf", "qube.urdf.xacro")
    robot_description = xacro.process_file(urdf_file).toxml()
    params = {'robot_description': robot_description}

    # Henter konfigurasjonsfilen for RViz
    rviz_config = os.path.join(get_package_share_directory("qube_description"), "rviz", "config.rviz")

    # Returnerer en liste med noder som kjøres opp ved initialisering
    return LaunchDescription([
        # Initialiserer publisering for robottilstand med utgnagspunkt i URDF
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            name="robot_state_publisher",
            parameters = [params]
        ),
        
        # Initialiserer publisering av leddtilstand for roboten 
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher"
        ),

        # Initialiserer RViz med spesifik konfigurasjon for robot visualisering
        Node(
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            arguments = ["-d", rviz_config]
        )
    ])
