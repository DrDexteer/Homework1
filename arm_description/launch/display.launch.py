from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    arm_description_path = FindPackageShare("arm_description").find("arm_description")
     
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "rviz", "my_config.rviz"]
            ),
            description="Percorso assoluto del file di configurazione di RViz."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Abilita l'uso del tempo di simulazione."
        )
    ]

    urdf_file = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", urdf_file]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node_links = Node(
        package="robot_state_publisher", #ros2 run robot_state_publisher robot_state_publisher
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": LaunchConfiguration("use_sim_time")}]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node_links,
        rviz_node
    ]

    
    return LaunchDescription(declared_arguments + nodes_to_start)

