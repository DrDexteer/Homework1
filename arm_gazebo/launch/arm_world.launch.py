from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r -v 1 empty.sdf',
            description='Arguments for gz_sim',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
    ]
    
    # Path to URDF/XACRO
    arm_description_path = get_package_share_directory('arm_description')
    urdf_arm = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")

    # Use xacro to process the URDF file
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", urdf_arm])
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration('use_sim_time')}],
    )  

    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'arm_robot', '-allow_renaming', 'true']
    )

    arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm_control'), 'launch', 'arm_control.launch.py')
        )
    )
    
    nodes = [
        robot_state_publisher_node,
        gazebo_ignition,
        gz_spawn_entity,
        arm_control_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)
