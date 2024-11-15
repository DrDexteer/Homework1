# arm_gazebo.launch.py

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    
    return LaunchDescription([
        
        # Include the arm_world.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('arm_gazebo'), 'launch', 'arm_world.launch.py'])
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Include the arm_control.launch.py to spawn controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('arm_control'), 'launch', 'arm_control.launch.py'])
            )
        ),

    ])