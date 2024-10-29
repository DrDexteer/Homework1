from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'
    )

    arm_control_config = os.path.join(
        get_package_share_directory('arm_control'),
        'config',
        'arm_control.yaml'
    )
    
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_position_controller = TimerAction(
        period=10.0,
        actions=[position_controller]
    )

    return LaunchDescription([
        use_sim_time,
        joint_state_broadcaster,
        delayed_position_controller
    ])
