# Homework1: ROS 2 Robotic Arm Simulation

## :package: About

This project contains code for simulating a robotic arm using ROS 2. It includes the robot's structure definition, control interface, Gazebo integration for physical simulation, and a virtual camera.

### Main Packages:
- **arm_control**: Manages joint control and control interfaces.
- **arm_description**: Contains the robot description, URDF/XACRO files, and camera configuration.
- **arm_gazebo**: Configurations and launch files for simulating the robot in Gazebo.

## :hammer: Build

To use this project, clone the repository into the `src` folder of your ROS 2 workspace. Make sure to install any missing dependencies with:
```
$ rosdep install -i --from-path src --rosdistro humble -y
```
Then, build the workspace:
```
$ colcon build --packages-select arm_control arm_description arm_gazebo
```
Source the setup files
```
$ . install/setup.bash
```

## :white_check_mark: Usage
### Launch the Robot in RViz

To visualize the robot directly in RViz (without Gazebo), use the following command:
```
$ ros2 launch arm_description display.launch.py
```
This will load the robot model in RViz for inspection and visualization purposes.

### Launch the Simulation
Start the robotic arm in Gazebo with:
```
$ ros2 launch arm_gazebo arm_world.launch.py
```
### Control the Robot’s Joints
To view robot with your desired configuration modify the command_msg in arm_controller_node.cpp file ,and you can launch:
```
$ ros2 launch arm_gazebo arm_world.launch.py
```
