# FYP-C144
This repository contains the program package for Heng Wei Chong's Final Year Project for NTU Bachelor of Engineering, for the AY 22/23.

Project Title (C144): Development of Dynamic Balancing Safety Program Package for Mobile Manipulators in Gazebo

This project was carried out in collaboration with A*STAR Singapore (Robotics and Autonomous Systems Department)

Supervisor: Professor Yeo Song Huat

Co-Supervisor: Dr Yuan Qilong


To run the program package:

Operating system: Ubuntu 20.04

ROS distro: ROS Noetic

Ensure that you have the following dependencies installed:
- message_generation (sudo apt-get install ros-noetic-message-generation)
- roscpp (sudo apt-get install ros-noetic-roscpp)
- rospy (sudo apt-get install ros-noetic-rospy)
- std_msgs (sudo apt-get install ros-noetic-std-msgs)
- tf2_geometry_msgs (sudo apt-get install ros-noetic-tf2-geometry-msgs)
- tf2_ros (sudo apt-get install ros-noetic-tf2-ros)
- urdf (sudo apt-get install ros-noetic-urdf)
- visualization_msgs (sudo apt-get install ros-noetic-visualization-msgs)
- tf (sudo apt-get install ros-noetic-tf)
- gazebo_ros (sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control)
- Husky packages (sudo apt-get install ros-noetic-husky-desktop)

1) Create a workspace via the command "mkdir -p ~/($name_of_workspace)_ws/src", with your own selected name for workspace.
2) Pull "dynamic_balancing_v2" folder program package into a source folder of the workspace and build via 'catkin build'.
3) Run launch file with command 'roslaunch dynamic_balancing_v2 dynamic_balancing_husky.launch
4) To run the following nodes in order
  
  a) rosrun dynamic_balancing_v2 find_CG_with_marker
  
     -For the input "Please input the model name that is to be calculated: ", please enter 'robot'.
     -For the input "Please input the reference frame where you would like the marker to be displayed in: ", please enter 'base_link'.
     
  b) rosrun dynamic_balancing_v2 imu_filter
  
     -For the input "Please input the ROS topic to extract IMU data from: ", please enter '/MobileManipulator/imu_topic'.
     
  c) rosrun dynamic_balancing_v2 get_ZMP
  
     -For the input "Please input the ROS topic to extract IMU data from: ", please enter '/filtered_imu'.
     -For the input "Please input the reference frame where you would like the marker to be displayed in: ", please enter 'base_link'.
     
  d) rosrun dynamic_balancing_v2 wheel_position
  
     -For the input "Please input the reference frame with respect to the wheels: ", please enter 'base_link'.
   
  e) rosrun dynamic_balancing_v2 support_polygon
  
Optional Nodes to run:
1) rosrun dynamic_balancing_v2 wheel_contact
2) rosrun dynamic_balancing_v2 check_ground_condition
   -For the input "Please input the ROS topic to extract IMU data from: ", please enter '/filtered_imu'.
   
   
Node description:
1) find_CG_with_marker:
   - Displays coordinates of CoM on command terminal and publishes a visualisation marker to RVIZ on the location of CoM
   
2) imu_filter:
   - Filters IMU sensor signals with moving average filter and publishes them on a new ROS topic
   
3) get_ZMP
   - Displays coordinates of ZMP on command terminal and publishes a visualisation marker to RVIZ on the location of ZMP
   
4) wheel_position
   - Displays coordinates of all wheels on husky on command terminal and publishes them to a new ROS topic
   
5) support_polygon
   - Displays distance between ZMP and the edge of support polygon as a percentage on command terminal, using the percentage as a definition of quality of      stability.
 
6) wheel_contact
   - Displays expected distance travelled by Odometry and actual distance travelled in Gazebo and the difference between them.
   
7) check_ground_condition
   - Displays the pitch and roll angles of the robot on command terminal.
