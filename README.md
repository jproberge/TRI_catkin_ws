# TRI_catkin_ws
This repo contains the entire catkin workspace for vision-based picking. 

More info to come...


things michael had to install:
- ur_modern_driver
- ur_msgs
- Eigen
- sudo apt-get install ros-melodic-soem
- installed muparser
- cloned new robotiq
- catkin_make
- rosdep install --from-paths src/ --ignore-src --rosdistro indigo
- sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs


workflow with hardware:
- roscore
- roslaunch ur_bringup ur5_bringup.launch robot_ip:=172.22.22.3
- roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
# for rviz
- roslaunch ur5_moveit_config moveit_rviz.launch config:=true
- rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
- rosrun vision_based_picking FindRobotiqLedCam.py
- rosrun vision_based_picking test_move.py

workflow with simulation:
- roscore
- roslaunch ur5_moveit_config demo.launch
- rosrun vision_based_picking FindRobotiqLedCam.py
- rosrun vision_based_picking test_move.py

