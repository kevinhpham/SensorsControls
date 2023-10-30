# SensorsControls
FetchRobotPickandPlace

Launches a gazebo simulation of the fetch robot in a simple 3d environment. In this environment, the fetch robot picks up a blue cube on a table and moves it around before placing it back down onto the table.

This repository contains some of the packages that were created and sourced online from other people. The package My_fetch contains all of the code and files that our group produced for this project.
The object detection system is located in my_fetch/objectDetection

World files for gazebo simulations in my_fetch/worlds

my_fetch models contains models for simulation environment from online

my_fetch/script contains control system for fetch robot

my_fetech/launch contains related launch files to start the demo.
To launch the demo, use "roslaunch my_fetch gazebo_sim.launch" in a shell then wait until the gazebo simulation is open. Afterwards open another shell and use "roslaunch my_fetch pick_place_demo.launch" to start the pick and place control system demo.

Code uses world files to launch a gazebo simulation with the fetch robot in a 3d environment. Plan to use object detection based on the fetch's RGBD camera to detech poses of cube in environment but was unfinished. Robot control system would then control fetch robot to pick up objects.

Paul Morian 12907578 - Produced gazebo simulation and related launch files of environment (40% contribution)

Ste Ven Lee 99141893 - Produced object detection system and pose estimation (unfinished) (10% contribution)

Hoang Nam Kevin Pham 14306233 - Produced Robot control system and related launch files (50% contribution)

This project was built for Ubuntu 18.04 and Ros melodic.



# Packages used
MoveIt from http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html (binary install)
  Used to perform joint path planning and to control the robot arm

moveit_visual_tools from https://github.com/ros-planning/moveit_visual_tools (source install)
  Used to help visualize path planning for the robot arm in Rviz

Fetch gazebo from https://docs.fetchrobotics.com/gazebo.html (binary install)
  Used to launch simulated fetch robot in gazebo

Fetch_ros,fetch_description,fetch_teleop,fetch_moveit_config from https://github.com/ZebraDevs/fetch_ros (source install)
  Used to connect fetch robot to moveit interface.

My_fetch created by the group
  Contains code created by our team to launch gazebo simulation and to get the fetch robot to perform a pick and place task. Also contains testing files based on code from the moveit tutorials that the team used to learn how to use the moveit packages.




  
