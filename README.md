# SensorsControls
FetchRobotPickandPlace

Launches a gazebo simulation of the fetch robot in a simple 3d environment. In this environment, the fetch robot picks up a blue cube on a table and moves it   around before placing it back down onto the table.

To launch the demo, use "roslaunch my_fetch gazebo_sim.launch" in a shell then wait until the gazebo simulation is open. Afterwards, open another shell and use "roslaunch my_fetch pick_place_demo.launch" to start the pick and place demo.

This project was built for Ubuntu 18.04 and Ros melodic.

# Packages used
MoveIt from http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html(binary install)
  Used to perform joint path planning and to control the robot arm

moveit_visual_tools from https://github.com/ros-planning/moveit_visual_tools(source install)
  Used to help visualize path planning for the robot arm in Rviz

Fetch gazebo from https://docs.fetchrobotics.com/gazebo.html (binary install)
  Used to launch simulated fetch robot in gazebo

Fetch_ros from https://github.com/ZebraDevs/fetch_ros(source install)
  Used for fetch_moveit_config to connect fetch robot to moveit interface.






  
