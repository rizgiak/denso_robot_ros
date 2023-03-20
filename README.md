Please refer to https://wiki.ros.org/denso_robot_ros.

This repository dedicated for COBOTTA Robot not OSS type in ROS Noetic.

Before execute the program, makesure to set the bcap_slave license number from VirtualTP and also change the communication from TP to Ethernet. 

Fill the value of Ethernet with your PC IP Address.
=======
For COBOTTA Robot Standard Type (Not OSS).

The most updated version in [noetic-devel](https://github.com/rizgiak/denso_robot_ros/tree/noetic-devel) branch.


Install moveit library
'''
cd denso_ws/src/
wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove  moveit_tutorials  # this is cloned in the next section
wstool update -t .
'''

Install additional library
'''
sudo apt-get install ros-noetic-ruckig
'''

