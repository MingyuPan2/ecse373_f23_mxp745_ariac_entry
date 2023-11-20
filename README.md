# Lab 5: ARIAC 2019 Part 1: Installation etc.

## Overview

This laboratory will develop the foundation for a system to address elements of a ARIAC competition.
The lab will extend existing knowledge of ROS message publishing and subscribing to the use of ROS
services and action servers.

### Lab 4 Link

Lab 5 Link: [Laboratory #5_20231016_cert.pdf](https://canvas.case.edu/courses/38747/assignments/509274)

ARIAC 2019 Environment: [Ariac 2019] (https://bitbucket.org/osrf/ariac/wiki/2019/Home)

## Basic Install

### Install Simulation Environment

For ros noetic, the ARIAC 2019 environment can be cloned as follows:

	mkdir -p ~/ariac_ws/src
	cd ~/ariac_ws/src
	git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git
	rosdep install --from-paths cwru_ariac_2019 --ignore-src -r -y
	cd ../
	sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"
	
The repo will be cloned to the new ariac_ws, where any missing dependencies will be installed. The simulator will the be installed. 

### Install ARIAC Comp and ARIAC node

The ARIAC Simulation does not work on its own at this time. It must be invoked through another package. That package should be built in its own workspace so that it is not conflated with the simulator itself. Use the following processes:

	mkdir -p ~/ecse_373_ariac_ws/src
	cd ~/ecse_373_ariac_ws/src
	git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git
	rosdep install --from-paths ecse_373_ariac --ignore-src -r -y
	cd ../
	catkin_make
	source devel/setup.bash
	
This package provides an inverse kinematics library for the Universal Robotics robotic manipulators used in the simulation and will be used later to help move the arm to pick up and place parts specified in an order.

#### Bug Fixing

Running the launch file for the first time will not work, since there is a bug in the "empy" module for Python3. THere are two ways to fix this. FIrst, simply ues the following: 

	roslaunch ecse_373_ariac ecse_373_ariac.launch python:=false
	
However, the above methos is inefficient and may cause problems in the future. SO, use the second method instead:

	sudo patch /usr/lib/python3/dist-packages/em.py < `rospack find ecse_373_ariac`/patches/empy.patch
	
This method will patch the em.py file of the Python3 empy module. By doing so, the bug can be fixed properly and permanently. 

### Launch Competition Environment

To launch ARIAC 2019, use the following:

	roslaunch ariac_entry lab5.launch
	
This will launch the environment of a warehouse where everything is positioned. 

To kill the launch properly and quickly, use:

	killall gzserver gzclient roslaunch
	
When the environment was launched, it will not start properly until until the play botton was pressed at the bottom middle left of the screen.

#### Run Competition Node

To run the entry node for the competition, use the following:

	rosrun ariac_entry lab5_node

## Problems to Fix

The node can be ran if the play botton was not pressed. However, the node will output "Competition start failed", since the ariac/start_competition service will not have popped up if the play botton was not pressed. Currently, I do not know how to fix this problem. The node will always output the order number, the shipment number, what the products are and which bin they are in. The location of the bin will also be outputed. The location of a desired product in the arm's frame will also be outputed.

I still do not fully understand the buffer part of the lab and i do not know if the transformation is working correctly.
