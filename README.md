# Lab 5-6-7: ARIAC Competition

## Overview

This is the final submission for the ariac competition. This lab also depends on lab 6, where inverse kinematics solutions were calculated.

### Lab 5-6-7 Link

Lab 5 Link: [Laboratory #5_20231016_cert.pdf](https://canvas.case.edu/courses/38747/assignments/509274)
Lab 6 Link: [Laboratory #6_20231027_cert.pdf](https://canvas.case.edu/courses/38747/assignments/509275)
Lab 7 Link: [Laboratory #7_20231113_cert.pdf](https://canvas.case.edu/courses/38747/assignments/527520)
ARIAC 2019 Environment: [Ariac 2019] (https://bitbucket.org/osrf/ariac/wiki/2019/Home)

## Basic Install (Lab 5)

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

#### Run the Competition Node

To run the competition, use the following:

	roslaunch ariac_entry lab5.launch

## Problems with the current node

Under the src folder, the lab5_node-backup.cpp is the old node that only worked for lab5. For the final node, please see the rewritten lab5_node.cpp.
The node is still not functioning properly and i still can't figure out how to fix some of the problems, so i put a lot of explanations next to the code for context of what i was trying to do inside each section of the node (ik_service node alone is working fine though). 
