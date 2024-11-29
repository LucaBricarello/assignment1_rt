# How to run the code

Firstly you have to be sure that ROS noetic has been installed in the correct way.
After sourcing the right setup files, to run the code you simply need to clone this repository inside a ROS workspace and then run each one of the following commands in a different terminal:

	- roscore
	- rosrun turtlesim turlesim_node

Always in different terminals, but this time inside the ROS workspace folder, run these commands:

	- rosrun assignment1_rt UI.py
	- rosrun assignment1_rt Distance.py

If you want to 'listen' the distance topic created by the Distance.py node, in another terminal run:

	- rostopic echo /turtle_distance

# What has been done

This repository contains the code of 2 ROS nodes for the simulator turtlesim.

The first node is UI.py.

This node spawns a second turtle in the simulator, called turtle2, then asks the user which turtle he wants to move.
The user will have to insert 1 to move turtle1 or 2 to move turtle2.  After that the node asks the velocities that the chosen turtle will keep for 1 second.
It asks for linear velocity on axis X and on axis Y, then for angular velocity on axis Z (referring to the chosen turtle's frame).
A message with this velocity is built and sent to the velocity topic of the chosen turtle, then the node waits for one second and asks again which turtle to move.

The second node is Distance.py.

This node subcribes to the topics of the position of the two turtles and computes the distance between them. This value is saved in a Float32 message and published on the topic '/turtle_distance'.
This node also stops the turles if they get too close to each other (threshold distance set to 1) or if they are too close to the boundaries.
When the turtles stop for any of the 2 reasons above they have already passed the limit values, so at the next iteration they will be stopped again and so on. For this reason as soon as the turtles are stopped they are brought back again in the 'safe zone' with the teleport service.
