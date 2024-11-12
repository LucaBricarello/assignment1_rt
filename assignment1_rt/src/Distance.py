#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

# -----------------------------------------------------------------------------------------------

def turtle1_callback(msg):
	global turtle1_x, turtle1_y

	# Saving updated position in global variables
	turtle1_x = msg.x
	turtle1_y = msg.y
	
	# Calling the function that computes the distance and publishes it
	compute_distance()	
	
# -----------------------------------------------------------------------------------------------
	
def turtle2_callback(msg):
	global turtle2_x, turtle2_y

	# Saving updated position in global variables
	turtle2_x = msg.x
	turtle2_y = msg.y
	
	# Calling the function that computes the distance and publishes it
	compute_distance()	
	
# -----------------------------------------------------------------------------------------------

def compute_distance() :
	global turtle1_x, turtle1_y, turtle2_x, turtle2_y, pub_dist
	
	# Computing the distance
	distance = math.sqrt(pow(turtle1_x - turtle2_x, 2) + pow(turtle1_y - turtle2_y, 2))
		
	# Preparing the msg
	distance_msg = Float32()
	distance_msg.data = distance
        
        # Publish the msg
	pub_dist.publish(distance_msg)
	# To check if the msg is sent correctly run the following command on a fresh terminal to listen to the topic: rostopic echo /turtle_distance
        
        # Debug
	rospy.loginfo(f"Published distance: {distance}\n")

# -----------------------------------------------------------------------------------------------

def main():
	global pub1, pub2, pub_dist, my_vel, rate, turtle1_x, turtle1_y, turtle2_x, turtle2_y
    
	# Initialize the node
	rospy.init_node('Assignment_UI', anonymous=True)

	# Define the publisher and subscriber
	pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
	pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1)
	pub_dist = rospy.Publisher('/turtle_distance', Float32, queue_size=1)
	rospy.Subscriber('/turtle1/pose', Pose, turtle1_callback)
	rospy.Subscriber('/turtle2/pose', Pose, turtle2_callback)

	#my_vel = Twist()
	
	# Setting up variables so that the first loop doesn't give warnings and problems
	turtle1_x = 5.54
	turtle1_y = 5.54
	turtle2_x = 4
	turtle2_y = 4
	
	rospy.spin()

# -----------------------------------------------------------------------------------------------

# Running main()
main()

