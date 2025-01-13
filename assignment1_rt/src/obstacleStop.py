#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

# -----------------------------------------------------------------------------------------------

def turtle1_callback(msg):   # it will be of type Float32MultiArray
	global my_vel
	
	threshold = 0.5
	
	for i in range(16) :
	
		if  msg.array[i] < threshold :
		
			# Generate stopping msg
			my_vel.linear.x = 0
			my_vel.linear.y = 0
			my_vel.angular.z = 0
				       	
			# publish stopping msg to both turtle since I don't know which one is moving
			pub1.publish(my_vel)
	
	
# -----------------------------------------------------------------------------------------------
	
def turtle2_callback(msg):
	global my_vel
	
	threshold = 0.5
	
	for i in range(16) :
	
		if  msg.array[i] < threshold :
		
			# Generate stopping msg
			my_vel.linear.x = 0
			my_vel.linear.y = 0
			my_vel.angular.z = 0
				       	
			# publish stopping msg to both turtle since I don't know which one is moving
			pub2.publish(my_vel)



# -----------------------------------------------------------------------------------------------

def main():
	global pub1, pub2, pub_dist, my_vel, turtle1_x, turtle1_y, turtle1_theta, turtle2_x, turtle2_y, turtle2_theta, v1x, v1y, v1theta, v2x, v2y, v2theta
    
	# Initialize the node
	rospy.init_node('Assignment_Distance', anonymous=True)

	# Define the publisher and subscriber
	pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
	pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1)
	rospy.Subscriber('/turtle1/obstacle', Float32MultiArray, turtle1_callback)
	rospy.Subscriber('/turtle2/obstacle', Float32MultiArray, turtle2_callback)


	my_vel = Twist()

	print("\nNODE RUNNING\n")
	
	rospy.spin()

# -----------------------------------------------------------------------------------------------

# Running main()
if __name__ == '__main__':
	main()

