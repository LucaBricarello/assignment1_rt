#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from turtlesim.srv import TeleportAbsolute
import math

# -----------------------------------------------------------------------------------------------

def turtle1_callback(msg):
	global turtle1_x, turtle1_y, turtle1_theta, my_vel

	# Saving updated position in global variables
	turtle1_x = msg.x
	turtle1_y = msg.y
	turtle1_theta = msg.theta
	
	# Checking if the turtle got out of the borders
	if msg.x > 10 or msg.x < 1 or msg.y > 10 or msg.y < 1 :
	
		print("WARNING: turtle1 too close to the border, slightly coming back and stopping it")
		
		# Generate stopping msg
		my_vel.linear.x = 0
		my_vel.linear.y = 0
		my_vel.angular.z = 0
				       	
		# publish stopping msg to both turtle since I don't know which one is moving
		pub1.publish(my_vel)
		
		# teleporting the turtle back into the border
		teleport_turtle1 = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
		
		if msg.x > 10 :
			teleport_turtle1(9.9, msg.y, msg.theta)
		if msg.x < 1 :
			teleport_turtle1(1.1, msg.y, msg.theta)
		if msg.y > 10 :
			teleport_turtle1(msg.x, 9.9, msg.theta)
		if msg.y < 1 :
			teleport_turtle1(msg.x, 1.1, msg.theta)
	
	# Calling the function that computes the distance and publishes it
	compute_distance()	
	
# -----------------------------------------------------------------------------------------------
	
def turtle2_callback(msg):
	global turtle2_x, turtle2_y, turtle2_theta, my_vel

	# Saving updated position in global variables
	turtle2_x = msg.x
	turtle2_y = msg.y
	turtle2_theta = msg.theta
	
	# Checking if the turtle got out of the borders
	if msg.x > 10 or msg.x < 1 or msg.y > 10 or msg.y < 1 :
		# Generate stopping msg
		my_vel.linear.x = 0
		my_vel.linear.y = 0
		my_vel.angular.z = 0
		
		print("WARNING: turtle2 too close to the border, stopping it")
        	
		# publish stopping msg to both turtle since I don't know wich one is moving
		pub2.publish(my_vel)
		
		# teleporting the turtle back into the border
		teleport_turtle2 = rospy.ServiceProxy('/turtle2/teleport_absolute', TeleportAbsolute)
		
		if msg.x > 10 :
			teleport_turtle2(9.9, msg.y, msg.theta)
		if msg.x < 1 :
			teleport_turtle2(1.1, msg.y, msg.theta)
		if msg.y > 10 :
			teleport_turtle2(msg.x, 9.9, msg.theta)
		if msg.y < 1 :
			teleport_turtle2(msg.x, 1.1, msg.theta)
	
	# Calling the function that computes the distance and publishes it
	compute_distance()	
	
# -----------------------------------------------------------------------------------------------

def compute_distance() :
	global turtle1_x, turtle1_y, turtle1_theta, turtle2_x, turtle2_y, turtle2_theta, pub_dist, my_vel, v1x, v1y, v1theta, v2x, v2y, v2theta
	
	# Computing the distance
	distance = math.sqrt(pow(turtle1_x - turtle2_x, 2) + pow(turtle1_y - turtle2_y, 2))
		
	# Preparing the msg
	distance_msg = Float32()
	distance_msg.data = distance
        
        # Publish the msg
	pub_dist.publish(distance_msg)
	# To check if the msg is sent correctly run the following command on a fresh terminal to listen to the topic: rostopic echo /turtle_distance
	
	teleport_turtle1 = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	teleport_turtle2 = rospy.ServiceProxy('/turtle2/teleport_absolute', TeleportAbsolute)
	
	threshold = 1
        
        # Checking if the turtles are distant enough to satisfy the threshold
	if (distance <= threshold)  :
		# If not I stop them and teleport the moving one slightly back
	
		backoff_dist = threshold - distance + 0.1
		delta_x = turtle2_x -  turtle1_x
		delta_y = turtle2_y -  turtle1_y
		gamma = math.atan2(delta_y, delta_x)
		
		if v1x != 0 or v1y != 0 or v1theta != 0 :
			des_x = turtle1_x - backoff_dist * math.cos(gamma)
			des_y = turtle1_y - backoff_dist * math.sin(gamma)
			teleport_turtle1(des_x, des_y, turtle1_theta)
				
		elif v2x != 0 or v2y != 0 or v2theta != 0 :
			des_x = turtle2_x + backoff_dist * math.cos(gamma)
			des_y = turtle2_y + backoff_dist * math.sin(gamma)
			teleport_turtle2(des_x, des_y, turtle2_theta)
		
		# Generate stopping msg
		my_vel.linear.x = 0
		my_vel.linear.y = 0
		my_vel.angular.z = 0
		
		print("WARNING: turtles too close to each other, stopping them")
        	
        	# Publishing stopping msg
		pub1.publish(my_vel)
		pub2.publish(my_vel)

		

# -----------------------------------------------------------------------------------------------

# Callback function to save the latest velocities of both turtles

def turtle1_save_latest_velocity(msg) :
	global v1x, v1y, v1theta
	
	v1x = msg.linear.x
	v1y = msg.linear.y
	v1theta = msg.angular.z
	
def turtle2_save_latest_velocity(msg) :
	global v2x, v2y, v2theta
	
	v2x = msg.linear.x
	v2y = msg.linear.y
	v2theta = msg.angular.z
	
	

# -----------------------------------------------------------------------------------------------

def main():
	global pub1, pub2, pub_dist, my_vel, turtle1_x, turtle1_y, turtle1_theta, turtle2_x, turtle2_y, turtle2_theta, v1x, v1y, v1theta, v2x, v2y, v2theta
    
	# Initialize the node
	rospy.init_node('Assignment_Distance', anonymous=True)

	# Define the publisher and subscriber
	pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
	pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1)
	pub_dist = rospy.Publisher('/turtle_distance', Float32, queue_size=1)
	rospy.Subscriber('/turtle1/pose', Pose, turtle1_callback)
	rospy.Subscriber('/turtle2/pose', Pose, turtle2_callback)
	rospy.Subscriber('/turtle1/cmd_vel', Twist, turtle1_save_latest_velocity)
	rospy.Subscriber('/turtle2/cmd_vel', Twist, turtle2_save_latest_velocity)

	my_vel = Twist()
	
	# Setting up variables so that the first loop doesn't give warnings and problems
	turtle1_x = 5.54
	turtle1_y = 5.54
	turtle1_theta = 0
	turtle2_x = 4
	turtle2_y = 4
	turtle2_theta = 0
	v1x = 0
	v1y = 0
	v1theta = 0
	v2x = 0
	v2y = 0
	v2theta = 0
	
	print("\nNODE RUNNING\n")
	
	rospy.spin()

# -----------------------------------------------------------------------------------------------

# Running main()
if __name__ == '__main__':
	main()

