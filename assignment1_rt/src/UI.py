#!/usr/bin/env python3




import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn





def turtle_callback():
    
	print("\n")
	print("Which turtle do you want to move? (insert 1 or 2)\n")
	turtle_decision = int(input("--> "))
	print("\n")
    
	print("To go ahead insert 1 \nTo rotate insert 2 \nTo translate left or right insert 3\n")
	movement_decision = int(input("--> "))
	print("\n")
    
	print("Choose velocity intensity\n")
	intensity_decision = float(input("--> "))
	print("\n")

	# ------------- building my_vel msg

	if movement_decision == 1 :
    		my_vel.linear.x = intensity_decision
    		my_vel.linear.y = 0
    		my_vel.angular.z = 0
    	
	elif movement_decision == 2 :
		my_vel.linear.x = 0
		my_vel.linear.y = 0
		my_vel.angular.z = intensity_decision
		
	elif movement_decision == 3 :
		my_vel.linear.x = 0
		my_vel.linear.y = intensity_decision
		my_vel.angular.z = 0
    	
	else :
		print("ERROR. movement code invalid")
		my_vel.linear.x = 0
		my_vel.angular.z = 0

	# ----------- sending my_vel msg to the right turtle

	if turtle_decision == 1 :

		pub1.publish(my_vel)
    
	elif turtle_decision == 2 :
        
		pub2.publish(my_vel)
    
	else :
		print("ERROR. only turtle1 and turtle2 are available, invalid input")
    
	rate.sleep()




def main():
    global pub1, pub2, rate, my_vel
    
    # Initialize the node
    rospy.init_node('Assignment_UI', anonymous=True)

    # Define the publisher and subscriber
    pub1 = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=1)
    
    # Define the service client and call it to spawn turtle2
    client1_spawn = rospy.ServiceProxy('/spawn', Spawn)
    client1_spawn(4.0, 4.0, 0.0, "turtle2")
    
    my_vel = Twist()
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown() :
    	turtle_callback()

    #rospy.spin()



# Running main()
main()

