#!/usr/bin/env python3





import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn





def turtle_command_move():

	# Ask which turtle to move
    
	print("\n")
	print("Which turtle do you want to move? (insert 1 or 2)\n")
	turtle_decision = int(input("--> "))
	print("\n")
	
	# Choose velocities and building my_vel msg
	
	print("Choose velocity on X turtle axis\n")
	my_vel.linear.x = float(input("--> "))
	print("\n")
	
	print("Choose velocity on Y turtle axis\n")
	my_vel.linear.y = float(input("--> "))
	print("\n")
	
	print("Choose angular velocity aroud Z turtle axis\n")
	my_vel.angular.z = float(input("--> "))
	print("\n")


	# sending my_vel msg to the right turtle

	if turtle_decision == 1 :

		pub1.publish(my_vel)
    
	elif turtle_decision == 2 :
        
		pub2.publish(my_vel)
    
	else :
		print("ERROR: only turtle1 and turtle2 are available, invalid input")
    
	rospy.sleep(1)			# sleep 1 sec




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
    
    # rate = rospy.Rate(1)
    
    while not rospy.is_shutdown() :
    	turtle_command_move()

    #rospy.spin()





# Running main()
if __name__ == '__main__':
	main()

