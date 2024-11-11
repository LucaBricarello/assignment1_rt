#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn





def turtle_callback(msg):

    rospy.loginfo("Turtle subscriber@[%f, %f, %f]", msg.x, msg.y, msg.theta)

    rate.sleep()




def main():
    global pub, rate
    
    # Initialize the node
    rospy.init_node('Assignment_UI', anonymous=True)

    # Define the publisher and subscriber
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('turtle1/pose', Pose, turtle_callback)
    
    # Define the service client and call it to spawn turtle2
    client1_spawn = rospy.ServiceProxy('/spawn', Spawn)
    client1_spawn(4.0, 4.0, 0.0, "turtle2")
    
    rate = rospy.Rate(1)

    rospy.spin()




# Running main()
main()

