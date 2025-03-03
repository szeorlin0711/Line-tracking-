#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty #To reset turtlesim if needed
import math

class TurtleController:
    def __init__(self):
        rospy.init_node("turtle_cmd_vel_listener", anonymous=True)

        #subscribe to the /cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        #publish to turtlesim velocity command topic
        self.turtle_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("TurtleSim Subscriber Node Started!")

    def cmd_vel_callback(self, msg):
        """Callback function that recieves velocity commands from /cmd_vel and forwards to turtlesim """
        rospy.loginfo(f"Received cmd_vel: Linear={msg.linear.x}, Angular={msg.angular.z}")
        self.turtle_pub.publish(msg) #Forward command to turtlesim

    def run(self):
        rospy.spin() #keep node running

if __name__ == '__main__':
    try:
        turtle_controller = TurtleController()
        turtle_controller.run()
    except rospy.ROSInternalException:
        pass