#!/usr/bin/env python

import rospy
import geometry_msgs.msg

class MoveTurtle():

    def __init__(self):
        rospy.init_node("move_turtle")
        self.pb= rospy.Publisher("/turtle_1/cmd_vel",geometry_msgs.msg.Twist,queue_size=10)
        self.tw = geometry_msgs.msg.Twist()
   
    def move_turtle(self):
        self.tw.linear.x = self.tw.angular.z = 0.25
        self.pb.publish(self.tw)
        print("a")
if __name__ == '__main__':
    print("start")
    try:
        x = MoveTurtle()
        for i in range(100):
			x.move_turtle()
		#while not rospy.is_shutdown():
		 #   x.move_turtle()
    except rospy.ROSInterruptException:
		print("exception")
		pass
