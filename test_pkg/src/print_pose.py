#!/usr/bin/env python

import sys
print(sys.version)

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion #, quaternion_from_euler
from math import pow, atan2, sqrt, pi


class TurtleBot3:

    def __init__(self):
        rospy.init_node('print_pose', anonymous = True)
        self.sub  = rospy.Subscriber('turtle_1/odom', Odometry, self.get_odom  )
        self.pub  = rospy.Publisher( '/cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(10)
        
        # Turtlebot3 pose #
        self.pos_x_2d = 0.0
        self.pos_y_2d = 0.0
        self.theta_2d = 0.0
        ###################
        self.prev_theta_2d = 0.0
        self.theta_2d_sum  = 0.0

        self.pure_x = 0.0
        self.pure_y = 0.0
        self.pure_th1 = 0.0
        self.pure_th2 = 0.0
            
    def get_odom(self, msg):
        pos_x, pos_y, theta = self.get_pose(msg)
        
        self.pos_x_2d = pos_x
        self.pos_y_2d = pos_y
        self.theta_2d = theta
        
        if   (self.theta_2d - self.prev_theta_2d) > 5.:
            d_theta = (self.theta_2d - self.prev_theta_2d) - 2 * pi            
        elif (self.theta_2d - self.prev_theta_2d) < -5.:
            d_theta = (self.theta_2d - self.prev_theta_2d) + 2 * pi
        else:
            d_theta = (self.theta_2d - self.prev_theta_2d)

        self.theta_2d_sum  = self.theta_2d_sum + d_theta
        self.prev_theta_2d = self.theta_2d
        
        self.theta_2d = self.theta_2d_sum
            
    def get_pose(self, data):
        q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
             data.pose.pose.orientation.z, data.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[2]	# euler_from_quaternion(q)[0] - roll
		                                    # euler_from_quaternion(q)[1] - pitch
		                                    # euler_from_quaternion(q)[2] - yaw <---
        
        self.pure_th1 = theta
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2
        self.pure_th2 = theta
        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y
        self.pure_x=pos_x
        self.pure_y=pos_y
        return pos_x, pos_y, theta
        
    def get_dist(self, goal_pose):
        return sqrt(pow((goal_pose.x-self.pos_x_2d),2) + pow((goal_pose.y-self.pos_y_2d),2))
        
    def get_lin_x(self, goal_pose, constant = 1.15):
        return constant * self.get_dist(goal_pose)
        
    def get_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pos_y_2d, goal_pose.x - self.pos_x_2d)

    def get_ang_z(self, goal_pose, constant = 1.15):        
        return constant * (self.get_angle(goal_pose) - self.theta_2d)
                
    def move2goal(self):
        goal_pose = Pose()

        goal_pose.x = input("Input x goal: " )
        goal_pose.y = input("Input y goal: " )
        tolerance = input("Input tolerance: ")

        t = Twist()
        cnt4print = 0	# counter for print pose every second(1Hz)

        while(self.get_dist(goal_pose) >= tolerance):
            
            if(cnt4print >=10):
                cnt4print = 0
                self.print_pose()
            
            cnt4print   = cnt4print + 1
            
            t.linear.x  = self.get_lin_x(goal_pose)
            t.linear.y  = t.linear.z  = 0
            
            t.angular.x = t.angular.y = 0
            t.angular.z = self.get_ang_z(goal_pose)

            self.pub.publish(t)
            self.rate.sleep()
            
        t.linear.x = t.angular.z = 0
        self.pub.publish(t)
        print("Robot is arrived at goal position!")
        
        rospy.spin()
	
    def getOdom(self):
        cnt4print = 0
        print("getOdom function start")
        print("ros shutdown"+(str)(rospy.is_shutdown()))
        while not rospy.is_shutdown():
            if(cnt4print >= 2):
                cnt4print = 0
                self.print_pose()
                self.print_pure_pose()
            self.rate.sleep()
            cnt4print +=1     
        print("print pose node close")
                
    def print_pose(self):
        print("pose      : p.x: %f,  p.y: %f,  th: %f" %(self.pos_x_2d, self.pos_y_2d, self.theta_2d))
    def print_pure_pose(self):
        print("pure pose: pp.x: %f, pp.y: %f,pp.th1:%f,pp.th2:%f" %(self.pure_x,self.pure_y,self.pure_th1,self.pure_th2))
if __name__ == '__main__':
    try:
        
        tb3 = TurtleBot3()
        tb3.getOdom()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
