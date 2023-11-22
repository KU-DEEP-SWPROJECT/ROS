#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import sys
print(sys.version)


# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import threading
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from collections import deque
#from typing import List, AnyStr, TYPE_CHECKING
from tf.transformations import euler_from_quaternion #, quaternion_from_euler
from math import pow, atan2, sqrt, pi


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

BOT_COUNT = 3
BOT_NAME_DEFAULT = "turtle_"

# 회전: 양수 = 반시계방향
class TurtleBot:
    RATE_HZ = 100
    BASE_DIST_SPEED = 1.0
    BASE_ANGLE_SPEED = 1.0
    BASE_STOP_TIME = 1.0

    ANGLE_TORLERANCE = 0.1
    
    def __init__(self, robot_name):
        self.queue = deque([])
        self.RATE = rospy.Rate(self.RATE_HZ)
        self.name = robot_name
        self.publisher = rospy.Publisher('/'+self.name+'/cmd_vel', Twist, queue_size = 20)
        self.sub  = rospy.Subscriber('/'+self.name+'/odom', Odometry, self.get_odom  )
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

        # Turtlebot3 pose #
        self.pos_x_2d = 0.0
        self.pos_y_2d = 0.0
        self.theta_2d = 0.0
        ###################
        self.prev_theta_2d = 0.0
        self.theta_2d_sum  = 0.0
    
    def move(self, dist, dist_speed=None, m_time=None):
        if dist_speed is None:
            dist_speed = self.BASE_DIST_SPEED
        if m_time is None:
            move_time = dist/dist_speed
        else:
            move_time=m_time
        start_time = time.time()
        #need time Not None, speed is None && all not None
        twist = self.twist_msg
        twist.linear.x = dist_speed
        #print("[%s] move time: %.3f / speed: %.3f" % (self.name, move_time, dist_speed))
        published_time = time.time()
        
        pos_now = Pose()
        print("pose now :",pos_now)
        pos_now.x = self.pos_x_2d
        pos_now.y = self.pos_y_2d
        while (float(self.get_dist(pos_now))<dist):
            delay = time.time() - published_time
            if delay > float(5) / self.RATE_HZ:
                print("[%s] delayed publish! (move): %.6f" % (self.name, delay))
            self.publisher.publish(self.twist_msg)
            published_time = time.time()
            self.RATE.sleep()
        print("distance : ",(self.get_dist(pos_now)))
        end_time = time.time()
        print("[%s] move time: %.3f / speed: %.3f" % (self.name, end_time-start_time, dist_speed))
        print("pose now : ",self.pos_x_2d," ",self.pos_y_2d)   
        self.stop()

    def rotate(self, angle, angle_speed=None, r_time=None):
        if angle_speed is None:
            angle_speed = self.BASE_ANGLE_SPEED
        #need time Not None, speed is None && all not None
        if r_time is None:
            rotate_time = angle/angle_speed
        else:
            rotate_time = r_time
        if angle<0:
            angle_speed *= -1
        start_time = time.time()

        pre_angle = self.theta_2d
        if(angle>=2*pi):
            while angle < 2*pi:
                angle -=2*pi
        if angle <=-2*pi:
            while angle>-2*pi:
                angle +=2*pi
        #now goal angle -2 pi ~ 2 pi

        #if want, goal angle -pi ~ +pi
        
            
        goal_angle = self.theta_2d + angle
        
        if(goal_angle > 2 * pi):
            goal_angle -= 2*pi
        if(goal_angle <0):
            goal_angle += 2*pi
        print(self.RATE_HZ,self.ANGLE_TORLERANCE)
        print("angle now%f" %(self.theta_2d))
        print("goal angle : %f" %(goal_angle))
        print(float(5 / self.RATE_HZ))

            
        twist = self.twist_msg
        twist.angular.z = angle_speed
        #print("[%s] rotate time: %.3f / speed: %.3f" % (self.name, rotate_time, angle_speed))

        published_time = time.time()
        print("distance: "+(str)(abs(self.theta_2d-goal_angle)))
        cnt = 0
        while (abs(self.theta_2d-goal_angle) > self.ANGLE_TORLERANCE) or (2*pi - abs(self.theta_2d-goal_angle) > self.ANGLE_TORLERANCE):
            delay = time.time() - published_time
            cnt +=1
            if cnt>50:
                print("now : %f , Goal : %f   distance :%f"%(self.theta_2d, goal_angle,abs(self.theta_2d-goal_angle)))
                cnt=0
            if delay > float(5) / self.RATE_HZ:
                print("[%s] delayed publish! (rotate): %.6f" % (self.name, delay))
            self.publisher.publish(self.twist_msg)
            published_time = time.time()
            self.RATE.sleep()
        end_time = time.time()
        print("[%s] rotate time: %.3f / speed: %.3f" % (self.name, end_time-start_time, angle_speed))
        self.stop()

    def stop(self, stop_time=None):
        if stop_time is None:
            stop_time = self.BASE_STOP_TIME
        start_time = time.time()
        
        twist = self.twist_msg
        twist.linear.x = 0
        twist.angular.z = 0
        print("[%s] stop time: %.3f" % (self.name, stop_time))
        published_time = time.time()
        while (time.time() - start_time <= stop_time):
            delay = time.time() - published_time
            if delay > float(5)/ self.RATE_HZ:
                print("[%s] delayed publish! (stop): %.6f" % (self.name, delay))
            self.publisher.publish(self.twist_msg)
            published_time = time.time()
            self.RATE.sleep()

    def push_command(self, cmd):
        self.queue.append(cmd)
    
    def push_commands(self, command_string):
        for command in command_string.split('/'):
            self.push_command(command)

    def set_queue(self, command_string):
        self.queue = deque(command_string.split("/"))

    def run(self):
        while self.queue:
            q = self.queue.popleft()
            cmd, arg = q[0], q[1:]
            try:
                if cmd == 'F':
                    dist = float(arg)
                    self.move(dist , 0.5)
                elif cmd == 'R':
                    angle = float(arg)
                    self.rotate(angle ,0.5)
                elif cmd == 'S':
                    if arg:
                        self.stop(float(arg))
                    else:
                        self.stop()
                elif cmd == 'W':
                    time.sleep(float(arg))
                else:
                    print("[%s] [Turtlebot#run] Error: Invalid command keyword: %s" % (self.name, cmd))
            except ValueError:
                print("[%s] [Turtlebot#run] Error: Invaild argument type: %s (%s)" % (self.name, arg, type(arg).__name__))
        # Turtlebot3 pose #
        self.pos_x_2d = 0.0
        self.pos_y_2d = 0.0
        self.theta_2d = 0.0
        ###################
        self.prev_theta_2d = 0.0
        self.theta_2d_sum  = 0.0
            
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
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2

        pos_x = data.pose.pose.position.x
        pos_y = data.pose.pose.position.y

        return pos_x, pos_y, theta
        
    def get_dist(self, goal_pose):
        return sqrt(pow((goal_pose.x-self.pos_x_2d),2) + pow((goal_pose.y-self.pos_y_2d),2))
        
    def get_lin_x(self, goal_pose, constant = 1.15):
        return constant * self.get_dist(goal_pose)
        
    def get_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pos_y_2d, goal_pose.x - self.pos_x_2d)

    def get_ang_z(self, goal_pose, constant = 1.15):        
        return constant * (self.get_angle(goal_pose) - self.theta_2d)
                
    def move2goal(self,goal_x,goal_y):
        goal_pose = Pose()

        #goal_pose.x = input("Input x goal: " )
        #goal_pose.y = input("Input y goal: " )
        tolerance = 0.05

        t = Twist()
        cnt4print = 0	# counter for print pose every second(1Hz)

        while(self.get_dist(goal_pose) >= tolerance):
            
            if(cnt4print >= 10):
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
                
    def print_pose(self):
        print("p.x: %f,  p.y: %f,  th: %f" %(self.pos_x_2d, self.pos_y_2d, self.theta_2d))
        

class BotController:
    def __init__(self, array_of_bot):#: List[TurtleBot]):
        self.bots = array_of_bot
        self.command_array = []
        pass
    
    def push_command(self, command):#: str):
        self.command_array.append(command)
        
    def execute_command(self):
        num, _, cmd = self.command_array.pop().rpartition(':')
        try:
            num = int(num)
        except ValueError:
            print("[BotController#execute_cmd] Error: Bot number is not an integer type: %s (%s)" % (num, type(num).__name__))
        if num < 1 or num > BOT_COUNT:
            print("[BotController#execute_cmd] Error: Bot number (%d) is out of range." % num)
            return
        self.bots[num].set_queue(cmd)
        
    def execute_all_commands(self):
        while self.command_array:
            self.execute_command()
            
    def run_all_turtlebots(self):
        threads = []
        for turtle_bot in self.bots:
            if turtle_bot is not None:
                threads.append(threading.Thread(target=turtle_bot.run))
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

if __name__=="__main__":
    try:

        rospy.init_node('teleop_twist_keyboard')
        bots = [None]
        for i in range(1, BOT_COUNT + 1):
            bots.append(TurtleBot(BOT_NAME_DEFAULT + str(i)))
        
        controller=BotController(bots)
        print("터틀봇에게 전송할 명령을 입력하세요.")
        print("(그만 입력하려면 아무것도 입력하지 않고 Enter를 누르세요.)")
        while True:
            temp_cmd = raw_input("turtlebot> ")
            if not temp_cmd:
                break
            controller.push_command(temp_cmd)
        controller.execute_all_commands()
        controller.run_all_turtlebots()
        rospy.spin()
        print("All executions finished.")
    except KeyboardInterrupt:
        rospy.signal_shutdown()
