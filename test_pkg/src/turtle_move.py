#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import sys
print(sys.version)

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import enum
import threading
from threading import Thread, Condition
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from collections import deque
#from math import *
import math
from math import pow, atan2, asin, sqrt, pi
#from Gunwoo import *

from incident_detect import IncidentDetector


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class State(enum.Enum):
    PARKING = 0
    FORWARD = 1
    BACKWARD = 2
    ROTATE = 3
    FOLLOW = 4
    WAIT_ROTATE = 5
    CARRYING = 6
    CARRY_MOVE_FORWARD = 7
    CARRY_MOVE_BACKWARD = 8


# 회전: 양수 = 반시계방향
class TurtleBot:
    __ID = 1
    RATE_HZ = 100
    BASE_STOP_TIME = 1.0
    BOT_NAME_PREFIX = "turtle_"

    ANGLE_TORLERANCE = 0.1
    
    # limitations of turtlebot3 burger model
    # MAX_LINEAR_SPEED  = 0.22  # m/s
    # MAX_ANGULAR_SPEED = 2.84  # rad/s
    
    # let's limit them slightly slower
    MAX_LINEAR_SPEED  = 0.2  # m/s
    MAX_ANGULAR_SPEED = 2.8  # rad/s
    
    def __init__(self, robot_name=None):
        self.queue = deque([])
        self.RATE = rospy.Rate(self.RATE_HZ)
        if robot_name is None:
            self.name = self.BOT_NAME_PREFIX + str(TurtleBot.__ID)
            TurtleBot.__ID += 1
        else:
            self.name = self.BOT_NAME_PREFIX + robot_name
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
        self.pure_theta = 0.0
        ###################
        self.prev_theta_2d = 0.0
        self.theta_2d_sum  = 0.0

        self.state = State.PARKING
        self.got_incident = False
        self.condition = Condition()
        self.incident_detector = IncidentDetector(self.name)
        self.incident_detector.callback = self.incident_callback

    
    @staticmethod
    def __range_predict(radius, move=0.0):
        result_dist = [0.0 for _ in range(360)]
        sin_sq = [math.sin(i * math.pi / 180) ** 2 for i in range(-90, 90)]
        cos_sq = [math.cos(i * math.pi / 180) ** 2 for i in range(-90, 90)]
        r_sq = radius * radius
        d_sq = move * move
        for angle in range(-90, 90):
            angle_idx = angle + 90
            _x, _D = move * sin_sq[angle_idx], math.sqrt(d_sq * sin_sq[angle_idx] * cos_sq[angle_idx] + r_sq * cos_sq[angle_idx])
            x1, x2 = _x + _D, _x - _D
            result_dist[angle+180] = math.sqrt((x1 - move)**2 + r_sq - x1**2)
            result_dist[angle%360] = math.sqrt((x2 - move)**2 + r_sq - x2**2)
        return result_dist
    
    def incident_callback(self, data):
        limitation = self.__range_predict(self.incident_detector.INCIDENT_DISTANCE, self.twist_msg.linear.x / 5)  # Scan data is published for 5 Hz.
        angles = list(filter(lambda x: 0.01 < data.ranges[x] < limitation[x], range(360)))
        fronts = list(filter(lambda angle: angle < 30 or angle > 330, angles))
        backs = list(filter(lambda angle: 150 < angle < 210, angles))
        self.condition.acquire()
        if self.state == State.PARKING:
            pass
        elif self.state == State.FORWARD:
            if fronts and not self.got_incident:
                self.got_incident = True  # STOP
            elif not fronts:
                self.got_incident = False
        elif self.state == State.BACKWARD:
            if backs and not self.got_incident:
                self.got_incident = True  # STOP
            elif not backs:
                self.got_incident = False
        elif self.state == State.ROTATE:
            pass
        elif self.state == State.FOLLOW:
            if fronts and not self.got_incident:
                self.got_incident = True  # STOP
            elif not fronts:
                self.got_incident = False
        elif self.state == State.WAIT_ROTATE:
            if fronts and not self.got_incident:
                self.got_incident = True  # STOP
            elif not fronts:
                self.got_incident = False
        elif self.state == State.CARRYING:
            pass
        elif self.state == State.CARRY_MOVE_FORWARD:
            pass
        elif self.state == State.CARRY_MOVE_BACKWARD:
            if backs and not self.got_incident:
                self.got_incident = True  # STOP
        self.condition.notify()
        self.condition.release()
    
    def check_incident(self):
        st = time.time()
        self.condition.acquire()
        if self.got_incident:
            while not rospy.is_shutdown():
                rospy.loginfo("[%s] Incident detected! Wait for removed...", self.name)
                self.publisher.publish(Twist())
                self.condition.wait_for(lambda: not self.got_incident, timeout=3.0)
                if not self.got_incident:
                    rospy.loginfo("[%s] Incident has removed. Continuing the task...", self.name)
                    break
        self.condition.notify()
        self.condition.release()
        return time.time() - st
        



    def move(self, dist=None, speed=None, move_time=None):
        # (speed, move_time) has higher priority
        check_param_bit = (dist is None) << 2 | (speed is None) << 1 | (move_time is None)
        if check_param_bit | 0b010 == 0b111:
            # dist and move_time is not given (111, 101) => raise error
            raise ValueError("Lack of arguments.")
        if check_param_bit in {0b011, 0b110}:
            # only dist or move_time is given (011, 110) => use max speed
            speed = self.MAX_LINEAR_SPEED/3
        if check_param_bit & 0b100 == 0:
            if check_param_bit & 0b001:
                # dist is given, speed is given or MAX_LINEAR_SPEED (001, 011) => calculate time
                move_time = dist / speed
            elif check_param_bit == 0b010:
                # dist and move_time is given and speed is not given (010) => calculate speed
                speed = dist / move_time
        # 100, 000 are OK as itself
        print("move start")
        print("move input: "+str(dist))
        
        twist = self.twist_msg
        twist.linear.x = speed

        if move_time < 0:
            twist.linear.x = -1*speed
            move_time *= -1
        if  dist <0 :
            twist.linear.x = -1*speed
            dist *= -1
        
        start_time = time.time()
        incident_time=time.time()
        #need time Not None, speed is None && all not None
        
        
        #print("[%s] move time: %.3f / speed: %.3f" % (self.name, move_time, dist_speed))
        published_time = time.time()

        pos_now = Pose()
        print("pose now :",pos_now)
        print("dist now :",dist )
        pos_now.x = self.pos_x_2d
        pos_now.y = self.pos_y_2d
        cnt = 0
        while (float(self.get_dist(pos_now))<dist):
            delay = time.time() - published_time
            cnt +=1
            if cnt> self.RATE_HZ:
                print("distance : ",(self.get_dist(pos_now)))
                print("pose now : ",self.pos_x_2d," ",self.pos_y_2d)   
                cnt=0
            if delay > float(5) / self.RATE_HZ:
                print("[%s] delayed publish! (move): %.6f" % (self.name, delay))
            self.publisher.publish(self.twist_msg)
            start_time += self.check_incident()
            published_time = time.time()
            self.RATE.sleep()
        print("[%s] end move | distance goal : %d " % (self.name, self.get_dist(pos_now) ))
        #print("end distance : ",(self.get_dist(pos_now)))
        end_time = time.time()
        print("[%s] move time: %.3f / speed: %.3f" % (self.name, end_time-start_time, speed))
        #print("pose now : ",self.pos_x_2d," ",self.pos_y_2d)   
        self.stop()

    def rotate(self, angle=None, speed=None, rotate_time=None):
        # (speed, rotate_time) has higher priority
        check_param_bit = (angle is None) << 2 | (speed is None) << 1 | (rotate_time is None)
        if check_param_bit | 0b010 == 0b111:
            # angle and rotate_time is not given (111, 101) => raise error
            raise ValueError("Lack of arguments.")
        if check_param_bit in {0b011, 0b110}:
            # only angle or rotate_time is given (011, 110) => use max speed
            speed = self.MAX_ANGULAR_SPEED/4
        if check_param_bit & 0b100 == 0:
            if check_param_bit & 0b001:
                # angle is given, speed is given or MAX_LINEAR_SPEED (001, 011) => calculate time
                rotate_time = angle / speed
            elif check_param_bit == 0b010:
                # angle and rotate_time is given and speed is not given (010) => calculate speed
                speed = angle / rotate_time
        # 100, 000 are OK as itself
        
        if rotate_time < 0:
            speed *= -1
            rotate_time *= -1

        start_time = time.time()

        pre_angle = self.pure_theta
        if(angle>=2*pi):
            while angle < 2*pi:
                angle -=2*pi
        if angle <=-2*pi:
            while angle>-2*pi:
                angle +=2*pi
        #now goal angle -2 pi ~ 2 pi

        #if want, goal angle -pi ~ +pi
        
            
        goal_angle = self.pure_theta + angle
        
        if(goal_angle > 2 * pi):
            goal_angle -= 2*pi
        if(goal_angle <0):
            goal_angle += 2*pi
        print(self.RATE_HZ,self.ANGLE_TORLERANCE)
        print("angle now%f" %(self.theta_2d))
        print("goal angle : %f" %(goal_angle))
        print(float(5 / self.RATE_HZ))

            
        twist = self.twist_msg
        twist.angular.z = speed
        #print("[%s] rotate time: %.3f / speed: %.3f" % (self.name, rotate_time, angle_speed))

        published_time = time.time()
        print("distance: "+(str)(abs(self.pure_theta-goal_angle)))
        cnt = 0
        while (abs(self.pure_theta-goal_angle) > self.ANGLE_TORLERANCE): #or (2*pi - abs(self.theta_2d-goal_angle) > self.ANGLE_TORLERANCE):
            delay = time.time() - published_time
            cnt +=1
            if cnt>50:
                print("now : %f , speed: %f , Goal : %f   distance :%f"%(self.pure_theta,self.twist_msg.angular.z ,goal_angle,abs(self.pure_theta-goal_angle)))
                cnt=0
            if delay > float(5) / self.RATE_HZ:
                print("[%s] delayed publish! (rotate): %.6f" % (self.name, delay))
            self.publisher.publish(self.twist_msg)
            published_time = time.time()
            self.RATE.sleep()
        end_time = time.time()
        print("[%s] end rotate | distance goal : %d " % (self.name, self.pure_theta-goal_angle ))
        print("[%s] rotate time: %.3f / speed: %.3f" % (self.name, end_time-start_time, speed))
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
            if delay > 5 / self.RATE_HZ:
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
        self.queue = deque(command_string.split('/'))

    def run(self):
        while self.queue:
            q = self.queue.popleft()
            cmd, arg = q[0], q[1:]
            try:
                if cmd == 'F':
                    dist = float(arg)
                    self.move(dist=dist)
                elif cmd == 'R':
                    angle = float(arg)
                    self.rotate(angle=angle)
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
        self.pure_theta = theta
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
             
        theta = self.euler_from_quaternion(q)[2]	# euler_from_quaternion(q)[0] - roll
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

    def euler_from_quaternion(self,q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x,y,z,w= q

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class BotController:
    def __init__(self, array_of_bot):
        self.bots = array_of_bot
        self.command_array = []
        pass
    
    def push_command(self, command: str):
        self.command_array.append(command)
        
    def execute_command(self):
        num, _, cmd = self.command_array.pop().rpartition(':')
        try:
            num = int(num)
        except ValueError:
            print("[BotController#execute_cmd] Error: Bot number is not an integer type: %s (%s)" % (num, type(num).__name__))
        if num < 1 or num >= len(self.bots):
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
        BOT_COUNT = int(input("터틀봇 운영 대수: "))
        bots = [None]
        for i in range(BOT_COUNT):
            bots.append(TurtleBot())
        
        controller = BotController(bots)
        
        #map_file = []
        #map_file = get_points(0)        
        #print(map_file)

        print("터틀봇에게 전송할 명령을 입력하세요.")
        print("(그만 입력하려면 아무것도 입력하지 않고 Enter를 누르세요.)")
        while True:
            temp_cmd = input("turtlebot> ")
            if not temp_cmd:
                break
            controller.push_command(temp_cmd)
        controller.execute_all_commands()
        controller.run_all_turtlebots()
        print("All executions finished.")
    except KeyboardInterrupt:
        rospy.signal_shutdown()
