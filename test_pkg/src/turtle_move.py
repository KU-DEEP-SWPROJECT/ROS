#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

from __future__ import print_function

import threading

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

from collections import deque
import time

TwistMsg = Twist
HZ_RATE = 0.01
BOT_COUNT = 2
BOT_NAME_DEFAULT = "turtle_"

#Rotation +1 is left (counter-clock-wise) 
class turtleBot:
    BASE_DIST_SPEED = 0.3#1.0
    BASE_ANGLE_SPEED = 1.0
    BASE_STOP_TIME =1.0
  # We should define proper values by trying vary values later.
    
    def __init__(self,robot_name):
        self.queue = deque([])
        self.name=robot_name
        self.publisher = rospy.Publisher('/'+self.name+'/cmd_vel', TwistMsg, queue_size = 20)
        self.twist_msg = TwistMsg()
        self.twist_msg.linear.x=0
        self.twist_msg.linear.y=0
        self.twist_msg.linear.z=0
        self.twist_msg.angular.x=0
        self.twist_msg.angular.y=0
        self.twist_msg.angular.z=0
    def move(self, move_time, dist_speed=None):
        if dist_speed is None:
            dist_speed = self.BASE_ANGLE_SPEED
        start_time = time.time()

        twist = self.twist_msg
        twist.linear.x = dist_speed
        while (time.time() - start_time <= move_time):
            self.publisher.publish(self.twist_msg)
            print("move", time.time() - start_time,"dist_speed",dist_speed)
            rospy.sleep(HZ_RATE)
            pass  # move command to actual moter.
        self.stop()

    def rotate(self, rotate_time, angle_speed=None):
        if angle_speed is None:
            angle_speed = self.BASE_ANGLE_SPEED
        start_time = time.time()

        twist = self.twist_msg
        twist.angular.z = angle_speed
        while (time.time() - start_time <= rotate_time):
            print("rotate ", time.time() - start_time,"angle speed ",angle_speed)
            self.publisher.publish(self.twist_msg)
            rospy.sleep(HZ_RATE)
            pass  # rotate command to actual moter.
        self.stop()

    def stop(self,stop_time = BASE_STOP_TIME):
        start_time = time.time()
        
        twist = self.twist_msg
        twist.linear.x = 0
        twist.angular.z = 0
        while (time.time() - start_time <= stop_time):
            self.publisher.publish(self.twist_msg)
            rospy.sleep(HZ_RATE)
            print("stop", time.time() - start_time)
            pass  # stop comamnd to actual moter.

    def push_cmd(self, cmd):
        self.queue.append(cmd)

    def make_queue(self, input_string):
            self.queue = deque(input_string.split("/"))

    def run(self):
        """Execute commands until the queue is empty.

        Or maybe we should run this function with thread and make this run forever."""
        while self.queue:
            q = self.queue.popleft()
            cmd, arg = q[0], q[1:]
            try:
                if cmd == 'F':
                    dist = int(arg)
                    self.move(dist / self.BASE_DIST_SPEED,0.5)
                elif cmd == 'R':
                    angle = int(arg)
                    self.rotate(angle / self.BASE_ANGLE_SPEED,0.5)
                elif cmd == 'S':
                    if arg:
                        self.stop(int(arg))
                    else:
                        self.stop()
                elif cmd == 'W':
                    time.sleep(float(arg))
                else:
                    print("unexpect cmd input")
            except ValueError:
                print("Invaild argument type:", type(arg))

class bot_controller:
    def __init__(self,bot_array_input):
        self.bot = bot_array_input
        self.cmd_array = []
        pass
    
    def push_cmd(self, cmd):
        self.cmd_array.append(cmd)
        
    def execute_cmd(self):
        cmd = self.cmd_array.pop().split(":")
        num, cmd = cmd
        num = int(num)
        if num < 1 or num > BOT_COUNT:
            print("bot num input error")
            return
        self.bot[num].make_queue(cmd)
        
    def execute_cmd_all(self):
        while self.cmd_array:
            self.execute_cmd()
            
    def run_all(self):
        for turtle in self.bot:
            if turtle is not None:
                turtle.run()
if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard')
    BOT = [None]
    for i in range(BOT_COUNT):
        BOT.append(turtleBot(BOT_NAME_DEFAULT+str(i+1)))
        print(BOT_NAME_DEFAULT+str(i))
    CMD_CENTER=bot_controller(BOT)

    import sys
    print(sys.version)

    ###
    def input_strings(n):
        result = []
        for _ in range(n):
            string_input = input("문자열을 입력하세요: ")
            result.append(string_input)
        return result

    #Bot = turtleBot("turtle_1")
    #Bot.make_queue("F2/R4/S2/F2")
    #Bot.run()

    # 사용자로부터 입력받을 문자열의 개수 설정
    n = int(input("몇 개의 문자열을 입력하시겠습니까? "))

    # 문자열 입력 받기
    strings_list = input_strings(n)

    # 결과 출력
    #print("입력받은 문자열들:", strings_list)
    ###
    while strings_list:
        CMD_CENTER.push_cmd(strings_list.pop())
    CMD_CENTER.execute_cmd_all()
    CMD_CENTER.run_all()
