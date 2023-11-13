#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import sys
print(sys.version)

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import threading
import time
from geometry_msgs.msg import Twist
from collections import deque
from typing import List, AnyStr, TYPE_CHECKING


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

BOT_COUNT = 2
BOT_NAME_DEFAULT = "turtle_"

# 회전: 양수 = 반시계방향
class TurtleBot:
    RATE_HZ = 100
    BASE_DIST_SPEED = 1.0
    BASE_ANGLE_SPEED = 1.0
    BASE_STOP_TIME = 1.0
    
    def __init__(self, robot_name):
        self.queue = deque([])
        self.RATE = rospy.Rate(self.RATE_HZ)
        self.name = robot_name
        self.publisher = rospy.Publisher('/'+self.name+'/cmd_vel', Twist, queue_size = 20)
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
    
    def move(self, move_time, dist_speed=None):
        if dist_speed is None:
            dist_speed = self.BASE_DIST_SPEED
        start_time = time.time()

        twist = self.twist_msg
        twist.linear.x = dist_speed
        print("[%s] move time: %.3f / speed: %.3f" % (self.name, move_time, dist_speed))
        published_time = time.time()
        while (time.time() - start_time <= move_time):
            delay = time.time() - published_time
            if delay > 5 / self.RATE_HZ:
                print("[%s] delayed publish! (move): %.6f" % (self.name, delay))
            self.publisher.publish(self.twist_msg)
            published_time = time.time()
            self.RATE.sleep()
        self.stop()

    def rotate(self, rotate_time, angle_speed=None):
        if angle_speed is None:
            angle_speed = self.BASE_ANGLE_SPEED
        start_time = time.time()

        twist = self.twist_msg
        twist.angular.z = angle_speed
        print("[%s] rotate time: %.3f / speed: %.3f" % (self.name, rotate_time, angle_speed))
        published_time = time.time()
        while (time.time() - start_time <= rotate_time):
            delay = time.time() - published_time
            if delay > 5 / self.RATE_HZ:
                print("[%s] delayed publish! (rotate): %.6f" % (self.name, delay))
            self.publisher.publish(self.twist_msg)
            published_time = time.time()
            self.RATE.sleep()
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
        self.queue = deque(command_string.split("/"))

    def run(self):
        while self.queue:
            q = self.queue.popleft()
            cmd, arg = q[0], q[1:]
            try:
                if cmd == 'F':
                    dist = float(arg)
                    self.move(dist / self.BASE_DIST_SPEED, 0.1)
                elif cmd == 'R':
                    angle = float(arg)
                    self.rotate(angle / self.BASE_ANGLE_SPEED, 1.0)
                elif cmd == 'S':
                    if arg:
                        self.stop(int(arg))
                    else:
                        self.stop()
                elif cmd == 'W':
                    time.sleep(float(arg))
                else:
                    print("[%s] [Turtlebot#run] Error: Invalid command keyword: %s" % (self.name, cmd))
            except ValueError:
                print("[%s] [Turtlebot#run] Error: Invaild argument type: %s (%s)" % (self.name, arg, type(arg).__name__))

class BotController:
    def __init__(self, array_of_bot: List[TurtleBot]):
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
            temp_cmd = input("turtlebot> ")
            if not temp_cmd:
                break
            controller.push_command(temp_cmd)
        controller.execute_all_commands()
        controller.run_all_turtlebots()
        print("All executions finished.")
    except KeyboardInterrupt:
        rospy.signal_shutdown()
