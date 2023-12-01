#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import sys
# print(sys.version)

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time
from enum import Enum, auto
from threading import Thread, Condition
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from collections import deque
from math import pow, sin, cos, atan2, asin, sqrt, pi
from typing import List

from incident_detect import IncidentDetector


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TWO_PI = pi + pi


class State(Enum):
    PARKING = auto()
    FORWARD = auto()
    BACKWARD = auto()
    ROTATE = auto()
    FOLLOW = auto()
    WAIT_ROTATE = auto()
    CARRY_WAIT = auto()
    CARRY_ALIGN = auto()
    CARRY_READY_TO_STICK = auto()
    CARRY_STICK = auto()
    CARRY_STANDBY = auto()
    CARRYING_MOVE_FORWARD = auto()
    CARRYING_MOVE_BACKWARD = auto()


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
        self.twist_pub = rospy.Publisher('/'+self.name+'/cmd_vel', Twist, queue_size = self.RATE_HZ)
        self.twist_msg = Twist()
        self.odom_sub = rospy.Subscriber('/'+self.name+'/odom', Odometry, self.get_odom)
        
        self.pos = [0.0, 0.0]  # x, y
        self.direction_raw = 0.0  # as radians (   0 ~ 2*pi)
        self.direction_acc = 0.0  # as radians (-inf ~ +inf; accumulation)

        self.state = State.PARKING
        self.got_incident = False
        self.condition = Condition()
        self.incident_detector = IncidentDetector(self.name)
        self.incident_detector.callback = self.incident_callback
        
        self.last_front_data = None  # for carrying task
        rospy.on_shutdown(lambda: self.twist_pub.publish(Twist()))
    
    @staticmethod
    def __range_predict(radius, move=0.0):
        result_dist = [0.0 for _ in range(360)]
        sin_sq = [sin(i * pi / 180) ** 2 for i in range(-90, 90)]
        cos_sq = [cos(i * pi / 180) ** 2 for i in range(-90, 90)]
        r_sq = radius * radius
        d_sq = move * move
        for angle in range(-90, 90):
            angle_idx = angle + 90
            _x, _D = move * sin_sq[angle_idx], sqrt(d_sq * sin_sq[angle_idx] * cos_sq[angle_idx] + r_sq * cos_sq[angle_idx])
            x1, x2 = _x + _D, _x - _D
            result_dist[angle+180] = sqrt((x1 - move)**2 + r_sq - x1**2)
            result_dist[angle%360] = sqrt((x2 - move)**2 + r_sq - x2**2)
        return result_dist
    
    def incident_callback(self, data: LaserScan):
        limitation = self.__range_predict(self.incident_detector.INCIDENT_DISTANCE, self.twist_msg.linear.x / 5)  # Scan data is published for 5 Hz.
        angles = list(filter(lambda x: 0.01 < data.ranges[x] < limitation[x], range(360)))
        fronts = list(filter(lambda angle: angle < 30 or angle > 330, angles))
        backs = list(filter(lambda angle: 150 < angle < 210, angles))
        self.last_front_data = [(data.ranges[x], x) for x in range(-30, 30) if data.ranges[x] > 0.01]
        with self.condition:
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
            elif self.state in {State.CARRY_WAIT, State.CARRY_ALIGN, State.CARRY_STICK, 
                                State.CARRY_STANDBY, State.CARRYING_MOVE_FORWARD}:
                pass
            elif self.state == State.CARRYING_MOVE_BACKWARD:
                if backs and not self.got_incident:
                    self.got_incident = True  # STOP
            self.condition.notify()
    
    def check_incident(self):
        st = time.time()
        with self.condition:
            if self.got_incident:
                while not rospy.is_shutdown():
                    rospy.loginfo("[%s] Incident detected! Wait for removed...", self.name)
                    self.twist_pub.publish(Twist())
                    self.condition.wait_for(lambda: not self.got_incident, timeout=3.0)
                    if not self.got_incident:
                        rospy.loginfo("[%s] Incident has removed. Continuing the task...", self.name)
                        break
            self.condition.notify()
        return time.time() - st

    def move(self, dist=None, speed=None, move_time=None):
        # (speed, move_time) has higher priority
        check_param_bit = (dist is None) << 2 | (speed is None) << 1 | (move_time is None)
        if check_param_bit | 0b010 == 0b111:
            # dist and move_time is not given (111, 101) => raise error
            raise ValueError("Lack of arguments.")
        if check_param_bit in {0b011, 0b110}:
            # only dist or move_time is given (011, 110) => use default speed
            speed = self.MAX_LINEAR_SPEED / 3
        if check_param_bit & 0b100 == 100:
            # dist is not given, but move_time is given (100, 110) => calculate dist
            dist = speed * move_time
        elif check_param_bit & 0b001 == 0b001:
            # calculate rotate_time
            move_time = dist / speed
        
        twist = self.twist_msg
        twist.linear.x = speed

        if dist < 0:
            twist.linear.x *= -1
            dist *= -1
        
        with self.condition:
            if self.state == State.CARRY_STANDBY:
                if speed < 0:
                    self.state = State.CARRYING_MOVE_BACKWARD
                else:
                    self.state = State.CARRYING_MOVE_FORWARD
            elif self.state.name.startswith('CARRY_'):
                pass
            else:
                if speed < 0:
                    self.state = State.BACKWARD
                else:
                    self.state = State.FORWARD
            self.condition.notify()

        x, y = self.pos
        print("[%s] start move | pose now :" % self.name, self.pos)
        print("[%s] start move | dist now :" % self.name, dist)
        print("[%s] start move | move estimated time: %.3f / speed: %.3f" % (self.name, move_time, speed))
        start_time = time.time()
        cnt = 0
        while (self.get_dist(x, y) < dist):
            if rospy.is_shutdown():
                return
            cnt += 1
            if cnt > self.RATE_HZ:
                print("[%s] moving | distance :" % self.name, self.get_dist(x, y))
                print("[%s] moving | pose now :" % self.name, x, y)   
                cnt = 0
            self.twist_pub.publish(self.twist_msg)
            start_time += self.check_incident()
            self.RATE.sleep()
        print("[%s] end move | distance goal : %d " % (self.name, self.get_dist(x, y)))
        end_time = time.time()
        print("[%s] end move | move time: %.3f / speed: %.3f" % (self.name, end_time-start_time, speed))
        self.stop()

    def rotate(self, angle=None, speed=None, rotate_time=None):
        # (speed, rotate_time) has higher priority
        check_param_bit = (angle is None) << 2 | (speed is None) << 1 | (rotate_time is None)
        if check_param_bit | 0b010 == 0b111:
            # angle and rotate_time is not given (111, 101) => raise error
            raise ValueError("Lack of arguments.")
        if check_param_bit in {0b011, 0b110}:
            # only angle or rotate_time is given (011, 110) => use default speed
            speed = self.MAX_ANGULAR_SPEED / 4
        if check_param_bit & 0b100 == 0b100:
            # angle is not given, but rotate_time is given (100, 110) => calculate angle
            angle = speed * rotate_time
        elif check_param_bit & 0b001 == 0b001:
            # calculate rotate_time
            rotate_time = angle / speed
        
        if rotate_time < 0:
            speed *= -1
            rotate_time *= -1

        angle %= TWO_PI
        if angle > pi:
            angle -= TWO_PI
        goal_direction_acc = self.direction_acc + angle
        
        print("[%s] start rotate | angle now : %f (%f)" % (self.name, self.direction_acc, self.direction_raw))
        print("[%s] start rotate | goal angle : %f (%f)" % (self.name, goal_direction_acc, goal_direction_acc % TWO_PI))
        print("[%s] start rotate | angle torlerance : %f" % (self.name, self.ANGLE_TORLERANCE))
            
        twist = self.twist_msg
        twist.angular.z = speed
        print("[%s] start rotate | rotate estimated time: %.3f / speed: %.3f" % (self.name, rotate_time, speed))
        
        with self.condition:
            if not self.state.name.startswith('CARRY_'):
                self.state = State.ROTATE
            self.condition.notify()

        start_time = time.time()
        
        if angle < 0:
            get_angle_dist = lambda: self.direction_acc - goal_direction_acc
        else:
            get_angle_dist = lambda: goal_direction_acc - self.direction_acc

        print("[%s] start rotate | distance :" % self.name, get_angle_dist())
        cnt = 0
        while get_angle_dist() > self.ANGLE_TORLERANCE:
            if rospy.is_shutdown():
                return
            cnt += 1
            if cnt > self.RATE_HZ:
                print("[%s] rotating | now(raw) : %f / speed : %f / Goal(raw) : %f / distance(acc) : %f" % \
                    (self.name, self.direction_raw, self.twist_msg.angular.z , goal_direction_acc, get_angle_dist()))
                cnt = 0
            self.twist_pub.publish(self.twist_msg)
            self.RATE.sleep()
        end_time = time.time()
        print("[%s] end rotate | distance goal : %d "% (self.name, get_angle_dist()))
        print("[%s] end rotate | rotate time: %.3f / speed: %.3f" % (self.name, end_time-start_time, speed))
        self.stop()

    def stop(self, stop_time=None):
        if stop_time is None:
            stop_time = self.BASE_STOP_TIME
        start_time = time.time()
        
        twist = self.twist_msg
        twist.linear.x = 0
        twist.angular.z = 0
        
        with self.condition:
            if self.state == State.CARRY_STICK:
                self.state = State.CARRY_STANDBY
            elif self.state.name.startswith('CARRY_'):
                pass
            else:
                self.state = State.PARKING
            self.condition.notify()

        print("[%s] stop time: %.3f" % (self.name, stop_time))
        while (time.time() - start_time <= stop_time):
            self.twist_pub.publish(self.twist_msg)
            self.RATE.sleep()
    
    def carry(self, target_dist=0):
        with self.condition:
            self.state = State.CARRY_WAIT
            self.condition.notify()
        
        ANGLE_TORLERANCE_DEGREES = self.ANGLE_TORLERANCE * 180 / pi
        
        while True:
            if rospy.is_shutdown():
                return
            if self.last_front_data is None:
                self.RATE.sleep()
                continue
            try:
                nearest_dist, nearest_angle = min(self.last_front_data)
                if abs(nearest_angle) < ANGLE_TORLERANCE_DEGREES:
                    break
                with self.condition:
                    self.state = State.CARRY_ALIGN
                    self.condition.notify()
                if nearest_angle > 180:
                    nearest_angle -= 360
                self.rotate(angle=nearest_angle * pi / 180.0,
                            speed=self.MAX_ANGULAR_SPEED / 15.0)
            except ValueError:  # no objects in front of the bot
                self.stop()
                return
            
        with self.condition:
            self.state = State.CARRY_READY_TO_STICK
            self.condition.notify()
        
        with self.condition:
            print("[%s] Waiting the controller to change my state..." % self.name)
            self.condition.wait_for(lambda: self.state == State.CARRY_STICK)
            self.condition.notify()
        # In controller, it should check all bots are ready to stick, and set all states to starting sticking
        # so all bots stick to the object simultaniously.
        
        self.move(dist=nearest_dist - target_dist + 0.05)
        # make the wheels over-spin (5cm further) to make sure the bot is combined to frame.

    def push_command(self, cmd):
        self.queue.append(cmd)
    
    def push_commands(self, command_string):
        for command in command_string.split('/'):
            self.push_command(command)

    def set_queue(self, command_string):
        self.queue = deque(command_string.split('/'))

    def run(self):
        while self.queue and not rospy.is_shutdown():
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
                elif cmd == 'B':
                    dist = float(arg)
                    self.move(dist=-dist)
                else:
                    print("[%s] [Turtlebot#run] Error: Invalid command keyword: %s" % (self.name, cmd))
            except ValueError:
                print("[%s] [Turtlebot#run] Error: Invaild argument type: %s (%s)" % (self.name, arg, type(arg).__name__))
           
    def get_odom(self, msg: Odometry):
        *self.pos, theta = self.get_pose(msg)
        d_theta = theta - self.direction_raw
        if d_theta > 5:  # clockwise, passing 0-360 border (ex: 20 deg -> 340 deg)
            d_theta -= TWO_PI
        elif d_theta < -5:  # counterclockwise, passing 0-360 border (ex: 340 deg -> 20 deg)
            d_theta += TWO_PI
        self.direction_raw = theta
        self.direction_acc += d_theta
            
    def get_pose(self, data: Odometry):
        qx, qy, qz, qw = map(data.pose.pose.orientation.__getattribute__, 'xyzw')
        _y = qw * qz + qx * qy
        _x = qy * qy + qz * qz
        theta = atan2(_y+_y, 1-_x-_x)
        if theta < 0:
            theta += TWO_PI
        return data.pose.pose.position.x, data.pose.pose.position.y, theta
        
    def get_dist(self, x, y):
        dx = x - self.pos[0]
        dy = y - self.pos[1]
        return sqrt(dx*dx + dy*dy)

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
                threads.append(Thread(target=turtle_bot.run))
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
    
    def carry_object(self):
        # Bots should be at the points after executing commands from the algorithm.
        threads = []
        active_bots = []
        for turtle_bot in self.bots:
            if turtle_bot is not None and turtle_bot.twist_pub.get_num_connections() > 0:
                threads.append(Thread(target=turtle_bot.carry))
                active_bots.append(turtle_bot)
        for thread in threads:
            thread.start()
        prev = time.time()
        while not rospy.is_shutdown():
            for bot in active_bots:
                bot.condition.acquire()
            try:
                for bot in active_bots:
                    if int(time.time()) - int(prev):
                        prev = time.time()
                    if bot.state != State.CARRY_READY_TO_STICK:
                        break
                else:  # All bots are ready to stick
                    for bot in active_bots:
                        bot.state = State.CARRY_STICK
                    break
            except:
                raise
            finally:
                for bot in active_bots:
                    bot.condition.notify()
                    bot.condition.release()
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
        print("터틀봇에게 전송할 명령을 입력하세요.")
        print("- 입력된 명령들을 실행하려면 `run`을 입력하세요.")
        print("- 물건 운반 작업을 진행하려면 `carry`를 입력하세요.")
        print("- 프로그램을 종료하려면 아무것도 입력하지 않고 Enter를 누르세요.")
        while True:
            temp_cmd = input("turtlebot> ")
            if temp_cmd.lower() == "run":
                controller.execute_all_commands()
                controller.run_all_turtlebots()
                print("[@] Running commands finished.")
            elif temp_cmd.lower() == "carry":
                controller.carry_object()
                print("[@] Carrying task finished.")
            elif not temp_cmd:
                break
            else:
                controller.push_command(temp_cmd)
        print("[@] Program exits.")
    except KeyboardInterrupt:
        rospy.signal_shutdown()
