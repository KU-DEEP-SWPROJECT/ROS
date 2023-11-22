#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import sys
print(sys.version)

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import enum
import threading
import time
import math
from threading import Thread, Condition
from collections import deque
from geometry_msgs.msg import Twist
from std_msgs.msg import String

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
    RATE_HZ = 10
    BASE_STOP_TIME = 1.0
    BOT_NAME_PREFIX = "turtle_"
    
    _CHECK_RATE_HZ = 50
    
    __ID = 1
    
    # limitations of turtlebot3 burger model
    # MAX_LINEAR_SPEED  = 0.22  # m/s
    # MAX_ANGULAR_SPEED = 2.84  # rad/s
    
    # let's limit them slightly slower
    MAX_LINEAR_SPEED  = 0.2  # m/s
    MAX_ANGULAR_SPEED = 2.5  # rad/s
    
    def __init__(self, robot_name=None):
        self.queue = deque([])
        self.RATE = rospy.Rate(self.RATE_HZ)
        if robot_name is None:
            self.name = self.BOT_NAME_PREFIX + str(TurtleBot.__ID)
            TurtleBot.__ID += 1
        else:
            self.name = self.BOT_NAME_PREFIX + robot_name
        self.publisher = rospy.Publisher('/'+self.name+'/cmd_vel', Twist, queue_size = 20)
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
        
        self.state = State.PARKING
        self.got_incident = False
        self.condition = Condition()
        self.incident_detector = IncidentDetector(self.name)
        self.incident_detector.callback = self.incident_callback
        
        self._CHECK_RATE = rospy.Rate(self._CHECK_RATE_HZ)
        
        self.__terminate = threading.Event()
        self.__task = threading.Thread(target=self.publishing_task)
        self.__task.start()
    
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
            temp_twist = self.twist_msg
            while not rospy.is_shutdown():
                rospy.loginfo("[%s] Incident detected! Wait for removed...", self.name)
                self.twist_msg = Twist()
                self.publisher.publish(self.twist_msg)
                self.condition.wait_for(lambda: not self.got_incident)
                if not self.got_incident:
                    rospy.loginfo("[%s] Incident has removed. Continuing the task...", self.name)
                    self.twist_msg = temp_twist
                    break
        self.condition.notify()
        self.condition.release()
        return time.time() - st
    
    def set_state(self, state):
        self.condition.acquire()
        self.state = state
        self.condition.notify()
        self.condition.release()
    
    def publishing_task(self):
        while not self.__terminate.is_set():
            self.publisher.publish(self.twist_msg)
            self.RATE.sleep()
    
    def _move_linear(self, speed, move_time):
        start_time = time.time()
        self.twist_msg.linear.x = speed
        print("[%s] move time: %.3f / speed: %.3f" % (self.name, move_time, speed))
        while (time.time() - start_time <= move_time):
            start_time += self.check_incident()
            self._CHECK_RATE.sleep()
        self.stop()

    def _move_angular(self, speed, rotate_time):
        start_time = time.time()
        self.twist_msg.angular.z = speed
        print("[%s] rotate time: %.3f / speed: %.3f" % (self.name, rotate_time, speed))
        while (time.time() - start_time <= rotate_time):
            start_time += self.check_incident()
            self._CHECK_RATE.sleep()
        self.stop()

    def stop(self, stop_time=None):
        if stop_time is None:
            stop_time = self.BASE_STOP_TIME
        start_time = time.time()
        self.twist_msg.linear.x = 0
        self.twist_msg.angular.z = 0
        print("[%s] stop time: %.3f" % (self.name, stop_time))
        while (time.time() - start_time <= stop_time):
            self.publisher.publish(self.twist_msg)
            self.RATE.sleep()
        self.set_state(State.PARKING)
    
    def forward(self, dist=None, speed=None, move_time=None):
        # (speed, move_time) has higher priority
        check_param_bit = (dist is None) << 2 | (speed is None) << 1 | (move_time is None)
        if check_param_bit | 0b010 == 0b111:
            # dist and move_time is not given (111, 101) => raise error
            raise ValueError("Lack of arguments.")
        if check_param_bit in {0b011, 0b110}:
            # only dist or move_time is given (011, 110) => use max speed
            speed = self.MAX_LINEAR_SPEED
        if check_param_bit & 0b100 == 0:
            if check_param_bit & 0b001:
                # dist is given, speed is given or MAX_LINEAR_SPEED (001, 011) => calculate time
                move_time = dist / speed
            elif check_param_bit == 0b010:
                # dist and move_time is given and speed is not given (010) => calculate speed
                speed = dist / move_time
        # 100, 000 are OK as itself
        if move_time < 0:
            speed *= -1
            move_time *= -1
        if self.state == State.CARRYING:
            if speed < 0:
                self.set_state(State.CARRY_MOVE_BACKWARD)
            else:
                self.set_state(State.CARRY_MOVE_FORWARD)
        else:
            if speed < 0:
                self.set_state(State.BACKWARD)
            else:
                self.set_state(State.FORWARD)
        self._move_linear(speed, move_time)
    
    def rotate(self, angle=None, speed=None, rotate_time=None):
        # (speed, rotate_time) has higher priority
        check_param_bit = (angle is None) << 2 | (speed is None) << 1 | (rotate_time is None)
        if check_param_bit | 0b010 == 0b111:
            # angle and rotate_time is not given (111, 101) => raise error
            raise ValueError("Lack of arguments.")
        if check_param_bit in {0b011, 0b110}:
            # only angle or rotate_time is given (011, 110) => use max speed
            speed = self.MAX_ANGULAR_SPEED
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
        self.set_state(State.ROTATE)
        self._move_angular(speed, rotate_time)
    
    def follow(self):
        self.set_state(State.FOLLOW)
        pass  # get following robot's movements
    
    def wait_rotate(self):
        self.set_state(State.WAIT_ROTATE)
        pass  # do nothing
    
    def carry(self):
        self.set_state(State.CARRYING)

    def push_command(self, cmd):
        self.queue.append(cmd)
    
    def push_commands(self, command_string):
        for command in command_string.split('/'):
            self.push_command(command)

    def set_queue(self, command_string):
        self.queue = deque(command_string.split('/'))

    def run(self, terminate=True):
        while self.queue:
            q = self.queue.popleft()
            cmd, arg = q[0], q[1:]
            try:
                if cmd == 'F':
                    dist = float(arg)
                    self.forward(dist=dist)
                elif cmd == 'R':
                    angle = float(arg) * math.pi / 180.0
                    self.rotate(angle=angle, speed=0.5)
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
        if terminate:
            self.terminate()
    
    def terminate(self):
        self.__terminate.set()

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
