#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import enum
import time
from incident_detect import IncidentDetector
from std_msgs.msg import String
from threading import Thread, Condition


class State(enum.Enum):
    PARKING = 0
    FORWARD = 1
    ROTATE = 2
    FOLLOW = 3
    WAIT_ROTATE = 4
    CARRYING = 5
    CARRY_MOVE_FORWARD = 6
    CARRY_MOVE_BACKWARD = 7


class DummyRobot:
    def __init__(self, name):
        self.state = State.PARKING
        self.linear_v = 0.0
        self.angular_v = 0.0
        self.rate = rospy.Rate(20)
        self.got_incident = False
        self.condition = Condition()
        self.name = name
        self.incident_detector = IncidentDetector()
        self.incident_detector.callback = lambda arg: self.incident_callback(self.incident_detector.base_callback(arg))
        rospy.on_shutdown(self.stop)
    
    def incident_callback(self, angles):
        fronts = filter(lambda angle: angle < 30 or angle > 330, angles)
        self.condition.acquire()
        if self.state == State.PARKING:
            return
        elif self.state == State.FORWARD:
            if fronts and not self.got_incident:
                self.got_incident = True  # STOP
            elif not fronts:
                self.got_incident = False
        elif self.state == State.ROTATE:
            return
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
            return
        elif self.state == State.CARRY_MOVE_FORWARD:
            return
        elif self.state == State.CARRY_MOVE_BACKWARD:
            if filter(lambda angle: 150 < angle < 210, angles) and not self.got_incident:
                self.got_incident = True  # STOP
        self.condition.notify()
        self.condition.release()
    
    def check_incident(self):
        st = time.time()
        self.condition.acquire()
        if self.got_incident:
            rospy.loginfo("Incident detected! Wait for removed...")
            self.condition.wait_for(lambda: not self.got_incident)
            rospy.loginfo("Incident has removed. Continuing the task...")
        self.condition.release()
        return time.time() - st
    
    def set_state(self, state):
        self.condition.acquire()
        self.state = state
        self.condition.notify()
        self.condition.release()
    
    def _move_linear(self, speed, mv_time):
        self.linear_v = speed
        start_time = time.time()
        while (time.time() - start_time <= mv_time):
            rospy.loginfo("[Moving] linear_speed = %.3f / moving_time = %.3f", self.linear_v, time.time() - start_time)
            self.rate.sleep()
            mv_time += self.check_incident()
    
    def _move_angular(self, speed, mv_time):
        self.angular_v = speed
        start_time = time.time()
        while (time.time() - start_time <= mv_time):
            rospy.loginfo("[Moving] angular_speed = %.3f / moving_time = %.3f", self.angular_v, time.time() - start_time)
            self.rate.sleep()
            mv_time += self.check_incident()
    
    def forward(self, speed, time):
        if self.state == State.CARRYING:
            if speed < 0:
                self.set_state(State.CARRY_MOVE_BACKWARD)
            else:
                self.set_state(State.CARRY_MOVE_FORWARD)
        else:
            self.set_state(State.FORWARD)
        self._move_linear(speed, time)
    
    def rotate(self, speed, time):
        self.set_state(State.ROTATE)
        self._move_angular(speed, time)
    
    def follow(self):
        self.set_state(State.FOLLOW)
        pass  # get following robot's movements
    
    def wait_rotate(self):
        self.set_state(State.WAIT_ROTATE)
        pass  # do nothing
    
    def carry(self):
        self.set_state(State.CARRYING)
    
    def stop(self):
        self.linear_v = 0.0
        self.angular_v = 0.0
        # rospy.loginfo("[Terminate] Stopping turtlebot...")
        # cmd_vel_publisher.publish(Twist())
        # rospy.sleep(1)
        pass


if __name__ == '__main__':
    rospy.init_node('testbot')
    robot = DummyRobot('friendship')
    
    def move():
        robot.forward(0.5, 10)
        robot.rotate(0.5, 10)
        robot.forward(0.5, 10)

    try:
        move()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown()
