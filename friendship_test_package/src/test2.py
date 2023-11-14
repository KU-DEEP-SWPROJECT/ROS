#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import math
import enum
from functools import reduce
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class State(enum.Enum):
    PARKING = 0
    FORWARD = 1
    ROTATE = 2
    FOLLOW = 3
    WAIT_ROTATE = 4
    CARRY = 5
    CARRY_MOVE = 6
    INCIDENT_WHILE_FORWARD = 7
    INCIDENT_WHILE_FOLLOW = 8

