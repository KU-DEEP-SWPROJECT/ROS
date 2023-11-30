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
from test_move_and_stop import State, TurtleBot, BotController


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty



