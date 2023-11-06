#!/usr/bin/env python

import rospy
import math
import enum
from sensor_msgs.msg import LaserScan


DIST_DETECT = [(30 + 10 * math.cos(((i * math.pi / 180.0 - math.pi) ** 3) / (math.pi * math.pi) + math.pi)) ** 1.2 / 150.0 for i in range(360)]


class Direction(enum.Enum):
    FRONT = 1
    FRONTLEFT = 2
    LEFT = 3
    BACKLEFT = 4
    BACK = 5
    BACKRIGHT = 6
    RIGHT = 7
    FRONTRIGHT = 8
    
    @staticmethod
    def get(angle):
        if angle < 0 or angle >= 360:
            return None
        elif angle < 10 or angle >= 350:
            return Direction.FRONT
        elif angle < 60:
            return Direction.FRONTLEFT
        elif angle < 120:
            return Direction.LEFT
        elif angle < 170:
            return Direction.BACKLEFT
        elif angle < 190:
            return Direction.BACK
        elif angle < 240:
            return Direction.BACKRIGHT
        elif angle < 300:
            return Direction.RIGHT
        else:
            return Direction.FRONTRIGHT


def is_trigger(range_values):
    # type: (list[float]) -> list[tuple[int, float]]
    """Determin it is under a specific circumstance by given distance values.
    
    Since this code is for the test, all infos about this function may be changed at any time.
    - Current the condition is: compare dist values with pre-defined values angle to angle.
    - Current return type is: `list[tuple[int, float]]`
    """
    return list(filter(lambda x: 0.001 < x[1] < DIST_DETECT[x[0]], enumerate(range_values)))


def act(directions):
    # type: (set[Direction]) -> int
    direction_bit = 0
    for i in range(1, 9):
        if Direction(i) in directions:
            direction_bit |= 1 << i
    return direction_bit


def callback(data):
    # type: (LaserScan) -> None
    trig = is_trigger(data.ranges)
    if trig:
        rospy.loginfo("Detected direction(s) = %b", act(set(map(Direction.get, trig))))


def main():
    rospy.init_node("test_subscriber")
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.loginfo("[%s] Subscribing", rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()

