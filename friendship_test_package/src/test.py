#!/usr/bin/env python

import rospy
import math
import enum
from sensor_msgs.msg import LaserScan


DIST_DETECT = [(30 + 10 * math.cos(((i * math.pi / 180.0 - math.pi) ** 3) / (math.pi * math.pi) + math.pi)) ** 1.2 / 150.0 for i in range(360)]


class Direction(enum.Enum):
    FRONT = 1
    FRONTRIGHT = 2
    RIGHT = 3
    BACKRIGHT = 4
    BACK = 5
    BACKLEFT = 6
    LEFT = 7
    FRONTLEFT = 8


def is_trigger(range_values):
    # type: (list[float]) -> list[tuple[int, float]]
    """Determin it is under a specific circumstance by given distance values.
    
    Since this code is for the test, all infos about this function may be changed at any time.
    - Current the condition is: compare dist values with pre-defined values angle to angle.
    - Current return type is: `list[tuple[int, float]]`
    """
    return list(filter(lambda x: 0.001 < x[1] < DIST_DETECT[x[0]], enumerate(range_values)))


def classify(angle):
    # type: (int) -> Direction | None
    """Classify the direction. The angle is measured in degrees.
    
    Returns `None` if the given angle is out of range. (<0 or >=360)
    """
    if angle < 0 or angle >= 360:
        return None
    elif angle < 10 or angle >= 350:
        return Direction.FRONT
    elif angle < 60:
        return Direction.FRONTRIGHT
    elif angle < 120:
        return Direction.RIGHT
    elif angle < 170:
        return Direction.BACKRIGHT
    elif angle < 190:
        return Direction.BACK
    elif angle < 240:
        return Direction.BACKLEFT
    elif angle < 300:
        return Direction.LEFT
    else:
        return Direction.FRONTLEFT
    

def callback(data):
    # type: (LaserScan) -> None
    trig = is_trigger(data.ranges)
    if trig:
        rospy.loginfo("Detected direction(s) = %s", tuple(set(map(lambda x: classify(x[0]).name, trig))))

def main():
    rospy.init_node("test_subscriber")
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.loginfo("[%s] Subscribing", rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()

