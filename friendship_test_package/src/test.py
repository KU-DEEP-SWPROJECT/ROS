#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan


DIST_DETECT = [(30 + 10 * math.cos(((i * math.pi / 180.0 - math.pi) ** 3) / (math.pi * math.pi) + math.pi)) ** 1.2 / 150.0 for i in range(360)]


def is_trigger(range_values):
    # type: (list[float]) -> tuple[int, float] | list[tuple[int, float]]
    """Determin it is under a specific circumstance by given distance values.
    
    Since this code is for the test, all infos about this function may be changed at any time.
    - Current the condition is: compare dist values with pre-defined values angle to angle.
    - Current return type is: `tuple[int, float] | list[tuple[int, float]]`
    """
    return list(filter(lambda x: 0.001 < x[1] < DIST_DETECT[x[0]], enumerate(range_values)))
    

def callback(data):
    # type: (LaserScan) -> None
    trig = is_trigger(data.ranges)
    if trig:
        rospy.loginfo("Detected: %s", str(min(trig, key=lambda x: x[1])))

def main():
    rospy.init_node("test_subscriber")
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.loginfo("[%s] Subscribing", rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()

