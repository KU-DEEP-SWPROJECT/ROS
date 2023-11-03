#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


DIST_LIMIT = 30


def is_trigger(range_values):
    # type: (list[float]) -> tuple[int, float] | bool
    """Determin it is under a specific circumstance by given distance values.
    
    Since this code is for the test, all infos about this function may be changed at any time.
    - Current the condition is: check if there's any lower distance value(m) than DIST_LIMIT(cm), excluding zero-values.
    - Current return type is: `tuple[int, float] | bool`
    """
    filtered = filter(lambda x: x[1] > 0.001, enumerate(range_values))
    minimum = min(filtered, key=lambda x: x[1])
    if minimum[1] * 100 < DIST_LIMIT:
        return minimum
    return False

def callback(data):
    # type: (LaserScan) -> None
    trig = is_trigger(data.ranges)
    if trig:
        rospy.loginfo("Detected object too close: %s", str(trig))

def main():
    rospy.init_node("test_subscriber")
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.loginfo("[%s] Subscribing", rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()

