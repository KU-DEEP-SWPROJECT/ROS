#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class IncidentDetector:
    INCIDENT_DISTANCE = 0.2  # meters
    
    def __init__(self, scan_prefix=''):
        self.scan_prefix = scan_prefix
        self.callback = self.base_callback
        rospy.Subscriber(scan_prefix + "/scan", LaserScan, self._callback)
    
    def _callback(self, arg):
        self.callback(arg)
    
    def base_callback(self, arg):
        return list(filter(lambda x: 0.01 < arg.ranges[x] < self.INCIDENT_DISTANCE, range(360)))


if __name__ == '__main__':
    rospy.init_node("incident_detector_node")
    pub = rospy.Publisher("test_incident_detect", String, queue_size=10)
    incident_detector = IncidentDetector()
    
    def foo(angles):
        pub.publish(','.join(map(str, incident_detector.base_callback(angles))))
        
    incident_detector.callback = foo
    rospy.spin()
