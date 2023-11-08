#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import enum
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# DIST_DETECT = [(30 + 10 * math.cos(((i * math.pi / 180.0 - math.pi) ** 3) / (math.pi * math.pi) + math.pi)) ** 1.2 / 150.0 for i in range(360)]


# class Direction(enum.Enum):
#     FRONT = 1
#     FRONTLEFT = 2
#     LEFT = 3
#     BACKLEFT = 4
#     BACK = 5
#     BACKRIGHT = 6
#     RIGHT = 7
#     FRONTRIGHT = 8
    
#     @staticmethod
#     def get(angle):
#         if angle < 0 or angle >= 360:
#             return None
#         elif angle < 10 or angle >= 350:
#             return Direction.FRONT
#         elif angle < 60:
#             return Direction.FRONTLEFT
#         elif angle < 120:
#             return Direction.LEFT
#         elif angle < 170:
#             return Direction.BACKLEFT
#         elif angle < 190:
#             return Direction.BACK
#         elif angle < 240:
#             return Direction.BACKRIGHT
#         elif angle < 300:
#             return Direction.RIGHT
#         else:
#             return Direction.FRONTRIGHT


# def is_trigger(range_values):
#     # type: (list[float]) -> list[tuple[int, float]]
#     """Determin it is under a specific circumstance by given distance values.
#     주어진 거리 값들을 보고, 특정 상황을 만족하는 지를 판단하는 함수입니다.
    
#     Since this code is for the test, all infos about this function may be changed at any time.
#     현재 이 코드는 테스트를 위한 함수이기 때문에, 이 함수는 언제든지 그 기능이 바뀔 수 있습니다.
#     - Current the condition is: compare dist values with pre-defined values angle to angle.
#     - Current return type is: `list[tuple[int, float]]`
#     """
#     return list(filter(lambda x: 0.001 < x[1] < DIST_DETECT[x[0]], enumerate(range_values)))


# def act(directions):
#     # type: (set[Direction]) -> int
#     direction_bit = 0
#     for i in range(1, 9):
#         if Direction(i) in directions:
#             direction_bit |= 1 << i
#     return direction_bit


VELOCITY = 1.0


def constrain(value, minimum, maximum):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value


class IncidentDetector:
    def __init__(self):
        self._time = rospy.Time()
        self._DETECTING = False
        self._DEBUG_STOP = False
        self.prev_ranges = [0.0] * 360
        self.prev_scan_time = 0
    
    def expect_next_data(self, direction, dist, step):
        # type: (float, int, float) -> tuple[float, float]
        """Calculate the value that is expected to be about same point after going `step` distance.
        같은 지점에 대해 거리와 방향의 정보가 `step`만큼 이동 후 어떻게 바뀔 지를 추정하는 함수."""
        cos_direction = math.cos(direction * math.pi / 180.0)
        new_dist = math.sqrt(dist*dist + step*step - 2*dist*step*cos_direction)
        new_direction = math.acos(constrain((dist*cos_direction - step) / new_dist, -1, 1)) * 180 / math.pi
        if direction > 180:
            new_direction = 360 - new_direction
        return new_direction, new_dist

    def expect_new_datas(self, ranges, step):
        # type: (list[float], float) -> list[float]
        """Calculate multiple values with `expect_next_data`.
        `expect_next_data`의 다중 변수 버전.
        
        Not just calculating, but also modifing angles' type onto int, for easily woking with those values later.
        단순히 하나하나 계산하는 것뿐만 아니라, 방향 값을 정수로 적절히 변환하여 이후 이 값들을 이용하는 작업에 용이하게 함."""
        
        rval = [0.0 for _ in range(360)]
        temp = [[] for _ in range(360)]
        for angle, dist in enumerate(ranges):
            new_data = self.expect_next_data(angle, dist, step)
            temp[int(new_data[0])].append(new_data)
        
        # 원하는 방향에서 위에서 구한 값들 중 가장 가까운 방향 양쪽으로 2개 구해서 선형추정
        # 각도 탐색에 boundary 설정
        BOUNDARY = 3
        for angle in range(360):
            max_data = (-999, 0)
            min_data = (999, 0)
            for i in range(angle, angle + BOUNDARY):
                if temp[i%360]:
                    max_data = min(temp[i%360])
                    break
            for i in range(angle, angle - BOUNDARY, -1):
                if temp[i%360]:
                    min_data = max(temp[i%360])
                    break
            if max_data[0] == -999 or min_data[0] == 999:
                continue
            rval[angle] = (angle - min_data[0]) * (max_data[1] - min_data[1]) / (min_data[1] - min_data[0]) + min_data[1]
        return rval
    
    def run(self, laser_data):
        # type: (LaserScan) -> None
        """Callback for scan topic."""
        if not self._DETECTING:
            self._DETECTING = True
            self.prev_scan_time = laser_data.header.stamp.to_sec()
            self.prev_ranges = laser_data.ranges
            return
        if self._DEBUG_STOP:
            return
        
        ntime = laser_data.header.stamp.to_sec()
        dtime = ntime - self.prev_scan_time
        step = VELOCITY * dtime
        
        expected_range = self.expect_new_datas(self.prev_ranges, step)
        actual_range = laser_data.ranges
        detected = []
        for angle in range(360):
            if actual_range[angle] < 0.001 or expected_range[angle] < 0.001 or self.prev_ranges[angle] < 0.001:
                continue
            if self.prev_ranges[angle] < actual_range[angle]:
                continue
            if actual_range[angle] < expected_range[angle] - constrain(expected_range[angle] / 1.5, 0.15, 0.5):
                detected.append((angle, actual_range[angle], expected_range[angle]))
        
        rospy.loginfo("Incident detected! : %s", str(detected))
        
        if not self._DEBUG_STOP:
            print('prev_time: %f / ntime: %f / dtime: %f' % (self.prev_scan_time, ntime, dtime))
            print("step: %f" % step)
            print(list(map(lambda x: round(x, 5), self.prev_ranges)))
            print(list(map(lambda x: round(x, 5), expected_range)))
            print(list(map(lambda x: round(x, 5), actual_range)))
            self._DEBUG_STOP = True
        
        self.prev_scan_time = ntime
        self.prev_ranges = actual_range


# def callback(data):
#     # type: (LaserScan) -> None
#     trig = is_trigger(data.ranges)
#     direction_set = set(map(lambda x: Direction.get(x[0]), trig))
#     rospy.loginfo("Detected direction bit = %09s", bin(act(direction_set))[2:])


def main():
    rospy.init_node("test_subscriber")
    incident_detector = IncidentDetector()
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel = Twist()
    vel.linear.x = VELOCITY
    while pub.get_num_connections() == 0:
        pass
    pub.publish(vel)
    
    rospy.Subscriber("scan", LaserScan, incident_detector.run)
    rospy.loginfo("Initiating!")
    try:
        rospy.spin()
    except:
        pass
    finally:
        pub.publish(Twist())

if __name__ == '__main__':
    main()

