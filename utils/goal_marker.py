#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

class GoalMarkerPublisher:
    def __init__(self):
        rospy.init_node('goal_box_marker_node')
        
        self.pub = rospy.Publisher('goal_box_marker', Marker, queue_size=10)
        
        self.x_range = [10.0, 12.0]
        self.y_range = [-10.0, -8.0] 
        self.z_range = [4.0, 6.0]
        
        self.rate = rospy.Rate(1)

    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "iss_body"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_area"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        marker.scale.x = 0.02
        marker.color.b = 1.0
        marker.color.a = 1.0

        x_min, x_max = self.x_range
        y_min, y_max = self.y_range
        z_min, z_max = self.z_range

        p = [Point(x, y, z) for x in [x_min, x_max] for y in [y_min, y_max] for z in [z_min, z_max]]

        edges = [
            (0,1), (2,3), (4,5), (6,7),
            (0,2), (1,3), (4,6), (5,7),
            (0,4), (1,5), (2,6), (3,7)
        ]

        for start, end in edges:
            marker.points.append(p[start])
            marker.points.append(p[end])
            
        return marker

    def run(self):
        while not rospy.is_shutdown():
            marker = self.create_marker()
            self.pub.publish(marker)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = GoalMarkerPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass