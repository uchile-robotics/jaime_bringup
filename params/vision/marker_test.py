#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker

class Marcador:
    def __init__(self):
        self.pub = rospy.Publisher("/test_marker", Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker()

    def init_marker(self):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.ns = "my_namespace"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.marker)
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("test_marker", anonymous=True)
    marker = Marcador()
    try:
        marker.start()
    except rospy.ROSInterruptException:
        pass