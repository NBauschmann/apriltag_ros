#!/usr/bin/env python

import rospy
from std_msgs.msg import Header


def make_header(frame_id, stamp=None):
    if not stamp:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header
