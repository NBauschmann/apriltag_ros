#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import *


def make_header(frame_id, stamp=None):
    if not stamp:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header
