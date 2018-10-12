#!/usr/bin/env python

import rospy
from pyquaternion import Quaternion
import settings as se
import utils as u
import tf_conversions
import tf
import tf2_ros
from geometry_msgs.msg import *

tags = se.tags

if __name__ == '__main__':

    rospy.init_node('static_transform')
    br = tf2_ros.StaticTransformBroadcaster()
    transforms = []

    for counter, tag in enumerate(tags):

        msg = geometry_msgs.msg.TransformStamped()
        msg.header = u.make_header("map")
        msg.child_frame_id = "Tag" + str(counter)

        msg.transform.translation.x = tag.get_position_wf()[0]
        msg.transform.translation.y = tag.get_position_wf()[1]
        msg.transform.translation.z = tag.get_position_wf()[2]

        msg.transform.rotation.x = tag.get_orientation_wf()[1]
        msg.transform.rotation.y = tag.get_orientation_wf()[2]
        msg.transform.rotation.z = tag.get_orientation_wf()[3]
        msg.transform.rotation.w = tag.get_orientation_wf()[0]
        transforms.append(msg)

    br.sendTransform(transforms)
    rospy.spin()
