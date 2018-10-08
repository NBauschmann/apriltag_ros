#!/usr/bin/env python

import rospy
from pyquaternion import Quaternion

import settings as se
#import tf_conversions
import tf
import tf2_ros
#import tf2_ros
from geometry_msgs.msg import *


if __name__ == '__main__':
    rospy.init_node('static_transform')
    test = tf2_ros.StaticTransformBroadcaster()
    #print se.Tag_1.get_position_wf()[0], se.Tag_1.get_position_wf()[1], se.Tag_1.get_position_wf()[2]
    #print se.Tag_1.get_orientation_wf()[1], se.Tag_1.get_orientation_wf()[2], se.Tag_1.get_orientation_wf()[3], se.Tag_1.get_orientation_wf()[0]
    #test.sendTransform((se.Tag_1.get_position_wf()[0], se.Tag_1.get_position_wf()[1], se.Tag_1.get_position_wf()[2]),
    #                      (se.Tag_1.get_orientation_wf()[1], se.Tag_1.get_orientation_wf()[2], se.Tag_1.get_orientation_wf()[3], se.Tag_1.get_orientation_wf()[0]),
    #                      rospy.Time.now(),
    #                      "Tag_1", "map")
    msg = geometry_msgs.msg.TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"    # world ?      #global frame: 1 according to std_msgs/Header
    msg.child_frame_id = "Tag1"
    msg.transform.translation.x = se.Tag_1.get_position_wf()[0]
    msg.transform.translation.y = se.Tag_1.get_position_wf()[1]
    msg.transform.translation.z = se.Tag_1.get_position_wf()[2]
    msg.transform.rotation.x = se.Tag_1.get_orientation_wf()[1]
    msg.transform.rotation.y = se.Tag_1.get_orientation_wf()[2]
    msg.transform.rotation.z = se.Tag_1.get_orientation_wf()[3]
    msg.transform.rotation.w = se.Tag_1.get_orientation_wf()[0]

    test.sendTransform(msg)

    rospy.spin()


