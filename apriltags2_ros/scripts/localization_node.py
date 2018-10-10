#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import tag_class as tc
import settings as se
import utils as u
from geometry_msgs.msg import *
import std_msgs.msg
import tf2_ros

from pyquaternion import Quaternion
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from apriltags2_ros.msg import HippoPose
from apriltags2_ros.msg import HippoPoses


# todo: move this to settings as well
Tag_list = se.tags

# add more tags in tags_file.py


class TagMonitor(object):
    def __init__(self, pub, br):
        self.__pub = pub
        self.__br = br

    def callback(self, msg):

        position_cam_wf_list = []
        orientation_cam_wf_list = []
        all_measurements = []

        transforms = []

        # get information from published message apriltags
        for tag in msg.detections:

            # get data from tag_detections
            tag_id = int(tag.id[0])
            x = tag.pose.pose.pose.position.x
            y = tag.pose.pose.pose.position.y
            z = tag.pose.pose.pose.position.z
            qx = tag.pose.pose.pose.orientation.x
            qy = tag.pose.pose.pose.orientation.y
            qz = tag.pose.pose.pose.orientation.z
            qw = tag.pose.pose.pose.orientation.w

            #print "position" + str(x) + str(y) + str(z)
            #print "orientation" + str(qx) + ' ' + str(qy) + ' ' +str(qz) + ' ' + str(qw)
            """
            # publish camera pose (in tag frame) for rviz
            # doesn't work yet
            msg = geometry_msgs.msg.TransformStamped()
            msg.header = u.make_header(("Tag" + str(tag_id)))
            msg.child_frame_id = "Pose_Tag" + str(tag_id)
            msg.transform.translation.x = -x
            msg.transform.translation.y = -y
            msg.transform.translation.z = -z

            # the right format seems to be: x, y, z, w
            msg.transform.rotation.x = qy
            msg.transform.rotation.y = qz
            msg.transform.rotation.z = qw
            msg.transform.rotation.w = qx
            transforms.append(msg)
            """
            # transform pose into world frame
            # todo: DOESNT WORK YET
            dist_cam_tag = np.array([[x], [y], [z]])

            quat_cam_tag = Quaternion(qx, qy, qz, qw)
            #print dist_cam_tag
            #print quat_cam_tag

            position_cam_wf = Tag_list[tag_id].convert_location_to_wf(quat_cam_tag, dist_cam_tag)
            # print position_cam_wf
            position_cam_wf_list.append(position_cam_wf)
            orientation_cam_wf = Tag_list[tag_id].convert_orientation_to_wf(quat_cam_tag)
            # print orientation_cam_wf
            orientation_cam_wf_list.append(orientation_cam_wf)

            measurement = []
            measurement.append(position_cam_wf[0])
            measurement.append(position_cam_wf[1])
            measurement.append(position_cam_wf[2])
            measurement.append(orientation_cam_wf[0])
            measurement.append(orientation_cam_wf[1])
            measurement.append(orientation_cam_wf[2])
            measurement.append(orientation_cam_wf[3])
            measurement.append(tag_id)

            all_measurements.append(measurement)

        # publish transforms
        if len(transforms) < 1:
            self.__br.sendTransform(transforms)

        elif len(transforms) == 1:
            self.__br.sendTransform(transforms[0])

        # publish calculated poses
        hps = HippoPoses()
        measurements_poses = []

        for measurement in all_measurements:

            pub_position = np.array([[measurement[0]], [measurement[1]], [measurement[2]]])
            pub_orientation = np.array([[measurement[3]], [measurement[4]], [measurement[5]], [measurement[6]]])
            pub_id = measurement[7]

            hp = HippoPose()

            hp.id = pub_id
            hp.pose.position.x = pub_position[0]
            hp.pose.position.y = pub_position[1]
            hp.pose.position.z = pub_position[2]
            hp.pose.orientation.x = pub_orientation[0]
            hp.pose.orientation.y = pub_orientation[1]
            hp.pose.orientation.z = pub_orientation[2]
            hp.pose.orientation.w = pub_orientation[3]

            measurements_poses.append(hp)

        hps.header = std_msgs.msg.Header()
        hps.header.stamp = rospy.Time.now()
        hps.header.frame_id = "map"
        hps.poses = measurements_poses

        self.__pub.publish(hps)


def main():

    rospy.init_node('localization_node')
    pub = rospy.Publisher('hippo_poses', HippoPoses, queue_size=10)
    br = tf2_ros.TransformBroadcaster()
    monitor = TagMonitor(pub, br)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, monitor.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
