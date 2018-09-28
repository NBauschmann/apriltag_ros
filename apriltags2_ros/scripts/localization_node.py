#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import tag_class as tc
import settings as se
import geometry_msgs
import time

from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from apriltags2_ros.msg import HippoPose
from apriltags2_ros.msg import HippoPoses


# todo: move this to settings as well
Tag_list = [se.Tag_0, se.Tag_1, se.Tag_2, se.Tag_3, se.Tag_4]
# add more tags in tags_file.py


class TagMonitor(object):
    def __init__(self, pub):
        self.__pub = pub

    def callback(self, msg):

        absolute_position_list = []
        absolute_orientation_list = []
        all_measurements = []

        # get information from published message apriltags
        for tag in msg.detections:

            tag_id = int(tag.id[0])
            dist_cam_tag = np.array([[tag.pose.pose.pose.position.x], [tag.pose.pose.pose.position.y], [tag.pose.pose.pose.position.z]])
            quat_cam_tag = Quaternion(tag.pose.pose.pose.orientation.x, tag.pose.pose.pose.orientation.y, tag.pose.pose.pose.orientation.z, tag.pose.pose.pose.orientation.w)

            absolute_position = Tag_list[tag_id].convert_location_to_absolute(quat_cam_tag, dist_cam_tag)
            absolute_position_list.append(absolute_position)
            absolute_orientation = Tag_list[tag_id].convert_orientation_to_absolute(quat_cam_tag)
            absolute_orientation_list.append(absolute_orientation)

            measurement = []

            measurement.append(absolute_position[0])
            measurement.append(absolute_position[1])
            measurement.append(absolute_position[2])
            measurement.append(absolute_orientation[0])
            measurement.append(absolute_orientation[1])
            measurement.append(absolute_orientation[2])
            measurement.append(absolute_orientation[3])
            measurement.append(tag_id)

            all_measurements.append(measurement)

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

        hps.poses = measurements_poses

        self.__pub.publish(hps)


def main():

    rospy.init_node('localization_node')
    pub = rospy.Publisher('hippo_poses', HippoPoses, queue_size=10)
    monitor = TagMonitor(pub)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, monitor.callback)

    rospy.spin()


if __name__ == '__main__':
    main()
