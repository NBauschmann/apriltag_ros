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

# add more tags or change parameters in settings.py


class TagMonitor(object):
    def __init__(self, pub, br):
        self.__pub = pub
        self.__br = br

    def callback(self, msg):

        all_measurements = []

        transforms = []

        # get data from topic /tag_detections
        for tag in msg.detections:

            tag_id = int(tag.id[0])
            x = tag.pose.pose.pose.position.x
            y = tag.pose.pose.pose.position.y
            z = tag.pose.pose.pose.position.z
            qx = tag.pose.pose.pose.orientation.x
            qy = tag.pose.pose.pose.orientation.y
            qz = tag.pose.pose.pose.orientation.z
            qw = tag.pose.pose.pose.orientation.w

            # transform pose into world frame
            dist_cam_tag = np.array([[x], [y], [z]])
            quat_cam_tag = Quaternion(qw, qx, qy, qz)
            S_ct = quat_cam_tag.rotation_matrix
            print 'S_ct ' + str(S_ct)
            print 'dist_ct_c' + str(dist_cam_tag)
            dist_ct_t = np.dot(S_ct, dist_cam_tag)
            print 'dist_ct_t' + str(dist_ct_t)
            S_wt = Tag_list[tag_id].get_orientation_wf().rotation_matrix
            print 'S_wt ' + str(S_wt)
            dist_ct_w = np.dot(S_wt, dist_ct_t)
            print 'dist_ct_w ' + str(dist_ct_w)
            #position_cam_wf = Tag_list[tag_id].convert_location_to_wf(quat_cam_tag, dist_cam_tag)
            orientation_cam_wf = Tag_list[tag_id].convert_orientation_to_wf(quat_cam_tag)
            position_cam_wf = dist_ct_w
            # test
            #orientation = np.dot(R_wt, R_ct)
            #print 'R_wt * R_ct' + str(orientation)
            #print 'resulting quat ' + str(Quaternion(matrix=orientation))
            #print 'conjugated quat ' + str(Quaternion(matrix=orientation).conjugate)
            # publish "measured by this tag" camera pose (in world frame) as transform
            if se.use_rviz:

                msg = geometry_msgs.msg.TransformStamped()
                msg.header = u.make_header("map")
                msg.child_frame_id = "Pose_Tag" + str(tag_id)
                msg.transform.translation.x = position_cam_wf[0]
                msg.transform.translation.y = position_cam_wf[1]
                msg.transform.translation.z = position_cam_wf[2]
                #msg.transform.rotation.x = orientation_cam_wf[1]
                #msg.transform.rotation.y = orientation_cam_wf[2]
                #msg.transform.rotation.z = orientation_cam_wf[3]
                #msg.transform.rotation.w = orientation_cam_wf[0]
                transforms.append(msg)

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
        if se.use_rviz:
            self.__br.sendTransform(transforms)

        # publish calculated poses
        hps = HippoPoses()
        measurements_poses = []

        for measurement in all_measurements:

            hp = HippoPose()

            hp.id = measurement[7]
            hp.pose.position.x = measurement[0]
            hp.pose.position.y = measurement[1]
            hp.pose.position.z = measurement[2]
            hp.pose.orientation.x = measurement[4]
            hp.pose.orientation.y = measurement[5]
            hp.pose.orientation.z = measurement[6]
            hp.pose.orientation.w = measurement[3]

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
