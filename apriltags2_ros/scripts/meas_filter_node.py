#!/usr/bin/env python

import rospy
import numpy as np
import numpy.matlib as npm
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


Tag_list = se.tags
# add more tags or change parameters in settings.py



def average_quaternions(Q):
    # Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
    # The quaternions are arranged as (w,x,y,z), with w being the scalar
    # The result will be the average quaternion of the input. Note that the signs
    # of the output quaternion can be reversed, since q and -q describe the same orientation

    # from: https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py

    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4, 4))

    for i in range(0, M):
        q = Q[i, :]
        # multiply q with its transposed version q' and add A
        A = np.outer(q, q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:, 0].A1)


class TagMonitor(object):
    def __init__(self, pub, br):
        self.__pub = pub
        self.__br = br
        self.__last_quaternions = [None] * np.size(Tag_list)

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
            quat_meas = Quaternion(qw, qx, qy, qz)

            # filtering measured orientation using moving average
            if self.__last_quaternions[tag_id] is not None:

                quat_last = self.__last_quaternions[tag_id]
                # calculating distance between last measurement and new one
                dist_quat = Quaternion.absolute_distance(quat_meas, quat_last)
                print str(dist_quat)

                # checking for outliers todo
                if dist_quat > 2:
                    k = 0.01
                    # taking the naive average by averaging each element todo
                    mov_av_qw = (1 - k) * quat_last[0] + k * quat_meas[0]
                    mov_av_qx = (1 - k) * quat_last[1] + k * quat_meas[1]
                    mov_av_qy = (1 - k) * quat_last[2] + k * quat_meas[2]
                    mov_av_qz = (1 - k) * quat_last[3] + k * quat_meas[3]

                else:
                    k = 0.2
                    # taking the naive average by averaging each element todo
                    mov_av_qw = (1 - k) * quat_last[0] + k * quat_meas[0]
                    mov_av_qx = (1 - k) * quat_last[1] + k * quat_meas[1]
                    mov_av_qy = (1 - k) * quat_last[2] + k * quat_meas[2]
                    mov_av_qz = (1 - k) * quat_last[3] + k * quat_meas[3]

                quat_av = Quaternion(mov_av_qw, mov_av_qx, mov_av_qy, mov_av_qz).normalised

            else:
                quat_av = quat_meas

            # saving measured quaternion in the corresponding list entry for this tag for filtering
            self.__last_quaternions[tag_id] = quat_av

            # transform pose into world frame
            dist_cam_tag = np.array([[x], [y], [z]])

            quat_cam_tag_meas = quat_meas  # using unfiltered orientation
            quat_cam_tag = quat_av   # using moving average orientation
            # quat_cam_tag = Quaternion(0, 1.0, 0, 0)  # hardcoded actual orientation

            print "Tag: " + str(tag_id)
            print "Measured orientation: " + str(quat_cam_tag_meas)
            print "Filtered orientation: " + str(quat_cam_tag)

            #position_cam_wf_meas = Tag_list[tag_id].convert_location_to_wf(quat_cam_tag_meas, dist_cam_tag)   # using unfiltered orientation
            position_cam_wf = Tag_list[tag_id].convert_location_to_wf(quat_cam_tag, dist_cam_tag)   # using filtered orientation
            orientation_cam_wf = Tag_list[tag_id].convert_orientation_to_wf(quat_cam_tag)


            #print "position using measured orientation: " + str(position_cam_wf_meas)
            #print "position using filtered orientation: " + str(position_cam_wf)

            # print "Umgerechnet: " + str(orientation_cam_wf)
            # print "Umgerechnet S: " + str(orientation_cam_wf.rotation_matrix)

            # publish transforms
            if se.use_rviz:

                # "measured by this tag" camera pose (in world frame)
                msg1 = geometry_msgs.msg.TransformStamped()
                msg1.header = u.make_header("map")
                msg1.child_frame_id = "Pose_Tag" + str(tag_id)
                msg1.transform.translation.x = position_cam_wf[0]
                msg1.transform.translation.y = position_cam_wf[1]
                msg1.transform.translation.z = position_cam_wf[2]
                msg1.transform.rotation.x = orientation_cam_wf[1]
                msg1.transform.rotation.y = orientation_cam_wf[2]
                msg1.transform.rotation.z = orientation_cam_wf[3]
                msg1.transform.rotation.w = orientation_cam_wf[0]
                transforms.append(msg1)

                # Measured tag frame using "filtered" orientation
                msg2 = geometry_msgs.msg.TransformStamped()
                msg2.header = u.make_header("camera")
                msg2.child_frame_id = "Tag_" + str(tag_id)
                msg2.transform.translation.x = dist_cam_tag[0]
                msg2.transform.translation.y = dist_cam_tag[1]
                msg2.transform.translation.z = dist_cam_tag[2]
                msg2.transform.rotation.x = quat_cam_tag[1]
                msg2.transform.rotation.y = quat_cam_tag[2]
                msg2.transform.rotation.z = quat_cam_tag[3]
                msg2.transform.rotation.w = quat_cam_tag[0]
                transforms.append(msg2)

            # saving measurement
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
            #self.__pub_tag_pos.publish(tp)

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
