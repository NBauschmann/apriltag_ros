import rospy
import numpy as np
import math

from pyquaternion import Quaternion
#from aprilslam.msg import Apriltags


class Tag(object):

    def __init__(self, tag_id, position_wf, orientation_wf):
        self.__id = tag_id
        self.__position_wf = position_wf
        self.__orientation_wf = orientation_wf

    def get_id(self):
        return self.__id

    def get_position_wf(self):
        return self.__position_wf

    def get_orientation_wf(self):
        return self.__orientation_wf

    def convert_location_to_wf(self, quat_cam_tag_x, dist_cam_tag_x):
        #dist_cam_tag_tf = quat_cam_tag_x.rotate(dist_cam_tag_x)
        #dist_cam_tag_wf = self.__orientation_wf.rotate(dist_cam_tag_tf)

        #position_cam_wf = np.subtract(self.__position_wf, dist_cam_tag_wf)


        position_cam_wf =
        return position_cam_wf
    
    def convert_orientation_to_wf(self, quat_cam_tag_x):
        orientation_cam_wf = self.__orientation_wf * quat_cam_tag_x  # should be right quaternion now
        # pyquaternion says: q3 = q1 * q2 for q1 first, then q2
        # it is: q3 = q2 * q1 for q1 first, then q2
        return orientation_cam_wf


"""
dist_cam_tag_tf = quat_cam_tag.rotate(dist_cam_tag)
dist_cam_tag_wf = tag_1_orientation.rotate(dist_cam_tag_tf)

absolute_position = np.subtract(tag_1_pose, dist_cam_tag_wf)
"""
