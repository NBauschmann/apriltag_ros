#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import tag_class as tc
import geometry_msgs
import time as t


from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
from aprilslam.msg import Apriltags
from aprilslam.msg import HippoPose
from aprilslam.msg import HippoPoses

from os import path

"""
Before measurement: change path to measurement file below to correct one.

If measurement before didn't finish (or has been stopped before reaching the end) change last value in current_position.txt back to 0.

"""


pos_file_rel_path = path.relpath('/home/hippoc/src/Gantry_Nathalie/current_position.txt', '/home/hippoc/catkin_ws/src/aprilslam/aprilslam/scripts')
meas_file_rel_path = path.relpath('/home/hippoc/src/Gantry_Nathalie/Measurements/d100_full_tank_1.txt', '/home/hippoc/catkin_ws/src/aprilslam/aprilslam/scripts')


class TagMonitor(object):
    def __init__(self, pub):
        self.__pub = pub
        self.__data = []

    def callback(self, msg):

        all_measurements = []

        # get information from published message apriltags
        for tag in msg.apriltags:

            measurement = []

            measurement.append(tag.id)
            measurement.append(np.round(tag.pose.position.x, decimals=5))
            # measurement.append(tag.pose.position.x) # not rounded
            measurement.append(np.round(tag.pose.position.y, decimals=5))
            measurement.append(np.round(tag.pose.position.z, decimals=5))
            measurement.append(np.round(tag.pose.orientation.x, decimals=5))
            measurement.append(np.round(tag.pose.orientation.y, decimals=5))
            measurement.append(np.round(tag.pose.orientation.z, decimals=5))
            measurement.append(np.round(tag.pose.orientation.w, decimals=5))

            all_measurements.append(measurement)

        self.__data = all_measurements

        # publish measurements (not really necessary atm)
        hps = HippoPoses()
        measurements_poses = []

        for measurement in all_measurements:
            hp = HippoPose()

            hp.id = measurement[0]
            hp.pose.position.x = measurement[1]
            hp.pose.position.y = measurement[2]
            hp.pose.position.z = measurement[3]
            hp.pose.orientation.x = measurement[4]
            hp.pose.orientation.y = measurement[5]
            hp.pose.orientation.z = measurement[6]
            hp.pose.orientation.w = measurement[7]

            measurements_poses.append(hp)

        hps.poses = measurements_poses

        self.__pub.publish(hps)

        # print('num tags: ' + str(len(msg.apriltags)))

    def return_data(self):
        return self.__data


def main():

    old_position = ([3, 2, 1])  # any position that's not the starting or ending position
    counter = 0

    rospy.init_node('measurement_node')

    pub = rospy.Publisher('hippo_poses', HippoPoses, queue_size=10)
    monitor = TagMonitor(pub)
    rospy.Subscriber("/usb_cam/apriltags", Apriltags, monitor.callback)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():

        with open(pos_file_rel_path) as pos_file:  # pos_file: 'wp_x, wp_y, wp_z, num_wp'
            data = pos_file.read().split(' ')

        # don't try to convert to float if data is empty to avoid crashing:
        if not data[0]:
            t.sleep(0.5)
            continue

        if data[0]:
            data_float = [float(i) for i in data]
            position = ([data_float[0], data_float[1], data_float[2]])

        if int(data[4]) == 1:  # to avoid starting measurement before gantry arrived at starting position
            # and to avoid having to reset position file, gantry_control sets this back to 0 after measurement

            if position == old_position:
                counter = 0

            else:
                print('Start measurement for ' + str(position))
                while counter < 5:   # number of saved measurements
                    t.sleep(0.25)  # wait some time between measurements, otherwise they're all the same
                    with open(meas_file_rel_path, 'a') as meas_file:

                        # way point data - structure 'wp_x, wp_y, wp_z, num_wp'
                        str_base_data = str(position[0]) + ' ' + str(position[1]) + ' ' + str(position[2]) + ' ' + str(data[3]) + ' ' + str(counter) + ' '  # data[3] is num_wp
                        # actual measurement data
                        all_measurements = monitor.return_data()

                        # print all_measurements
                        str_tag = ''

                        for i in range(len(all_measurements)):
                            str_tag = str_tag + ' '.join(map(str, all_measurements[i])) + ' '

                        meas_file.write(str_base_data + str_tag + '\n')
                    counter += 1

                old_position = position

        rate.sleep()

    # rospy.spin()   # not needed if using while not rospy.is_shutdown and rate.sleep()


if __name__ == '__main__':

    main()
