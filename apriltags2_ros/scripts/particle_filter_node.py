#!/usr/bin/env python

import rospy
import numpy as np
from numpy import *
import math
from pyquaternion import Quaternion
import random
from scipy.stats import multivariate_normal

from os import path
import tag_class as tc
import settings as se

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from apriltags2_ros.msg import HippoPose
from apriltags2_ros.msg import HippoPoses

"""
Particle Filter 2

This particle filter uses absolute positions in world frame instead of the 
actual measurements (distance and orientation from camera to tag).

"""

# What kind of measurement?
"""
set meas_type: 
meas_type = 1: 3D measurement with known camera_orientation (set below!) NOT WORKING ATM
meas_type = 2: 3D measurement with unknown camera_orientation
                -> 6D measurement, using measured quaternion to transform measured position in world frame
"""
meas_type = 2

# set this if meas_type = 1
cam_orientation = Quaternion(0.5, 0.5, 0.5, 0.5)  # quaternion for camera facing wall with big windows

# number of measured variables per measurement:
if meas_type == 2:
    numV = 4  # tag_id, x, y, z
else:
    numV = 4  # tag_id, x, y, z

# number of particles used
numP = 200



# tags from tags_file, add more there
tags = [se.Tag_0]

fig = plt.figure()


def random_quaternion():
    """returns uniformly-distributed random unit quaternion for sampling orientations"""
    s = random.random()
    sigma1 = sqrt(1 - s)
    sigma2 = sqrt(s)
    theta1 = 2 * math.pi * random.random()
    theta2 = 2 * math.pi * random.random()
    w = math.cos(theta2) * sigma2
    x = math.sin(theta1) * sigma1
    y = math.cos(theta1) * sigma1
    z = math.sin(theta2) * sigma2
    return Quaternion(w, x, y, z)


class Boat(object):
    """used for both the actual boat and the particles"""

    def __init__(self):
        """initialize boat/particle, will have (uniformly-distributed) random position and orientation for now"""
        self.__x = random.random() * tank_size_x
        self.__y = random.random() * tank_size_y
        self.__z = random.random() * tank_size_z

        self.__orientation = random_quaternion()

        self.__sense_noise = 0
        self.__move_noise = 0
        self.__turn_noise = 0

    def set(self, x, y, z, orientation):
        """set boat's position and orientation in the tank"""
        if x >= tank_size_x or x < 0:
            raise ValueError('x coordinate out of bound')
        if y >= tank_size_y or y < 0:
            raise ValueError('y coordinate out of bound')
        if z >= tank_size_z or z < 0:
            raise ValueError('z coordinate out of bound')

        self.__x = x
        self.__y = y
        self.__z = z

        self.__orientation = orientation

    def get_position(self):
        return self.__x, self.__y, self.__z

    def get_x(self):
        return self.__x

    def get_y(self):
        return self.__y

    def get_z(self):
        return self.__z

    def get_orientation(self):
        return self.__orientation

    def set_noise(self, new_sense_noise, new_move_noise, new_turn_noise):
        """set noise parameters for boat and particles"""
        self.__sense_noise = new_sense_noise
        self.__move_noise = new_move_noise
        self.__turn_noise = new_turn_noise

    def move(self, forward, sideways, upward, turn, noisy=False):
        """
        move boat or particles, if noisy: adding random gaussian noise
        parameter turn needs to be a quaternion!
        """
        # so far boat can't turn, camera coordinate system stays the same

        self.__x += forward
        self.__y += sideways
        self.__z += upward
        self.__orientation = self.__orientation * turn  # no idea whether that works -> try changing order

        # for the future: to have the most realistic behaviour first change orientation, then move forward (in direction of z axis of camera frame)

        # to make sure to stay in tank:
        # works but probably not the best solution (not gaussian anymore)
        if noisy:
            # adding noise to position
            a = random.gauss(0.0, self.__move_noise)
            while (0 > (a + self.__x)) or ((a + self.__x) >= se.tank_size_x):
                a = random.gauss(0.0, self.__move_noise)

            b = random.gauss(0.0, self.__move_noise)
            while (0 > (b + self.__y)) or ((b + self.__y) >= se.tank_size_y):
                b = random.gauss(0.0, self.__move_noise)

            c = random.gauss(0.0, self.__move_noise)
            while (0 > (c + self.__z)) or ((c + self.__z) >= se.tank_size_z):
                c = random.gauss(0.0, self.__move_noise)

            self.__x += a
            self.__y += b
            self.__z += c
            """
            # 'adding' noise to orientation
            # from: A Quaternion-based Unscented Kalman Filter for Orientation Tracking
            # todo: still to be tested!
            w_q = np.array([random.gauss(0.0, self.__turn_noise), random.gauss(0.0, self.__turn_noise), random.gauss(0.0, self.__turn_noise)])
            alpha_w = np.linalg.norm(w_q)
            e_w = w_q / np.linalg.norm(w_q)

            q_noise = Quaternion(math.cos(alpha_w / 2), math.sin(alpha_w / 2) * e_w[0], math.sin(alpha_w / 2) * e_w[1], math.sin(alpha_w / 2) * e_w[2])

            self.__orientation = self.__orientation * q_noise
            """
        x = self.__x
        y = self.__y
        z = self.__z
        orientation = self.__orientation
        res = Boat()
        res.set(x, y, z, orientation)
        res.set_noise(self.__sense_noise, self.__move_noise, self.__turn_noise)
        return res

    def sense(self, tags, noisy=False):
        """

        """
        measured_tags = []

        for i in range(len(tags)):

            if meas_type == 2:  # measurement needs to include quaternions as well!!!!
                if noisy:  # for simulation
                    # orientation not noise yet
                    measurement = [tags[i].get_id(), self.__x + random.gauss(0.0, self.__sense_noise),
                                   self.__y + random.gauss(0.0, self.__sense_noise),
                                   self.__z + random.gauss(0.0, self.__sense_noise), self.__orientation]
                    measured_tags.append(measurement)

                else:  # for measurement_prob
                    measurement = [tags[i].get_id(), self.__x, self.__y, self.__z, self.__orientation]
                    measured_tags.append(measurement)

            else:
                if noisy:  # for simulation
                    measurement = [tags[i].get_id(),
                                   tags[i].get_position_wf()[0] - self.__x + random.gauss(0.0, self.__sense_noise),
                                   tags[i].get_position_wf()[1] - self.__y + random.gauss(0.0, self.__sense_noise),
                                   tags[i].get_position_wf()[2] - self.__z + random.gauss(0.0, self.__sense_noise)]
                    measured_tags.append(measurement)
                else:
                    measurement = [tags[i].get_id(), tags[i].get_position_wf()[0] - self.__x,
                                   tags[i].get_position_wf()[1] - self.__y, tags[i].get_position_wf()[2] - self.__z]
                    measured_tags.append(measurement)

        return measured_tags

    @staticmethod
    def real_sense(meas_data, time_step):  # real data -> always noisy
        """read real meas_data, returning measurements list"""
        # real measurements
        if len(all_meas_data[time_step]) > 2:
            num_seen_tags = len(meas_data[time_step][2])
        else:
            num_seen_tags = 0  # todo: What happens if no seen tags?

        # print ('Waypoint, Number of measurement at waypoint: ' + str(all_meas_data[line][1]))
        # print ('seen tags: ' + str(num_seen_tags))

        measurements = []

        for tag in range(num_seen_tags):

            tag_id = int(all_meas_data[time_step][2][tag][0])

            if meas_type == 1:  # 3d measurement, known camera position
                dist_cam_tag_wf = cam_orientation.rotate(
                    np.array(all_meas_data[time_step][2][tag][1:4]))  # vector cam-tag in world frame

                measurement = [tag_id, dist_cam_tag_wf[0] * 1000, dist_cam_tag_wf[1] * 1000, dist_cam_tag_wf[2] * 1000]

                measurements.append(measurement)

            if meas_type == 2:  # 3d measurement, unknown camera orientation
                orientation_cam_tag = Quaternion(all_meas_data[time_step][2][tag][4:8])

                dist_cam_tag_cf = np.array(all_meas_data[time_step][2][tag][1:4])  # vector cam-tag in camera frame

                measurement = [tag_id, dist_cam_tag_cf[0] * 1000, dist_cam_tag_cf[1] * 1000, dist_cam_tag_cf[2] * 1000,
                               orientation_cam_tag]
                measurements.append(measurement)

        return measurements

    def measurement_prob(self, measurements, tags):
        """calculate measurement probability given state of particle (x, y, z)"""

        seen_tag_ids = []

        for ind in range(len(measurements)):
            seen_tag_ids.append(measurements[ind][0])

        predicted_measurement_all = self.sense(tags, noisy=False)

        # print predicted_measurement_all

        predicted_measurement = []

        for ind in range(len(predicted_measurement_all)):
            for id in seen_tag_ids:
                if predicted_measurement_all[ind][0] == id:
                    predicted_measurement.append(predicted_measurement_all[ind])
        """
        if meas_type == 2:
            q_dist_list = []

            for ind in range(len(predicted_measurement)):
                q_dist = Quaternion.absolute_distance(predicted_measurement[ind][4], measurements[ind][4])
                q_dist_list.append(q_dist)
        """
        # print q_dist_list

        # resizing necessary
        len_meas = len(measurements)

        # numV = number of variables per measurement, defined at the top
        a = np.zeros((len_meas, numV))
        for ind in range(len(measurements)):
            a[ind][0] = measurements[ind][0]
            a[ind][1] = measurements[ind][1]
            a[ind][2] = measurements[ind][2]
            a[ind][3] = measurements[ind][3]
            # if meas_type == 2:
            #    a[ind][4] = 0
        measurements_resized = np.reshape(a, len_meas * numV)

        b = np.zeros((len_meas, numV))
        for ind in range(len(predicted_measurement)):
            b[ind][0] = predicted_measurement[ind][0]
            b[ind][1] = predicted_measurement[ind][1]
            b[ind][2] = predicted_measurement[ind][2]
            b[ind][3] = predicted_measurement[ind][3]
            # if meas_type == 2:
            #   b[ind][4] = q_dist_list[ind]
        predicted_measurement_resized = np.reshape(b, len_meas * numV)

        # covariance matrix (diagonal)
        m = np.zeros((len_meas * numV, len_meas * numV))

        if meas_type == 1:
            for ind in range(len_meas * numV):
                m[ind][ind] = 50.0 * 50.0

        if meas_type == 2:
            for ind in range(len_meas * numV):
                m[ind][ind] = 50.0 * 50.0
            """
            for x in range(len_meas):
                for ind in range(numV - 1):
                    m[ind + numV*x][ind + numV*x] = 50.0*50.0
                m[(numV - 1) + numV*x][(numV - 1) + numV*x] = 0.01*0.01
            """
        cov_matrix = m

        weight = multivariate_normal.pdf(predicted_measurement_resized, mean=measurements_resized, cov=cov_matrix)

        # print weight
        return weight

    @staticmethod
    def read_data(meas_file_rel_path):

        with open(meas_file_rel_path, 'r') as meas_file:
            load_description = True
            load_grid_settings = False
            load_meas_data = False
            all_meas_data = []
            # every_wp_list = []  # includes duplicates of wps because of several measurements per wp
            # wp_list = []  # doesn't include duplicates of wps -> better for plotting

            # num_meas_per_wp = 5  # change this if more or less than 5 measurements per waypoint saved.
            plotdata_mat_list = []
            previous_meas = []  # previous measured tags to kick out duplicate measurements because no tag was detected

            for i, line in enumerate(meas_file):

                if line == '### begin grid settings\n':
                    # print('griddata found')
                    load_description = False
                    load_grid_settings = True
                    load_meas_data = False
                    continue
                elif line == '### begin measurement data\n':
                    load_description = False
                    load_grid_settings = False
                    load_meas_data = True
                    # print('Measurement data found')
                    continue
                if load_description:
                    # print('file description')
                    print(line)

                if load_grid_settings and not load_meas_data:

                    grid_settings = map(float, line[:-2].split(' '))
                    x0 = [grid_settings[0] + offset_camera[0], grid_settings[1] + offset_camera[1],
                          grid_settings[2] + offset_camera[2]]
                    xn = [grid_settings[3] + offset_camera[0], grid_settings[4] + offset_camera[1],
                          grid_settings[5] + offset_camera[2]]
                    grid_dxdyda = [grid_settings[6], grid_settings[7], grid_settings[8]]
                    timemeas = grid_settings[9]

                    data_shape_file = []
                    for i in range(3):  # range(num_dof)
                        try:
                            shapei = int((xn[i] - x0[i]) / grid_dxdyda[i] + 1)
                        except ZeroDivisionError:
                            shapei = 1
                        data_shape_file.append(shapei)
                    # old: data_shape_file = [int((xn[0]-x0[0]) / grid_dxdyda[0] + 1), int((xn[1]-x0[1]) / grid_dxdyda[1] + 1), int((xn[2]-x0[2]) / grid_dxdyda[2] + 1)]
                    print('data shape  = ' + str(data_shape_file))

                    # print out
                    # print('filename = ' + meas_data_filename)
                    print('num_of_gridpoints = ' + str(data_shape_file[0] * data_shape_file[1]))
                    print('x0 = ' + str(x0))
                    print('xn = ' + str(xn))
                    print('grid_shape = ' + str(data_shape_file))
                    print('steps_dxdyda = ' + str(grid_dxdyda))
                    print('timemeas = ' + str(timemeas))

                    startx = x0[0]
                    endx = xn[0]
                    stepx = data_shape_file[0]

                    starty = x0[1]
                    endy = xn[1]
                    stepy = data_shape_file[1]

                    startz = x0[2]
                    endz = xn[2]
                    stepz = data_shape_file[2]

                    xpos = np.linspace(startx, endx, stepx)
                    ypos = np.linspace(starty, endy, stepy)
                    zpos = np.linspace(startz, endz, stepz)

                    wp_matx, wp_maty, wp_matz = np.meshgrid(xpos, ypos, zpos)

                    # print(xpos)
                    # print wp_matx
                    wp_vecx = np.reshape(wp_matx, (len(xpos) * len(ypos) * len(zpos), 1))
                    wp_vecy = np.reshape(wp_maty, (len(ypos) * len(zpos) * len(xpos), 1))
                    wp_vecz = np.reshape(wp_matz, (len(zpos) * len(xpos) * len(ypos), 1))

                    wp_mat = np.append(wp_vecx, np.append(wp_vecy, wp_vecz, axis=1), axis=1)
                    print wp_mat

                if load_meas_data and not load_grid_settings:

                    meas_data_line = map(float, line[:-2].split(' '))

                    meas_data_line_list = []

                    # reading waypoint data
                    meas_data_line_list.append(
                        [meas_data_line[0] + offset_camera[0], meas_data_line[1] + offset_camera[1],
                         meas_data_line[2] + offset_camera[2]])  # wp_x, wp_y, wp_z
                    meas_data_line_list.append(
                        [int(meas_data_line[3]), int(meas_data_line[4])])  # wp_num, meas_num of that wp

                    # reading tag data
                    if len(meas_data_line) > 5:
                        # print ('found at least one tag')
                        num_tags = (len(meas_data_line) - 5) / 8
                        meas_all_tags_list = []

                        for t in range(num_tags):
                            meas_tag_list = []

                            for index in range(8 * t, 8 * (t + 1)):
                                meas_tag_list.append(meas_data_line[5 + index])

                            meas_all_tags_list.append(meas_tag_list)
                        # checking for exact duplicates of measurements (in that moment probably no tag seen
                        # -> saving most recent measurement again)
                        if meas_all_tags_list == previous_meas:
                            # (either delete) or simply don't append to meas_data_line_list
                            # todo: maybe better to actually delete them in file?
                            pass
                        else:
                            meas_data_line_list.append(meas_all_tags_list)

                        previous_meas = meas_all_tags_list

                    # print meas_data_line_list
                    all_meas_data.append(meas_data_line_list)

        return all_meas_data


hippo = Boat()

hippo.set_noise(10, 0,
                0.1)  # set noise parameters (sense_noise, move_noise, turn_noise (hippo needs turn_noise for simulated measurement))

particles = []

# create random particles (numP = number of particles)
# move_noise determines how much particles move each iteration during update atm
for i in range(numP):
    particle = Boat()
    particle.set_noise(50, 50, 0.5)
    particles.append(particle)

# -------------- real data ----------------- #

if simulation is False:

    all_meas_data = Boat.read_data(meas_file_rel_path)

    steps = 9  # number of iterations
    for t in range(steps):

        # 'moving' boat to current waypoint, so far camera had same orientation for all waypoints -> cam_orientation defined at the top is used
        hippo.set(all_meas_data[t][0][0], all_meas_data[t][0][1], all_meas_data[t][0][2], cam_orientation)
        # print all_meas_data[t][0][0], all_meas_data[t][0][1], all_meas_data[t][0][2]

        # take noisy measurement of tag distances
        measurements = Boat.real_sense(all_meas_data, t)

        # move particles (so far only adding random noise, noise parameter: move_noise)
        particles2 = []
        for i in range(len(particles)):
            particles2.append(particles[i].move(0, 0, 0, 1, noisy=True))

        particles_old = particles
        particles = particles2

        # print len(measurements)

        if len(measurements) > 0:
            # weight particles according to how likely the measurement would be
            weights = []
            n = 1
            for i in range(len(particles)):
                weight = particles[i].measurement_prob(measurements, tags)
                weights.append(weight)

            weights /= sum(weights)  # normalize

            # resample particles, @todo look over this again
            particles3 = []
            index = int(random.random() * numP)
            beta = 0.0
            mw = np.amax(weights)
            for i in range(numP):
                beta += random.random() * 2.0 * mw

                while beta > weights[index]:
                    beta -= weights[index]
                    index = (index + 1) % numP

                particles3.append(particles[index])

        print 'step: ' + str(t) + ', measurements: ' + str(measurements)

        all_x = []
        all_y = []
        all_z = []
        for particle in particles3:
            all_x.append(particle.get_x())
            all_y.append(particle.get_y())
            all_z.append(particle.get_z())
        x_mean = mean(all_x)
        y_mean = mean(all_y)
        z_mean = mean(all_z)

        print 'mean of resampled particles x position: ' + str(x_mean) + ', real x position hippo: ' + str(
            hippo.get_x())
        print 'error in x: ' + str(abs(x_mean - hippo.get_x()))

        print 'mean of resampled particles y position: ' + str(y_mean) + ', real y position hippo: ' + str(
            hippo.get_y())
        print 'error in y: ' + str(abs(y_mean - hippo.get_y()))

        print 'mean of resampled particles z position: ' + str(z_mean) + ', real z position hippo: ' + str(
            hippo.get_z())
        print 'error in z: ' + str(abs(z_mean - hippo.get_z()))

        # -------------- plot each iteration --------------- #

        # uncomment this and fig = plt.figure() at the top to use subplots
        subplot_position = 331 + t  # 231 # plotting 6 plots, so either adjust this or choose 6 steps to plot if steps > 6
        ax = fig.add_subplot(subplot_position)
        # fig = plt.figure()
        plt.suptitle('Particle filter, using real measurements, meas_type: ' + str(meas_type))
        plt.title('Step ' + str(t))  # title includes number of step
        grid = [0, tank_size_x, 0, tank_size_y]
        plt.axis(grid)
        plt.grid(b=True, which='major', color='0.75', linestyle='--')

        # plot tags in red
        for i in range(len(tags)):
            # Circle((x,y), radius), tags are red and opaque (alpha = 1)
            circle = plt.Circle((tags[i].get_position_wf()[0], tags[i].get_position_wf()[1]), 50.0, facecolor='#cc0000',
                                edgecolor='#330000')
            plt.gca().add_patch(circle)

        # plot hippo's real position in blue, on top of particles
        circle = plt.Circle((hippo.get_x(), hippo.get_y()), 50.0, facecolor='#6666ff', edgecolor='#0000cc', zorder='3')
        plt.gca().add_patch(circle)

        # plot particles in orange and transparent (0 < alpha < 1)
        for i in range(len(particles)):
            circle = plt.Circle((particles[i].get_x(), particles[i].get_y()), 50.0, facecolor='#ffb266',
                                edgecolor='#994c00', alpha=0.2)
            plt.gca().add_patch(circle)

        if len(measurements) > 0:
            # plot resampled particles
            for i in range(len(particles)):
                circle = plt.Circle((particles3[i].get_x(), particles3[i].get_y()), 50.0, facecolor='#66ff66',
                                    edgecolor='#009900', alpha=0.2)
                plt.gca().add_patch(circle)

        if len(measurements) > 0:
            particles = particles3

            # particles = particles3

    plt.show()
    plt.close()
