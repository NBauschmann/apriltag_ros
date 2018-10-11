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

import geometry_msgs
from std_msgs.msg import *
from geometry_msgs.msg import *
import utils as u

from apriltags2_ros.msg import HippoPose
from apriltags2_ros.msg import HippoPoses

"""
Particle Filter 2

This particle filter uses absolute positions in world frame instead of the 
actual measurements (distance and orientation from camera to tag).

"""


# set this if meas_type = 1
cam_orientation = Quaternion(0.5, 0.5, 0.5, 0.5)  # quaternion for camera facing wall with big windows

# number of measured variables per measurement:
numV = 4  # tag_id, x, y, z

# number of particles used
numP = 200

# tags from settings, add more there
tags = se.tags

# not needed anymore
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
        self.__x = random.random() * se.tank_size_x
        self.__y = random.random() * se.tank_size_y
        self.__z = random.random() * se.tank_size_z

        # self.__orientation = random_quaternion()

        self.__sense_noise = 0
        self.__move_noise = 0
        self.__turn_noise = 0

    def set(self, x, y, z): #, orientation):
        """set boat's position and orientation in the tank"""
        if x >= se.tank_size_x or x < 0:
            raise ValueError('x coordinate out of bound')
        if y >= se.tank_size_y or y < 0:
            raise ValueError('y coordinate out of bound')
        if z >= se.tank_size_z or z < 0:
            raise ValueError('z coordinate out of bound')

        self.__x = x
        self.__y = y
        self.__z = z

    def get_position(self):
        return self.__x, self.__y, self.__z

    def get_x(self):
        return self.__x

    def get_y(self):
        return self.__y

    def get_z(self):
        return self.__z

    # def get_orientation(self):
    #    return self.__orientation

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
        # self.__orientation = self.__orientation * turn  # no idea whether that works -> try changing order

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
        #orientation = self.__orientation
        res = Boat()
        res.set(x, y, z)#, orientation)
        res.set_noise(self.__sense_noise, self.__move_noise, self.__turn_noise)
        return res

    def sense(self, tags, noisy=False):
        """

        """
        measured_tags = []

        for i in range(len(tags)):

            if noisy:  # for simulation
                # orientation not noise yet
                measurement = [tags[i].get_id(), self.__x + random.gauss(0.0, self.__sense_noise),
                               self.__y + random.gauss(0.0, self.__sense_noise),
                               self.__z + random.gauss(0.0, self.__sense_noise)]#, self.__orientation]
                measured_tags.append(measurement)

            else:  # for measurement_prob
                measurement = [tags[i].get_id(), self.__x, self.__y, self.__z]#, self.__orientation]
                measured_tags.append(measurement)

        return measured_tags

    def measurement_prob(self, measurements, tags):
        """calculate measurement probability given state of particle (x, y, z)"""

        seen_tag_ids = []

        for ind in range(len(measurements)):
            seen_tag_ids.append(measurements[ind][0])

        predicted_measurement_all = self.sense(tags, noisy=False)

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
        for ind in range(len_meas * numV):
            m[ind][ind] = se.cov_mat_parameter * se.cov_mat_parameter
        cov_matrix = m

        # weight of particles
        weight = multivariate_normal.pdf(predicted_measurement_resized, mean=measurements_resized, cov=cov_matrix)

        # print weight
        return weight


class ParticleFilter(object):
    def __init__(self, pub, pub2, pub3, particles):
        self.__pub = pub
        self.__pub2 = pub2
        self.__pub3 = pub3
        self.__particles = particles
        self.__message = []

    @staticmethod
    def particle_to_pose(particle):
        pose = Pose()
        pose.position.x = particle.get_x()
        pose.position.y = particle.get_y()
        pose.position.z = particle.get_z()
        return pose

    def particles_to_poses(self, particles):
        return map(self.particle_to_pose, particles)

    def callback(self, msg):

        measurements = []

        for p in msg.poses:
            measurement = [p.id, p.pose.position.x, p.pose.position.y, p.pose.position.z]

            measurements.append(measurement)

        # move particles (so far only adding random noise, noise parameter: move_noise)
        particles2 = []
        for i in range(numP):
            particles2.append(self.__particles[i].move(0, 0, 0, 1, noisy=True))

        # particles_old = self.__particles # only needed for plotting
        self.__particles = particles2

        if len(msg.poses) > 0:

            # weight particles according to how likely the measurement would be
            weights = []
            # n = 1

            for i in range(numP):
                weight = self.__particles[i].measurement_prob(measurements, tags)
                weights.append(weight)

            weights /= sum(weights)  # normalize

            # resample particles
            particles3 = []
            index = int(random.random() * numP)
            beta = 0.0
            mw = np.amax(weights)
            for i in range(numP):
                beta += random.random() * 2.0 * mw

                while beta > weights[index]:
                    beta -= weights[index]
                    index = (index + 1) % numP

                particles3.append(self.__particles[index])
            self.__particles = particles3

        # if len(msg.poses) = 0 -> no new measurements
        else:
            print("No new Measurement")

        all_x = []
        all_y = []
        all_z = []
        for particle in self.__particles:
            all_x.append(particle.get_x())
            all_y.append(particle.get_y())
            all_z.append(particle.get_z())
        x_mean = mean(all_x)
        y_mean = mean(all_y)
        z_mean = mean(all_z)

        # print np.std(all_x), np.std(all_y), np.std(all_z)
        # rospy.loginfo('seen_tag_id: {}, x_mean: {}'.format(tag_id, x_mean))

        # publish estimated pose

        # as PoseStamped()
        pub_pose = PoseStamped()
        pub_pose.header = u.make_header("map")
        pub_pose.pose.position.x = x_mean
        pub_pose.pose.position.y = y_mean
        pub_pose.pose.position.z = z_mean
        self.__pub.publish(pub_pose)

        # as PointStamped()
        pub_2 = PointStamped()
        pub_2.header = u.make_header("map")
        pub_2.point.x = x_mean
        pub_2.point.y = y_mean
        pub_2.point.z = z_mean
        self.__pub2.publish(pub_2)

        # publish particles as PoseArray()
        pub3 = PoseArray()
        pub3.header = u.make_header("map")
        pub3.poses = self.particles_to_poses(self.__particles)
        self.__pub3.publish(pub3)


def main():

    particles = []
    # create random particles (numP = number of particles)
    # move_noise determines how much particles move each iteration during update atm
    for i in range(numP):
        particle = Boat()
        particle.set_noise(se.particle_sense_noise, se.particle_move_noise, se.particle_turn_noise)
        particles.append(particle)
    print "Particles initialized"
    # initialize subscriber and publisher
    rospy.init_node('particle_filter_node')
    pub = rospy.Publisher('estimated_pose', PoseStamped, queue_size=1)
    pub2 = rospy.Publisher('estimated_position', PointStamped, queue_size=1)
    pub3 = rospy.Publisher('particle_poses', PoseArray, queue_size=1)
    particle_filter = ParticleFilter(pub, pub2, pub3, particles)
    rospy.Subscriber("/hippo_poses", HippoPoses, particle_filter.callback, queue_size=1)

    # rospy.loginfo('counter: {}'.format(counter))

    while not rospy.is_shutdown():
        pass

    # rospy.spin()

if __name__ == '__main__':
    main()
