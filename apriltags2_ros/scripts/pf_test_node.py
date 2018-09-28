#!/usr/bin/env python

import rospy
import numpy as np
from numpy import *
import time
import math
from pyquaternion import Quaternion
import random
from scipy.stats import multivariate_normal

import threading
from threading import Lock
from os import path

# import settings
import tag_class as tc
import settings as se

import utils as Utils   # is this useful?

# import messages
from geometry_msgs.msg import Pose
from apriltags2_ros.msg import HippoPose
from apriltags2_ros.msg import HippoPoses

"""
Particle Filter 2

This particle filter uses absolute positions in world frame instead of the 
actual measurements (distance and orientation from camera to tag).

"""


simulation = True

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

# tags from tags_file, add more there
tags = [se.Tag_4]

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

        #self.__orientation = orientation

    def get_position(self):
        return self.__x, self.__y, self.__z

    def get_x(self):
        return self.__x

    def get_y(self):
        return self.__y

    def get_z(self):
        return self.__z

    #def get_orientation(self):
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

            if meas_type == 2:  # measurement needs to include quaternions as well!!!!
                if noisy:  # for simulation
                    # orientation not noise yet
                    measurement = [tags[i].get_id(), self.__x + random.gauss(0.0, self.__sense_noise),
                                   self.__y + random.gauss(0.0, self.__sense_noise),
                                   self.__z + random.gauss(0.0, self.__sense_noise)]#, self.__orientation]
                    measured_tags.append(measurement)

                else:  # for measurement_prob
                    measurement = [tags[i].get_id(), self.__x, self.__y, self.__z]#, self.__orientation]
                    measured_tags.append(measurement)
            """
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
            """
        return measured_tags

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
        for ind in range(len_meas * numV):
            m[ind][ind] = 50.0 * 50.0
        cov_matrix = m

        # weight of particles
        weight = multivariate_normal.pdf(predicted_measurement_resized, mean=measurements_resized, cov=cov_matrix)

        # print weight
        return weight


class ParticleFilter1(object):
    def __init__(self, pub, particles):
        self.__pub = pub
        self.__particles = particles

    def callback(self, msg):

        measurements = []

        for p in msg.poses:

            tag_id = p.id
            # position = np.array([[p.pose.position.x], [p.pose.position.y], [p.pose.position.z]])
            # orientation = Quaternion(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
            # measurement = [tag_id, position, orientation]
            measurement = [tag_id, p.pose.position.x * 1000, p.pose.position.y * 1000, p.pose.position.z * 1000]

            measurements.append(measurement)

        #print measurements

        # move particles (so far only adding random noise, noise parameter: move_noise)
        particles2 = []
        for i in range(numP):
            particles2.append(self.__particles[i].move(0, 0, 0, 1, noisy=True))

        # particles_old = self.__particles # only needed for plotting
        self.__particles = particles2

        # print len(measurements)

        x_mean = None
        y_mean = None
        z_mean = None

        if len(measurements) > 0:

            # weight particles according to how likely the measurement would be
            weights = []
            # n = 1

            for i in range(numP):
                weight = self.__particles[i].measurement_prob(measurements, tags)
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

                particles3.append(self.__particles[index])

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

            #rospy.loginfo('seen_tag_id: {}, x_mean: {}'.format(tag_id, x_mean))

            self.__particles = particles3

        # publish estimated pose

        pub_pose = Pose()

        pub_pose.position.x = x_mean
        pub_pose.position.y = y_mean
        pub_pose.position.z = z_mean

        self.__pub.publish(pub_pose)

particles1 = np.zeros((se.numP, 3))
class ParticleFilter():
    """
    
    """

    def __init__(self):

        # parameters
        self.numP = se.numP
        self.move_noise_x = se.move_noise_x
        self.move_noise_y = se.move_noise_y
        self.move_noise_z = se.move_noise_z

        # data used in particle filter algorithm
        self.camera_initialized = False
        self.initialized_particles = False

        self.iterations = 0
        self.state_lock = Lock()

        #
        self.measurement = None

        # particle poses and weights
        self.particle_indices = np.arange(self.numP)
        self.particles1 = np.zeros((self.numP, 3))  # so far x, y, z
        self.particles = self.initialize_global(particles1)
        self.weights = np.ones(self.numP) / float(self.numP)
        self.estimated_pose = None

        # initialize the state
        self.timer = Utils.Timer(10)

        # published topic
        self.pose_pub = rospy.Publisher("/estimated_pose", Pose, queue_size=10)

        # subscribed topic
        self.tags_sub = rospy.Subscriber("/hippo_poses", HippoPoses, self.read_hippo_poses)

    def read_hippo_poses(self, msg):

        #if not self.initialized_particles:
        #    self.initialize_global()


        counter = len(msg.poses)
        self.measurement = np.zeros((counter, 4))
        # array = ([[tag id, x, y, z],
        #           [tag id, x, y, z],
        #           ...
        #           [tag id, x, y, z]])
        #
        for counter, p in enumerate(msg.poses):
            self.measurement[counter][0] = p.id
            self.measurement[counter][1] = p.pose.position.x * 1000
            self.measurement[counter][2] = p.pose.position.y * 1000
            self.measurement[counter][3] = p.pose.position.z * 1000

        self.camera_initialized = True
        #print self.measurement

        self.update()

    def initialize_global(self, particles):

        self.state_lock.acquire()
        for counter in range(self.numP):
            particles[counter][0] = random.random() * se.tank_size_x
            particles[counter][1] = random.random() * se.tank_size_y
            particles[counter][2] = random.random() * se.tank_size_z
        self.state_lock.release()
        self.initialized_particles = True
        print "initialized global particle distribution"

        return particles

    def motion_model(self, proposal_dist):
        #
        for counter in range(self.numP):
            proposal_dist[counter][0] += random.gauss(0, self.move_noise_x)
            proposal_dist[counter][1] += random.gauss(0, self.move_noise_y)
            proposal_dist[counter][2] += random.gauss(0, self.move_noise_z)




    def sensor_model(self, proposal_dist, observation, weights):
        # do something
        #print self.iterations
        #print observation

        len_obs = len(observation)
        predicted_observation = np.zeros((len_obs, 4))

        m = np.zeros((len_obs * 4, len_obs * 4))
        for ind in range(len_obs * 4):
            m[ind][ind] = 50.0 * 50.0
        cov_matrix = m

        for particle in range(self.numP):
            for i in range(len_obs):
                predicted_observation[i][0] = observation[i][0]
                predicted_observation[i][1] = self.particles[particle][0]
                predicted_observation[i][2] = self.particles[particle][1]
                predicted_observation[i][3] = self.particles[particle][2]

                #print self.particles
                #print observation
                #print predicted_observation

            self.weights[particle] = multivariate_normal.pdf(np.reshape(predicted_observation, len_obs * 4), mean=np.reshape(observation, len_obs * 4), cov=cov_matrix)
            #print self.weights


    def particle_filter_algorithm(self, observation):

        # todo: change order?
        # draw the proposal distribution from the old particles
        proposal_indices = np.random.choice(self.particle_indices, self.numP, p=self.weights)
        proposal_distribution = self.particles[proposal_indices,:]

        # apply motion model
        self.motion_model(proposal_distribution)
        # sensor model, weighting particles
        self.sensor_model(proposal_distribution, observation, self.weights)
        # normalizing weights again
        self.weights /= np.sum(self.weights)

        self.particles = proposal_distribution

    def expected_pose(self):
        # print np.dot(self.particles.transpose(), self.weights)
        return np.dot(self.particles.transpose(), self.weights)

    def update(self):
        if self.camera_initialized:
            if self.state_lock.locked():
                #pass
                print "concurrency error avoided"
            else:
                self.state_lock.acquire()
                self.timer.tick()



                self.iterations += 1

                observation = np.copy(self.measurement)
                # action =

                # run partilce_filter_algoritm:
                self.particle_filter_algorithm(observation)
                self.estimated_pose = self.expected_pose()
                #
                self.state_lock.release()

                # publish estimated pose
                if isinstance(self.estimated_pose, np.ndarray):
                    p = Pose()
                    p.position.x = self.estimated_pose[0]
                    p.position.y = self.estimated_pose[1]
                    p.position.z = self.estimated_pose[2]
                    self.pose_pub.publish(p)
        else:
            print "Camera not initialized"

"""
def main():

    particles = []
    # create random particles (numP = number of particles)
    # move_noise determines how much particles move each iteration during update atm
    for i in range(numP):
        particle = Boat()
        particle.set_noise(0.01, 10, 0.5)
        particles.append(particle)

    # initialize subscriber and publisher
    rospy.init_node('particle_filter_node')
    pub = rospy.Publisher('estimated_pose', Pose, queue_size=10)
    particle_filter = ParticleFilter1(pub, particles)
    rospy.Subscriber("/hippo_poses", HippoPoses, particle_filter.callback)

    # rospy.loginfo('counter: {}'.format(counter))

    while not rospy.is_shutdown():
        pass

    # rospy.spin()
"""



if __name__ == '__main__':
    # main()
    rospy.init_node('pf_test_node')
    pf = ParticleFilter()
    rospy.spin()

