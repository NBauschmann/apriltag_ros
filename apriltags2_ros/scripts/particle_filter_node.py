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
from apriltags2_ros.msg import Euler

import numpy.matlib as npm   # for averaging quaternions function
from pyquaternion import Quaternion

"""
Particle Filter 2

This particle filter uses absolute positions in world frame instead of the 
actual measurements (distance and orientation from camera to tag).

"""


# number of measured variables per measurement:
#numV = 4  # tag_id, x, y, z
numV = 3  # x, y, z

# number of particles used
numP = 200

# tags from settings, add more there
tags = se.tags

last_orientation_x = 0
last_orientation_y = 0
last_orientation_z = 0
last_orientation_w = 0

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


def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])


# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation

# from: https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
def average_quaternions(Q):
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

        # to make sure to stay in tank
        # works but not the best solution (not gaussian anymore) and probably incredibly slow
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
        """taking a measurement for each tag, measurement is the position of the camera in world frame"""
        measured_tags = []

        for i in range(len(tags)):

            if noisy:  # for simulation
                # orientation not noisy yet
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
        # get ids of seen tags
        for ind in range(len(measurements)):
            seen_tag_ids.append(measurements[ind][0])

        # get all predicted measurements (all tags) for this particle
        predicted_measurement_all = self.sense(tags, noisy=False)

        predicted_measurement = []

        # only use measurements of actually seen tags
        for ind in range(len(predicted_measurement_all)):
            for id in seen_tag_ids:
                if predicted_measurement_all[ind][0] == id:
                    predicted_measurement.append(predicted_measurement_all[ind])

        # resizing necessary
        len_meas = len(measurements)

        # numV = number of variables per measurement, defined at the top
        a = np.zeros((len_meas, numV))
        for ind in range(len(measurements)):

            # without id
            a[ind][0] = measurements[ind][1]
            a[ind][1] = measurements[ind][2]
            a[ind][2] = measurements[ind][3]

        measurements_resized = np.reshape(a, len_meas * numV)

        b = np.zeros((len_meas, numV))
        for ind in range(len(predicted_measurement)):

            # without id
            b[ind][0] = predicted_measurement[ind][1]
            b[ind][1] = predicted_measurement[ind][2]
            b[ind][2] = predicted_measurement[ind][3]

        predicted_measurement_resized = np.reshape(b, len_meas * numV)

        # covariance matrix (diagonal)
        m = np.zeros((len_meas * numV, len_meas * numV))
        for ind in range(len_meas * numV):
            m[ind][ind] = se.cov_mat_parameter * se.cov_mat_parameter
        cov_matrix = m

        # weight of particles
        weight = multivariate_normal.pdf(predicted_measurement_resized, mean=measurements_resized, cov=cov_matrix)
        return weight


class ParticleFilter(object):
    def __init__(self, pub_est_pose, pub_particles, pub_mavros_pose, pub_euler, particles):
        self.__pub_est_pose = pub_est_pose
        self.__pub_particles = pub_particles
        self.__pub_mavros_pose = pub_mavros_pose
        self.__pub_euler = pub_euler
        self.__particles = particles
        self.__message = []
        self.__last_orientation_x = 0
        self.__last_orientation_y = 0
        self.__last_orientation_z = 0
        self.__last_orientation_w = 0
        self.__euler = np.array([0, 0, 0])

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
        orientations = []

        for p in msg.poses:

            # orientation doesn't have to be in here
            measurement = [p.id, p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z]
            measurements.append(measurement)

            orientation = [p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z]
            orientations.append(orientation)

        # move particles (so far only adding random noise, noise parameter: move_noise)
        particles2 = []
        for i in range(numP):
            particles2.append(self.__particles[i].move(0, 0, 0, 1, noisy=True))

        # particles_old = self.__particles # only needed for plotting
        self.__particles = particles2

        if len(msg.poses) > 0:

            num_meas = len(msg.poses)

            # weight particles according to how likely the measurement would be
            weights = []
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


            # To publish orientation, a mean quaternion is calculated from all measured orientations
            # num_meas x 4 matrix, containing quaternions in rows, in order: w, x, y, z
            quaternions_mat = np.asarray(orientations)

            average_quaternion = average_quaternions(quaternions_mat)   # this is also in order: w, x, y, z
            # published further down

        # if len(msg.poses) = 0 -> no new measurements
        else:
            print("No new Measurement")

        # calculate mean position of particles
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
        # THIS POSE IS IN NED
        pub_pose = PoseStamped()
        pub_pose.header = u.make_header("map")
        pub_pose.pose.position.x = x_mean
        pub_pose.pose.position.y = y_mean
        pub_pose.pose.position.z = z_mean

        # since orientation isn't being filtered, only publish if measurement received
        if len(msg.poses) > 0:
            pub_pose.pose.orientation.x = average_quaternion[1]
            pub_pose.pose.orientation.y = average_quaternion[2]
            pub_pose.pose.orientation.z = average_quaternion[3]
            pub_pose.pose.orientation.w = average_quaternion[0]

            # save this to publish last measured orientation
            self.__last_orientation_x = average_quaternion[1]
            self.__last_orientation_y = average_quaternion[2]
            self.__last_orientation_z = average_quaternion[3]
            self.__last_orientation_w = average_quaternion[0]

            # convert quaternion -> rotation matrix -> euler angles
            meas_orient_quat = Quaternion(average_quaternion[0], average_quaternion[1], average_quaternion[2], average_quaternion[3])
            meas_orient_quat = meas_orient_quat.normalised   # normalize
            meas_orient_matrix = meas_orient_quat.rotation_matrix
            self.__euler = rotation_matrix_to_euler_angles(meas_orient_matrix)

        # if no measurement received: publish last measured orientation
        else:
            pub_pose.pose.orientation.x = self.__last_orientation_x
            pub_pose.pose.orientation.y = self.__last_orientation_y
            pub_pose.pose.orientation.z = self.__last_orientation_z
            pub_pose.pose.orientation.w = self.__last_orientation_w

        self.__pub_est_pose.publish(pub_pose)

        # publish estimated pose to /mavros/vision_pose/pose
        # expects to get coordinates in ENU, so need to change axes
        # THIS POSE IS IN ENU
        pub_mav_pose = PoseStamped()
        pub_mav_pose.header = u.make_header("map")    # not sure what header is needed todo
        pub_mav_pose.pose.position.x = y_mean
        pub_mav_pose.pose.position.y = x_mean
        pub_mav_pose.pose.position.z = - z_mean

        # publish euler angles to /euler
        pub_euler = Euler()
        pub_euler.roll = self.__euler[0]
        pub_euler.pitch = self.__euler[1]
        pub_euler.yaw = self.__euler[2]
        self.__pub_euler.publish(pub_euler)

        if len(msg.poses) > 0:
            # conversion from NED to ENU

            rot_mat = np.array([[0, 1.0, 0], [1.0, 0, 0], [0, 0, -1.0]])
            quat_ned = Quaternion(matrix=rot_mat)
            test_quat = meas_orient_quat * quat_ned
            """
            # Not sure if this is working (apparently: NED -> ENU: (w x y z) -> (y x -z w))
            # NOT WORKING
            # might work if publishing of local_pose in mavros_setpoints does not work
            pub_pose.pose.orientation.w = average_quaternion[2]
            pub_pose.pose.orientation.x = average_quaternion[1]
            pub_pose.pose.orientation.y = - average_quaternion[3]
            pub_pose.pose.orientation.z = average_quaternion[0]
            """

            #swapped_axes_quat = Quaternion(pub_pose.pose.orientation.w, pub_pose.pose.orientation.x, pub_pose.pose.orientation.y, pub_pose.pose.orientation.z)
            #print "swapped axes: " + str(swapped_axes_quat)
            #print "transforemd quat: " + str(test_quat)

            """
            # body-fixed NED -> ROS ENU: 
            # NOT WORKING
            pub_pose.pose.orientation.w = average_quaternion[1]
            pub_pose.pose.orientation.x = - average_quaternion[2]
            pub_pose.pose.orientation.y = - average_quaternion[3]
            pub_pose.pose.orientation.z = average_quaternion[0]
            """

            # TEST
            pub_mav_pose.pose.orientation.w = 0.
            pub_mav_pose.pose.orientation.x = 2.0
            pub_mav_pose.pose.orientation.y = 3.0
            pub_mav_pose.pose.orientation.z = 4.0

        self.__pub_mavros_pose.publish(pub_mav_pose)
        # without changing to ENU:
        # self.__pub4.publish(pub_pose)

        if se.use_rviz:
            # publish particles as PoseArray() (only used for rviz)
            pub3 = PoseArray()
            pub3.header = u.make_header("map")
            pub3.poses = self.particles_to_poses(self.__particles)
            self.__pub_particles.publish(pub3)


def main():

    particles = []
    # create random particles (numP = number of particles)
    # move_noise determines how much particles move during update each iteration  atm
    for i in range(numP):
        particle = Boat()
        particle.set_noise(se.particle_sense_noise, se.particle_move_noise, se.particle_turn_noise)
        particles.append(particle)
    print "Particles initialized"
    # initialize subscriber and publisher
    rospy.init_node('particle_filter_node')
    pub_est_pose = rospy.Publisher('estimated_pose', PoseStamped, queue_size=1)
    pub_particles = rospy.Publisher('particle_poses', PoseArray, queue_size=1)
    pub_mavros_pose = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    pub_euler = rospy.Publisher('euler', Euler, queue_size=1)
    particle_filter = ParticleFilter(pub_est_pose, pub_particles, pub_mavros_pose, pub_euler, particles)
    rospy.Subscriber("/hippo_poses", HippoPoses, particle_filter.callback, queue_size=1)

    while not rospy.is_shutdown():
        pass

    # rospy.spin()

if __name__ == '__main__':
    main()
