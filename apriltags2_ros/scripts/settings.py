#!/usr/bin/env python

from pyquaternion import Quaternion
import numpy as np
import tag_class as tc


"""
This file is for setting any parameters for the particle 
filter and the position and orientation of the tags.

todo: move NumV here as well
"""

""" Particle filter parameters """

# Publish transforms and particles for Rviz
use_rviz = True

# Number of particles
numP = 200

# Noise parameters
particle_sense_noise = 0
particle_move_noise = 0.025
particle_turn_noise = 0

# Covariance matrix: standard deviation * standard deviation
cov_mat_parameter = 0.1


# size of tank:
# so far minimum = 0 todo
tank_size_x = 4.0
tank_size_y = 2.0
tank_size_z = 1.5


""" Tag positions and orientations inside the tank """

"""
frames: 
       - world/map frame: north, east, down, origin in corner 
         between wall 3 and 4 (defined below)
       - camera frame: looking from behind the camera (like a
         photographer), x is right, y is down and z is straight
         ahead
       - tag frame: looking straight at the tag (oriented correctly),
         x is right, y is up and z is towards you (out of the tag)


Tag orientation:

rotation from world frame (wf) to tag frame (tf) according to wall 

for wall closest to big windows (wall 1): q1 = 0.5 - 0.5i - 0.5j + 0.5k
for wall closest to wave tank (wall 2): q2 = 0 + 0i - 0.707107j + 0.707107k
for wall closest to computers (wall 3): q3 = 0.5 - 0.5i + 0.5j - 0.5k 
for wall closest to stairs (wall 4): q4 = 0.707107 - 0.707107i + 0j + 0k
calculated below
"""

# calculating quaternion for wall 1 (windows)

rotation_w1 = np.array([[0, 0, -1.0], [1.0, 0, 0], [0, -1.0, 0]])
tag_w1_orientation = Quaternion(matrix=rotation_w1)

# calculating quaternion for wall 2 (wave tank)
rotation_w2 = np.array([[-1.0, 0, 0], [0, 0, -1.0], [0, -1.0, 0]])
tag_w2_orientation = Quaternion(matrix=rotation_w2)

# calculating quaternion for wall 3 (computers)
rotation_w3 = np.array([[0, 0, 1.0], [-1.0, 0, 0], [0, -1.0, 0]])
tag_w3_orientation = Quaternion(matrix=rotation_w3)

# calculating quaternion for wall 4 (stairs)
rotation_w4 = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])
tag_w4_orientation = Quaternion(matrix=rotation_w4)


"""
Create object of class Tag for every used tag 
Position needs to be [m]

Add Tags in list tags at the bottom
"""

# Tags an Stangen

# laenge stangen nr
l1 = 1.76 
l2 = 1.98
l3 = 1.98
l4 = 1.49
l5 = 1.98
l6 = 1.98

# offset stange nr
s1 = 0.1
s2 = 0
s3 = 0   # abstand zwischen 2 und 3
s4 = 0.15
s5 = 0   # abstand zwischen 5 und 6
s6 = 0

# anbringtiefe stangen
d = 1.0

# Tags for orientation test
Tag_0 = tc.Tag(0, np.array([1.6, 2.0, 0.7]), tag_w2_orientation)
Tag_1 = tc.Tag(1, np.array([2.0, 2.0, 0.7]), tag_w2_orientation)
Tag_2 = tc.Tag(2, np.array([2.4, 2.0, 0.7]), tag_w2_orientation)

tags = [Tag_0, Tag_1, Tag_2]

"""
# Used tags mounted in tank

Tag_2 = tc.Tag(0, np.array([0.0, 0.4 + s1, d]), tag_w3_orientation) # tag 2 and 0 were originally swapped, todo: change order
Tag_1 = tc.Tag(1, np.array([0.0, 0.85 + s1, d]), tag_w3_orientation)
Tag_0 = tc.Tag(2, np.array([0.0, 1.35 + s1, d]), tag_w3_orientation)
Tag_3 = tc.Tag(3, np.array([0.8 + s2, 0.0, d]), tag_w4_orientation)
Tag_4 = tc.Tag(4, np.array([1.6 + s2, 0.0, d]), tag_w4_orientation)
Tag_5 = tc.Tag(5, np.array([0.4 + l2 + s2 + s3, 0.0, d]), tag_w4_orientation)
Tag_6 = tc.Tag(6, np.array([1.2 + l2 + s2 + s3, 0.0, d]), tag_w4_orientation)
Tag_7 = tc.Tag(7, np.array([4.0, 0.3 + s4, d]), tag_w1_orientation)
Tag_8 = tc.Tag(8, np.array([4.0, 0.8 + s4, d]), tag_w1_orientation)
Tag_9 = tc.Tag(9, np.array([4.0, 1.3 + s4, d]), tag_w1_orientation)
Tag_10 = tc.Tag(10, np.array([1.2 + l6 + s6 + s5, 2.0, d]), tag_w2_orientation)
Tag_11 = tc.Tag(11, np.array([0.4 + l6 + s6 + s5, 2.0, d]), tag_w2_orientation)
Tag_12 = tc.Tag(12, np.array([1.6 + s6, 2.0, d]), tag_w2_orientation)
Tag_13 = tc.Tag(13, np.array([0.8 + s6, 2.0, d]), tag_w2_orientation) 
#not used from here
Tag_14 = tc.Tag(14, np.array([0.0, 0.0, 0.0]), tag_w1_orientation)
Tag_15 = tc.Tag(15, np.array([0.0, 0.0, 0.0]), tag_w1_orientation) 
Tag_16 = tc.Tag(16, np.array([0.0, 0.0, 0.0]), tag_w1_orientation)
Tag_17 = tc.Tag(17, np.array([0.0, 0.0, 0.0]), tag_w1_orientation)
Tag_18 = tc.Tag(18, np.array([0.0, 0.0, 0.0]), tag_w1_orientation)
Tag_19 = tc.Tag(19, np.array([0.0, 0.0, 0.0]), tag_w1_orientation)
# small tags
Tag_20 = tc.Tag(20, np.array([0.0, 1.08 + s1, d]), tag_w3_orientation)
Tag_21 = tc.Tag(21, np.array([0.0, 0.63 + s1, d]), tag_w3_orientation)
Tag_22 = tc.Tag(22, np.array([1.1 + s2, 0.0, d]), tag_w4_orientation)
Tag_23 = tc.Tag(23, np.array([1.3 + s2, 0.0, d]), tag_w4_orientation)
Tag_24 = tc.Tag(24, np.array([1.9 + s2, 0.0, d]), tag_w4_orientation)
Tag_25 = tc.Tag(25, np.array([0.1 + l2 + s2 + s3, 0.0, d]), tag_w4_orientation)
Tag_26 = tc.Tag(26, np.array([0.7 + l2 + s2 + s3, 0.0, d]), tag_w4_orientation)
Tag_27 = tc.Tag(27, np.array([0.9 + l2 + s2 + s3, 0.0, d]), tag_w4_orientation)
Tag_28 = tc.Tag(28, np.array([4.0, 0.55 + s4, d]), tag_w1_orientation)
Tag_29 = tc.Tag(29, np.array([4.0, 1.05 + s4, d]), tag_w1_orientation)
Tag_30 = tc.Tag(30, np.array([0.9 + l6 + s6 + s5, 2.0, d]), tag_w2_orientation)
Tag_31 = tc.Tag(31, np.array([0.7 + l6 + s6 + s5, 2.0, d]), tag_w2_orientation)
Tag_32 = tc.Tag(32, np.array([0.1 + l6 + s6 + s5, 2.0, d]), tag_w2_orientation)
Tag_33 = tc.Tag(33, np.array([1.9 + s6, 2.0, d]), tag_w2_orientation)
Tag_34 = tc.Tag(34, np.array([1.3 + s6, 2.0, d]), tag_w2_orientation)
Tag_35 = tc.Tag(35, np.array([1.1 + s6, 2.0, d]), tag_w2_orientation)

tags = [Tag_0, Tag_1, Tag_2, Tag_3, Tag_4, Tag_5, Tag_6, Tag_7, Tag_8, Tag_9, Tag_10, Tag_11, Tag_12, Tag_13, Tag_14, Tag_15, Tag_16, Tag_17, Tag_18, Tag_19, Tag_20, Tag_21, Tag_22, Tag_23, Tag_24, Tag_25, Tag_26, Tag_27, Tag_28, Tag_29, Tag_30, Tag_31, Tag_32, Tag_33, Tag_34, Tag_35]
"""

