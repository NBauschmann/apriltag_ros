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
numP = 150

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

# Tag_1 = tc.Tag(1, np.array([4.383, 0.512, 0.0]), tag_w1_orientation)
# Tag_2 = tc.Tag(2, np.array([4.383, 1.477, 0.0]), tag_w1_orientation)
"""
Tag_0 = tc.Tag(0, np.array([4.0, 1.0, 0.5]), tag_w1_orientation)
Tag_1 = tc.Tag(1, np.array([4.0, 1.0, 0.2]), tag_w1_orientation)
Tag_2 = tc.Tag(2, np.array([4.0, 1.3, 0.5]), tag_w1_orientation)
Tag_3 = tc.Tag(3, np.array([4.0, 0.308, 1.14]), tag_w1_orientation)
Tag_4 = tc.Tag(4, np.array([2.68, 0.0, 1.128]), tag_w4_orientation)
Tag_5 = tc.Tag(5, np.array([2.64, 2.0, 1.236]), tag_w2_orientation)
Tag_6 = tc.Tag(6, np.array([4.0, 0.773, 1.305]), tag_w1_orientation)
Tag_7 = tc.Tag(7, np.array([1.39, -0.152, 1.118]), tag_w4_orientation)
Tag_8 = tc.Tag(8, np.array([3.43, 2.15, 1.12]), tag_w2_orientation)
"""

# Tags im Flur
"""
Tag_0 = tc.Tag(0, np.array([2.61, 0.0, 0.645]), tag_w4_orientation)
Tag_1 = tc.Tag(1, np.array([3.315, -0.152, 0.705]), tag_w4_orientation)
Tag_2 = tc.Tag(2, np.array([4.0, 0.18, 0.52]), tag_w1_orientation)
Tag_3 = tc.Tag(3, np.array([4.0, 0.77, 0.685]), tag_w1_orientation)
Tag_4 = tc.Tag(4, np.array([3.55, 2.15, 0.695]), tag_w2_orientation)
Tag_5 = tc.Tag(5, np.array([3.025, 2.15, 0.77]), tag_w2_orientation)
Tag_6 = tc.Tag(6, np.array([2.6, 2.0, 0.615]), tag_w2_orientation)
Tag_7 = tc.Tag(7, np.array([2.15, 2.15, 0.78]), tag_w2_orientation)
Tag_8 = tc.Tag(8, np.array([1.75, 2.15, 0.82]), tag_w2_orientation)
Tag_9 = tc.Tag(9, np.array([1.875, -0.15, 0.75]), tag_w4_orientation)
Tag_10 = tc.Tag(10, np.array([1.385, -0.15, 0.805]), tag_w4_orientation)
Tag_11 = tc.Tag(11, np.array([0.71, -0.15, 0.655]), tag_w4_orientation)
Tag_12 = tc.Tag(12, np.array([1.25, 2.15, 0.715]), tag_w2_orientation)
Tag_13 = tc.Tag(13, np.array([0.6, 2.15, 0.7]), tag_w2_orientation)
Tag_14 = tc.Tag(14, np.array([0.0, 1.4, 0.78]), tag_w3_orientation)
Tag_15 = tc.Tag(15, np.array([0.0, 0.6, 0.8]), tag_w3_orientation) 
"""

#tags = [Tag_0, Tag_1, Tag_2, Tag_3, Tag_4, Tag_5, Tag_6, Tag_7, Tag_8, Tag_9, Tag_10, Tag_11, Tag_12, Tag_13, Tag_14, Tag_15]

# Tags im Tank
Tag_0 = tc.Tag(0, np.array([0, 0 ,0]), tag_w1_orientation) # not used
Tag_1 = tc.Tag(1, np.array([0, 0, 0]), tag_w1_orientation)  # not used
Tag_2 = tc.Tag(2, np.array([0, 0, 0]), tag_w1_orientation)  # not used -> not declared in tags.yaml
Tag_3 = tc.Tag(3, np.array([0.8, 0, 0.48]), tag_w4_orientation)
Tag_4 = tc.Tag(4, np.array([1.6, 0, 0.48]), tag_w4_orientation)
Tag_5 = tc.Tag(5, np.array([2.4, 0, 0.48]), tag_w4_orientation)
Tag_6 = tc.Tag(6, np.array([3.2, 0, 0.48]), tag_w4_orientation)
Tag_7 = tc.Tag(7, np.array([4.0, 0.6, 0.48]), tag_w1_orientation)
Tag_8 = tc.Tag(8, np.array([4.0, 1.0, 0.48]), tag_w1_orientation)
Tag_9 = tc.Tag(9, np.array([4.0, 1.4, 0.48]), tag_w1_orientation)
Tag_10 = tc.Tag(10, np.array([3.2, 2.0, 0.48]), tag_w2_orientation)
Tag_11 = tc.Tag(11, np.array([2.4, 2.0, 0.48]), tag_w2_orientation)
Tag_12 = tc.Tag(12, np.array([1.6, 2.0, 0.48]), tag_w2_orientation)
Tag_13 = tc.Tag(13, np.array([0.8, 2.0, 0.48]), tag_w2_orientation)
Tag_14 = tc.Tag(14, np.array([0.0, 1.4, 0.48]), tag_w3_orientation)
Tag_15 = tc.Tag(15, np.array([0.0, 1.0, 0.48]), tag_w3_orientation)
Tag_16 = tc.Tag(16, np.array([0.0, 0.6, 0.48]), tag_w3_orientation)

tags = [Tag_0, Tag_1, Tag_2, Tag_3, Tag_4, Tag_5, Tag_6, Tag_7, Tag_8, Tag_9, Tag_10, Tag_11, Tag_12, Tag_13, Tag_14, Tag_15, Tag_16]
