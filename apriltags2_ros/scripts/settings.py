#!/usr/bin/env python

from pyquaternion import Quaternion
import numpy as np
import tag_class as tc


"""
This file is for setting the position and orientation of each tag in the tank 
(in world frame) and for setting tank measurements for particle filter
and any other parameters needed to avoid setting them in each node separately.

"""

# Number of particles for particle filter
numP = 200

# Particle filter parameters
move_noise_x = 5
move_noise_y = 5
move_noise_z = 5

# size of tank:
# so far minimum = 0 todo
tank_size_x = 4000.0
tank_size_y = 2000.0
tank_size_z = 200.0


"""
Tag orientation:

rotation from world frame (wf) to tag frame (tf) according to wall 

tag coordinate system: in tag center, x-axis pointing right, y-axis pointing down, z-axis pointing into tag

for wall closest to big windows (wall 1): q1 = 0.5 + 0.5i + 0.5j + 0.5k
for wall closest to wave tank (wall 2): q2 = 0 + 0i +0.707107j + 0.707107k
for wall closest to computers (wall 3): q3 = 0,5 + 0,5i - 0,5j - 0,5k 
for wall closest to stairs (wall 4): q4 = 0.707107 + 0.707107i + 0j + 0k
calculated below
"""

# calculating quaternion for wall 1 (windows)

tag_w1_orientation_1 = Quaternion(axis=(0, 0, 1.0), degrees=90)
tag_w1_orientation_2 = Quaternion(axis=(1.0, 0, 0), degrees=90)
tag_w1_orientation = tag_w1_orientation_1*tag_w1_orientation_2

# different way
# rotation_w1 = np.array([[0, 0, 1.0], [1.0, 0, 0], [0, 1.0, 0]])
# tag_w1_orientation = Quaternion(matrix=rotation_w1)

# calculating quaternion for wall 2 (wave tank)
rotation_w2 = np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])
tag_w2_orientation = Quaternion(matrix=rotation_w2)

# calculating quaternion for wall 3 (computers)
rotation_w3 = np.array([[0, 0, -1.0], [-1.0, 0, 0], [0, 1.0, 0]])
tag_w3_orientation = Quaternion(matrix=rotation_w3)

# calculating quaternion for wall 4 (stairs)
rotation_w4 = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
tag_w4_orientation = Quaternion(matrix=rotation_w4)


"""
Create object of class Tag for every used tag 
in METER!!
"""
Tag_0 = tc.Tag(0, np.array([0, 0, 0]), tag_w1_orientation)
Tag_1 = tc.Tag(1, np.array([4.383, 0.512, 0.0]), tag_w1_orientation)
Tag_2 = tc.Tag(2, np.array([4.383, 1.477, 0.0]), tag_w1_orientation)
Tag_3 = tc.Tag(3, np.array([0, 0, 0]), tag_w1_orientation)
Tag_4 = tc.Tag(4, np.array([4.000, 1.0, 0.5]), tag_w1_orientation)
