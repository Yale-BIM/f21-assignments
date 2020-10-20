#!/usr/bin/env python
"""
Script that trains a NN policy for controlling the robot
"""

import sys
import numpy as np


def load_data(input_file):
    """
    Load training data from disk
    :param input_file: path to input file
    :return: NxF features matrix (state), and Nx2 target values (action)
    """

    target_frame = None
    target_x = []
    target_y = []
    target_z = []
    joint_1 = []
    joint_3 = []
    action_1 = []
    action_3 = []

    with open(input_file, 'r') as fid:
        for line in fid:
            if line[0] == "#":
                # skip comments
                continue

            line = line.rstrip().split("\t")
            assert len(line) == 8, "Expected each line to have 8 elements."

            if target_frame is not None and line[0] != target_frame:
                print("Inconsistent target frame for the target. Expected {} but got {}".format(target_frame, line[0]))
                sys.exit(1)
            else:
                target_frame = line[0]

            target_x.append(line[1])
            target_y.append(line[2])
            target_z.append(line[3])
            joint_1.append(line[4])
            joint_3.append(line[5])
            action_1.append(line[6])
            action_3.append(line[7])

    features = np.column_stack((target_x, target_y, target_z, joint_1, joint_3))
    targets = np.column_stack((action_1, action_3))

    return features, targets

