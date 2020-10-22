#!/usr/bin/env python
"""
Script that trains a NN policy for controlling the robot
"""

import sys
import numpy as np
import gdown

# TODO - replace None in the MODEL_URL with the URL to your keras model in Google Drive.
# Ensure that anybody can download the file and test this before submitting your assignment by running within a python
# shell:
#   $ import train_utils
#   $ train_utils.download_keras_model("my_keras_model")
# Then, you should have a copy of your model named my_keras_model in your local disk.
MODEL_URL = None


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


def download_keras_model(output_path, quiet=False):
    """
    Helper function to download a trained Keras model from Google Drive
    :param output_path: output path for the model
    :param quiet: print info?
    :return: None if there was an error and the file could not be downloaded; output path otherwise.
    """
    if MODEL_URL is None:
        print("MODEL_URL is not set in train_utils.py. Failed to download the model")
        return None
    out = gdown.download(MODEL_URL, output_path, quiet=quiet)
    if out is None:
        print("Failed to download {} from Google Drive. Check that the path is correct and the file can be downloaded "
              "by anybody without having to log into Google".format(MODEL_URL))
    return out
