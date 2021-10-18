#!/usr/bin/env python3
"""
Script that trains a NN policy for controlling the robot
"""

import os
import sys
import numpy as np
import gdown

# TODO - replace None in the MODEL_URL with the URL to your keras model in Google Drive.
# TODO - if your model requires input normalization, then also replace the NORM_PARAMS_URL with the URL to your normalization parameters
# Ensure that anybody can download the files and test this before submitting your assignment by running within a Python
# shell:
#   $ import train_utils
#   $ train_utils.download_model_files(quiet=False)
# Then, you should have a copy of your model named my_keras_model in your local disk.
MODEL_URL = None
NORM_PARAMS_URL = None


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

            target_x.append(float(line[1]))
            target_y.append(float(line[2]))
            target_z.append(float(line[3]))
            joint_1.append(float(line[4]))
            joint_3.append(float(line[5]))
            action_1.append(float(line[6]))
            action_3.append(float(line[7]))

    features = np.column_stack((target_x, target_y, target_z, joint_1, joint_3))
    targets = np.column_stack((action_1, action_3))

    return features, targets


def download_model_files(output_path=".", quiet=False, model_name="keras_model", norm_params_name="norm_params"):
    """
    Helper function to download a trained Keras model from Google Drive
    :param output_path: output path for the model and normalization parameters
    :param quiet: print info?
    :param model_name: name for the saved model
    :param norm_params_name: name for the file with normalization params
    :return: output path of each downloaded file or None if there was an error
    """
    if MODEL_URL is None:
        print("MODEL_URL is not set in train_utils.py. Failed to download the model")
        return None, None

    out1 = gdown.download(MODEL_URL, os.path.join(output_path, model_name), quiet=quiet)
    if out1 is None:
        print("Failed to download {} from Google Drive. Check that the path is correct and the file can be downloaded "
              "by anybody without having to log into Google".format(MODEL_URL))

    if NORM_PARAMS_URL is not None:
        out2 = gdown.download(NORM_PARAMS_URL, os.path.join(output_path, norm_params_name), quiet=quiet)
        if out1 is None:
            print("Failed to download {} from Google Drive. Check that the path is correct and the file can "
                  "be downloaded by anybody without having to log into Google".format(NORM_PARAMS_URL))
    else:
        out2 = None

    return out1, out2
