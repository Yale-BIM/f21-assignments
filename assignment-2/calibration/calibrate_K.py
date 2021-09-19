#!/usr/bin/env python3
# Complete this script with your solution to Part V of Assignment 2
import sys
import numpy as np


def compute_K(data_file):
    """
    Solve Assignment 3 - Part V
    :param data_file: path to file with 3D-2D point correspondences (correspondences.txt)
    :return: K as a 3x3 numpy array, error
    """
    # Load data and solve for K...
    return np.eye(3), np.inf  # Change this default return statement with your solution


if __name__ == '__main__':
    """
    Main function
    """

    if len(sys.argv) < 2:
        print("Not enough arguments. Run as: ./calibrate_K.py <correspondences-txt-file>")
        sys.exit(1)

    data_file = sys.argv[1]
    K, error = compute_K(data_file)

    print("Input file: {}".format(data_file))
    print("K:\n{}".format(K))
    print("Error: {}".format(error))
    sys.exit(0)
