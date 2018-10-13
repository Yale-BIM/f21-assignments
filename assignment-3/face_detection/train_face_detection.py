#!/usr/bin/env python
# Script to train and test a neural network with TF's Keras API for face detection

import os
import sys
import argparse
import datetime
import numpy as np
import tensorflow as tf


def load_data_from_npz_file(file_path):
    """
    Load data from npz file
    :param file_path: path to npz file with training data
    :return: input features and target data as numpy arrays
    """
    data = np.load(file_path)
    return data['input'], data['target']


def main(npz_data_file, epochs, lr, val):
    """
    Main function that performs training and test on a validation set
    :param npz_data_file: npz input file with training data
    :param epochs: number of epochs to train for
    :param lr: learning rate
    :param val: percentage of the training data to use as validation
    """

    input, target = load_data_from_npz_file(npz_data_file)
    N = input.shape[0]
    assert N == target.shape[0], \
        "The input and target arrays had different amounts of data ({} vs {})".format(N, target.shape[0]) # sanity check!
    print "Loaded {} training examples".format(N)

    # TODO. Complete. Implement code to train a network for image classification



if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", help="number of epochs for training",
                        type=int, default=50)
    parser.add_argument("--lr", help="learning rate for training",
                        type=float, default=50)
    parser.add_argument("--val", help="percent of training data to use for validation",
                        type=float, default=0.8)
    parser.add_argument("--input", help="input file (npz format)",
                        type=str)
    args = parser.parse_args()

    # run the main function
    main(args.input, args.epochs, args.lr, args.val)
    sys.exit(0)