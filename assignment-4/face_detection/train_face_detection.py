#!/usr/bin/env python3
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


def normalize_data_per_row(data):
    """
    Normalize a give matrix of data (samples must be organized per row)
    :param data: input data as a numpy array with dimensions NxHxWxC
    :return: normalized data with pixel values in [0,1] (array with same dimensions as input)
    """

    # sanity checks!
    assert len(data.shape) == 4, "Expected the input data to be a 4D matrix"

    # TODO. Complete.


def main(npz_data_file, batch_size, epochs, lr, val, logs_dir):
    """
    Main function that performs training and test on a validation set
    :param npz_data_file: npz input file with training data
    :param batch_size: batch size to use at training time
    :param epochs: number of epochs to train for
    :param lr: learning rate
    :param val: percentage of the training data to use as validation
    :param logs_dir: directory where to save logs and trained parameters/weights
    """

    input, target = load_data_from_npz_file(npz_data_file)
    N = input.shape[0]
    assert N == target.shape[0], \
        "The input and target arrays had different amounts of data ({} vs {})".format(N, target.shape[0]) # sanity check!
    print("Loaded {} training examples.".format(N))

    # TODO. Complete. Implement code to train a network for image classification



if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", help="number of epochs for training",
                        type=int, default=50)
    parser.add_argument("--batch_size", help="batch size used for training",
                        type=int, default=100)
    parser.add_argument("--lr", help="learning rate for training",
                        type=float, default=1e-3)
    parser.add_argument("--val", help="percent of training data to use for validation",
                        type=float, default=0.8)
    parser.add_argument("--input", help="input file (npz format)",
                        type=str, required=True)
    parser.add_argument("--logs_dir", help="logs directory",
                        type=str, default="")
    args = parser.parse_args()

    if len(args.logs_dir) == 0: # parameter was not specified
        args.logs_dir = 'logs/log_{}'.format(datetime.datetime.now().strftime("%m-%d-%Y-%H-%M"))

    if not os.path.isdir(args.logs_dir):
        os.makedirs(args.logs_dir)

    # run the main function
    main(args.input, args.batch_size, args.epochs, args.lr, args.val, args.logs_dir)
    sys.exit(0)
