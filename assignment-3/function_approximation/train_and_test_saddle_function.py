#!/usr/bin/env python
# Script to train and test a neural network with TF's Keras API

import os
import sys
import argparse
import datetime
import numpy as np
import tensorflow as tf
import saddle_function_utils as sfu

def compute_normalization_parameters(data):
    """
    Compute normalization parameters (mean, st. dev.)
    :param data: matrix with data organized by rows [N x num_variables]
    :return: mean and standard deviation per variable as row matrices of dimension [1 x num_variables]
    """
    # TO-DO. Remove the two lines below and complete this function by computing the real mean and std. dev for the data
    mean = np.zeros((data.shape[1],))
    stdev = np.ones((data.shape[1],))

    return mean, stdev


def normalize_data_per_row(data, mean, stdev):
    """
    Normalize a give matrix of data (samples must be organized per row)
    :param data: input data
    :param mean: mean for normalization
    :param stdev: standard deviation for normalization
    :return: whitened data, (data - mean) / stdev
    """

    # sanity checks!
    assert len(data.shape) == 2, "Expected the input data to be a 2D matrix"
    assert data.shape[1] == mean.shape[1], "Data - Mean size mismatch ({} vs {})".format(data.shape[1], mean.shape[1])
    assert data.shape[1] == stdev.shape[1], "Data - StDev size mismatch ({} vs {})".format(data.shape[1], stdev.shape[1])

    # TODO. Complete. Replace the line below with code to whitten the data.
    normalized_data = data

    return normalized_data


def build_linear_model(num_inputs):
    """
    Build NN model with Keras
    :param num_inputs: number of input features for the model
    :return: Keras model
    """
    # TO-DO: Complete. Remove the None line below, define your model, and return it.
    return None


def train_model(model, train_input, train_target, val_input, val_target, input_mean, input_stdev,
                epochs=20, learning_rate=0.01, batch_size=16):
    """
    Train the model on the given data
    :param model: Keras model
    :param train_input: train inputs
    :param train_target: train targets
    :param val_input: validation inputs
    :param val_target: validation targets
    :param input_mean: mean for the variables in the inputs (for normalization)
    :param input_stdev: st. dev. for the variables in the inputs (for normalization)
    :param epochs: epochs for gradient descent
    :param learning_rate: learning rate for gradient descent
    :param batch_size: batch size for training with gradient descent
    """
    # TO-DO. Complete. Remove the pass line below, and add the necessary training code.
    pass


def test_model(model, test_input, test_target, input_mean, input_stdev, batch_size=60):
    """
    Test a model on a given data
    :param model: trained model to perform testing on
    :param test_input: test inputs
    :param test_target: test targets
    :param input_mean: mean for the variables in the inputs (for normalization)
    :param input_stdev: st. dev. for the variables in the inputs (for normalization)
    :return: predicted targets for the given inputs
    """
    # TODO. Complete. Remove the return line below and add the necessary code to make predictions with the model.
    return np.zeros(test_target.shape)


def compute_average_L2_error(test_target, predicted_targets):
    """
    Compute the average L2 error for the predictions
    :param test_target: matrix with ground truth targets [N x 1]
    :param predicted_targets: matrix with predicted targets [N x 1]
    :return: average L2 error
    """
    # TO-DO. Complete. Replace the line below with code that actually computes the average L2 error over the targets.
    average_l2_err = 0

    return average_l2_err


def main(num_examples, epochs, lr, visualize_training_data, build_fn=build_linear_model):
    """
    Main function
    :param num_training: Number of examples to generate for the problem (including training, testing, and val)
    :param epochs: number of epochs to train for
    :param lr: learning rate
    :param visualize_training_data: visualize the training data?
    """

    np.random.seed(0) # make the generated values deterministic. do not change!
    
    # generate data
    monkey_function = lambda x: np.power(x[0], 3) - 3*x[0]*np.power(x[1],2)
    input, target = sfu.generate_data_for_2D_function(monkey_function, N=num_examples)

    # split data into training (70%) and testing (30%)
    all_train_input, all_train_target, test_input, test_target = sfu.split_data(input, target, 0.6)

    # visualize all training/testing (uncomment if you want to visualize the whole dataset)
    # plot_train_and_test(all_train_input, all_train_target, test_input, test_target, "train", "test", title="Train/Test Data")

    # split training data into actual training and validation
    train_input, train_target, val_input, val_target = sfu.split_data(all_train_input, all_train_target, 0.8)

    # visualize training/validation (uncomment if you want to visualize the training/validation data)
    if visualize_training_data:
        sfu.plot_train_and_test(train_input, train_target, val_input, val_target, "train", "validation", title="Train/Val Data")

    # normalize input data and save normalization parameters to file
    mean, stdev = compute_normalization_parameters(train_input)

    # build the model
    model = build_fn(train_input.shape[1])

    # train the model
    print "\n\nTRAINING..."
    train_model(model, train_input, train_target, val_input, val_target, mean, stdev, epochs=epochs, learning_rate=lr)

    # test the model
    print "\n\nTESTING..."
    predicted_targets = test_model(model, test_input, test_target, mean, stdev)

    # Report average L2 error
    l2_err = compute_average_L2_error(test_target, predicted_targets)
    print "L2 Error on Testing Set: {}".format(l2_err)

    # visualize the result (uncomment the line below to plot the predictions)
    # sfu.plot_test_predictions(test_input, test_target, predicted_targets, title="Predictions")



if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", help="total number of examples (including training, testing, and validation)",
                        type=int, default=600)
    parser.add_argument("--epochs", help="number of epochs for training",
                        type=int, default=50)
    parser.add_argument("--lr", help="learning rate for training",
                        type=float, default=50)
    parser.add_argument("--visualize_training_data", help="visualize training data",
                        action="store_true")
    parser.add_argument("--build_fn", help="model to train (e.g., 'linear')",
                        type=str, default="linear")
    args = parser.parse_args()

    # define the model function that we will use to assemble the Neural Network
    if args.build_fn == "linear":
        build_fn = build_linear_model # function that builds linear model
    else:
        print "Invalid build function name {}".format(args.build_fn)
        sys.exit(1)

    # run the main function
    main(args.n, args.epochs, args.lr, args.visualize_training_data, build_fn=build_fn)
    sys.exit(0)