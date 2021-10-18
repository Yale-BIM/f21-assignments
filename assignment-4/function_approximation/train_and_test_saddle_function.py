#!/usr/bin/env python3
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

    mean = np.mean(data, axis=0)
    stdev = np.std(data, axis=0)

    # transpose mean and stdev in case they are (2,) arrays
    if len(mean.shape) == 1:
        mean = np.reshape(mean, (1,mean.shape[0]))
    if len(stdev.shape) == 1:
        stdev = np.reshape(stdev, (1,stdev.shape[0]))

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

    centered = data - np.tile(mean, (data.shape[0], 1))
    normalized_data = np.divide(centered, np.tile(stdev, (data.shape[0],1)))

    return normalized_data


def build_linear_model(num_inputs):
    """
    Build NN model with Keras
    :param num_inputs: number of input features for the model
    :return: Keras model
    """
    input = tf.keras.layers.Input(shape=(num_inputs,), name="inputs")
    hidden1 = tf.keras.layers.Dense(64, use_bias=True)(input)
    hidden2 = tf.keras.layers.Dense(128, use_bias=True)(hidden1)
    output = tf.keras.layers.Dense(1, use_bias=True)(hidden2)
    model = tf.keras.models.Model(inputs=input, outputs=output, name="monkey_model")
    return model


def build_nonlinear_model(num_inputs):
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

    # normalize
    norm_train_input = normalize_data_per_row(train_input, input_mean, input_stdev)
    norm_val_input = normalize_data_per_row(val_input, input_mean, input_stdev)

    # compile the model: define optimizer, loss, and metrics
    model.compile(optimizer=tf.keras.optimizers.Adam(lr=learning_rate),
                 loss='mse',
                 metrics=['mae'])

    # TODO - Create callbacks for saving checkpoints and visualizing loss on TensorBoard

    # do training for the specified number of epochs and with the given batch size
    # TODO - Add callbacks to fit funciton
    model.fit(norm_train_input, train_target, epochs=epochs, batch_size=batch_size,
             validation_data=(norm_val_input, val_target))


def test_model(model, test_input, input_mean, input_stdev, batch_size=60):
    """
    Test a model on a given data
    :param model: trained model to perform testing on
    :param test_input: test inputs
    :param input_mean: mean for the variables in the inputs (for normalization)
    :param input_stdev: st. dev. for the variables in the inputs (for normalization)
    :return: predicted targets for the given inputs
    """
    # normalize
    norm_test_input = normalize_data_per_row(test_input, input_mean, input_stdev)

    # evaluate
    predicted_targets = model.predict(norm_test_input, batch_size=batch_size)

    return predicted_targets


def compute_average_L2_error(test_target, predicted_targets):
    """
    Compute the average L2 error for the predictions
    :param test_target: matrix with ground truth targets [N x 1]
    :param predicted_targets: matrix with predicted targets [N x 1]
    :return: average L2 error
    """
    diff = predicted_targets - test_target
    l2_err = np.sqrt(np.sum(np.power(diff, 2), axis=1))
    assert l2_err.shape[0] == predicted_targets.shape[0], \
        "Invalid dim {} vs {}".format(l2_err.shape, predicted_targets.shape)
    average_l2_err = np.mean(l2_err)

    return average_l2_err


def main(num_examples, epochs, lr, visualize_training_data, build_fn=build_linear_model, batch_size=16):
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
    print("\n\nTRAINING...")
    train_model(model, train_input, train_target, val_input, val_target, mean, stdev,
                epochs=epochs, learning_rate=lr, batch_size=batch_size)

    # test the model
    print("\n\nTESTING...")
    predicted_targets = test_model(model, test_input, mean, stdev)

    # Report average L2 error
    l2_err = compute_average_L2_error(test_target, predicted_targets)
    print("L2 Error on Testing Set: {}".format(l2_err))

    # visualize the result
    sfu.plot_test_predictions(test_input, test_target, predicted_targets, title="Predictions")


if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", help="total number of examples (including training, testing, and validation)",
                        type=int, default=600)
    parser.add_argument("--batch_size", help="batch size used for training",
                        type=int, default=16)
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
        print("Invalid build function name {}".format(args.build_fn))
        sys.exit(1)

    # run the main function
    main(args.n, args.epochs, args.lr, args.visualize_training_data, build_fn=build_fn, batch_size=args.batch_size)
    sys.exit(0)
