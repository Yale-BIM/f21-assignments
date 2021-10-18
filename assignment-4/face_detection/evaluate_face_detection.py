#!/usr/bin/env python3
# Script to train and test a neural network with TF's Keras API for face detection

import os
import sys
import argparse
import numpy as np
import tensorflow as tf

from train_face_detection import normalize_data_per_row, load_data_from_npz_file


def evaluate(input_features, target, model):
    """
    Helper function to evaluate model
    :param input_features: input tensor
    :param target: target tensor
    :param model: Keras model
    """
    # normalize the inputs
    norm_input = normalize_data_per_row(input_features)

    # set loss for evaluation
    model.compile(optimizer=tf.keras.optimizers.Adam(),
                  loss='binary_crossentropy',
                  metrics=['binary_accuracy'])  # accuracy threshold is 0.5

    # run evaluation
    eval_out = model.evaluate(x=norm_input, y=target, batch_size=input_features.shape[0])

    loss = eval_out[0]
    accuracy = eval_out[1]
    return loss, accuracy


def main(input_file, weights_file):
    """
    Evaluate the model on the given input data
    :param input_file: npz data
    :param weights_file: path to h5 file with model definition and weights
    """
    # load data
    input_features, target = load_data_from_npz_file(input_file)
    N = input_features.shape[0]

    # sanity check!
    assert N == target.shape[0], \
        "Error: The input and target arrays had different amounts of data ({} vs {})".format(N, target.shape[0])

    print("Loaded {} testing examples.".format(N))

    # load keras model from file
    model = tf.keras.models.load_model(weights_file)

    # compute results
    loss, accuracy = evaluate(input_features, target, model)

    print("Loss = {}".format(loss))
    print("Accuracy = {}".format(accuracy))


if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="input file (npz format)",
                        type=str, required=True)
    parser.add_argument("--weights-path", help="path for the weights file",
                        type=str, required=True)
    args = parser.parse_args()
    
    # run the main function
    main(args.input, args.weights_path)
    sys.exit(0)
