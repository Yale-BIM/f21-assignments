#!/usr/bin/env python
# Script to train and test a neural network with TF's Keras API for face detection

import os
import sys
import argparse
import numpy as np
import tensorflow as tf

from train_face_detection import normalize_data_per_row, load_data_from_npz_file

def main(input_file, weights_file, norm_file):
    """
    Evaluate the model on the given input data
    :param input_file: npz data
    :param weights_file: h5 file with model definition and weights
    :param norm_file: normalization params
    """
    # load data
    input, target = load_data_from_npz_file(input_file)
    N = input.shape[0]
    assert N == target.shape[0], \
        "The input and target arrays had different amounts of data ({} vs {})".format(N,
                                                                                      target.shape[0])  # sanity check!
    print "Loaded {} testing examples.".format(N)

    # normalize the inputs
    norm_input = normalize_data_per_row(input)

    # load keras model from file
    model = tf.keras.models.load_model(weights_file)

    # set loss for evaluation
    model.compile(optimizer=tf.keras.optimizers.Adam(),
                  loss='binary_crossentropy',
                  metrics=['binary_accuracy']) # accuracy threshold is 0.5

    # run evaluation
    eval_out = model.evaluate(x=norm_input, y=target, batch_size=input.shape[0])

    print "Loss = {}".format(eval_out[0])
    print "Accuracy = {}".format(eval_out[1])


if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="input file (npz format)",
                        type=str, required=True)
    parser.add_argument("--logs_dir", help="logs directory",
                        type=str, required=True)
    parser.add_argument("--norm_filename", help="name for the normalization params file",
                        type=str, default="normalization_params.npz")
    parser.add_argument("--weights_filename", help="name for the weights file",
                        type=str, default="weights.h5")
    args = parser.parse_args()

    weights_path = os.path.join(args.logs_dir, args.weights_filename)
    norm_path = os.path.join(args.logs_dir, args.norm_filename)

    # run the main function
    main(args.input, weights_path, norm_path)
    sys.exit(0)
