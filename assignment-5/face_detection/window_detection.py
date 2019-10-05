#!/usr/bin/env python
# Script to run a model in a windowed approach on an input image
import os
import sys
import argparse
import numpy as np
import tensorflow as tf
import cv2
import copy

from train_face_detection import normalize_data_per_row


def non_max_suppression(boxes, overlapThresh):
    """
    Non maximum suppression from https://www.pyimagesearch.com/2015/02/16/faster-non-maximum-suppression-python/
    :param boxes: array of bounding boxes
    :param overlapThresh: overlap threshold
    :return: subset of boxes
    """
    # if there are no boxes, return an empty list
    if len(boxes) == 0:
        return []

    # if the bounding boxes integers, convert them to floats --
    # this is important since we'll be doing a bunch of divisions
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")

    # initialize the list of picked indexes
    pick = []

    # grab the coordinates of the bounding boxes
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    # compute the area of the bounding boxes and sort the bounding
    # boxes by the bottom-right y-coordinate of the bounding box
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)

    # keep looping while some indexes still remain in the indexes
    # list
    while len(idxs) > 0:
        # grab the last index in the indexes list and add the
        # index value to the list of picked indexes
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)

        # find the largest (x, y) coordinates for the start of
        # the bounding box and the smallest (x, y) coordinates
        # for the end of the bounding box
        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])

        # compute the width and height of the bounding box
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        # compute the ratio of overlap
        overlap = (w * h) / area[idxs[:last]]

        # delete all indexes from the index list that have
        idxs = np.delete(idxs, np.concatenate(([last],
                                               np.where(overlap > overlapThresh)[0])))

    # return only the bounding boxes that were picked using the
    # integer data type
    return boxes[pick]


def visualize_boxes(image, boxes):
    """
    Visualize the face detections as boxes overlayed on the input image
    :param image: input image
    :param boxes: face detections as bounding boxes of (min_x, min_y, max_x, max_y)
    """
    result = copy.deepcopy(image)
    for b in range(boxes.shape[0]):
        cv2.rectangle(result, (int(boxes[b,0]), int(boxes[b,1])), (int(boxes[b,2]), int(boxes[b,3])), (0, 255, 0), 3)
    cv2.imshow("Detections", result)

    cv2.waitKey(0)


def make_predictions(im, model, scales=[50], stride = 10):
    """
    Make predictions on the input image
    :param im: input image
    :param model: keras model
    :param scales: list of scales for the sliding window in pixels (e.g., [50] corresponds to one window of 50x50 pixels)
    :param stride: stride for the sliding window (in pixels)
    :return: numpy array of Nx5 dimension, where N is the number of detected faces (>=0) and the 5 columns
    correspond to: min_x, min_y, max_x, max_y, prob. The bounding box of a face is defined the by top-left
    corner (min_x,min_y) and the bottom-right corner (max_x, max_y).
    """

    # TO-DO. Remove the line below and complete the function so that it returns bounding boxes.
    return np.empty((0,5))


def main(input_file, weights_file):
    """
    Evaluate the model on the given input data
    :param input_file: npz data
    :param weights_file: h5 file with model definition and weights
    """

    # load the input image
    input_image = cv2.imread(input_file)
    assert input_image is not None, "Failed to load input image: {}".format(input_file)
    print "Loaded image with dimensions {}".format(input_image.shape)

    # resize the image in case it's huge. Max size is 600 pixels on the biggest dimension
    height, width, _ = input_image.shape
    if height > width:
        new_h = np.min([height, 600])
        new_w = int(new_h * width / height)
    else:
        new_w = np.min([width, 600])
        new_h = int(new_w * height / width)

    resized_im = cv2.resize(input_image, (new_w, new_h))

    # load the model
    model = tf.keras.models.load_model(weights_file)
    print "Loaded keras model from {}".format(weights_file)

    # make predictions on windows within the image
    boxes = make_predictions(resized_im, model, 
                             scales=[50, 100, 150, 200, 250])

    print "DETECTIONS:"
    print boxes

    # visualize the detections
    visualize_boxes(resized_im, boxes)

if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="input image",
                        type=str, required=True)
    parser.add_argument("--logs_dir", help="logs directory",
                        type=str, required=True)
    parser.add_argument("--weights_filename", help="name for the weights file",
                        type=str, default="weights.h5")
    args = parser.parse_args()

    weights_path = os.path.join(args.logs_dir, args.weights_filename)

    # run the main function
    main(args.input, weights_path)
    sys.exit(0)
