# Utility functions for the train_and_test_saddle_function.py script

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def generate_data_for_2D_function(function, N=1000, domain=[20, 100]):
    """
    Generate training data for a function with 2 inputs
    :param function: function from which to generate the data
    :param num_inputs: number of inputs to the function
    :param N: number of samples to generate
    :param domain: domain for each of the input variables, [-domain*0.5, domain*0.5]
    :return: input (Nxnum_inputs), target (Nxnum_outputs) pairs
    """

    # create inputs from uniform distribution in [-domain*0.5, domain*0.5]
    num_inputs = 2
    inputs = np.random.rand(N, num_inputs)

    for i in range(inputs.shape[1]):
        inputs[:,i] = inputs[:,i]*domain[i] - domain[i]*0.5

    # compute target for each input
    targets = [function(inputs[x,:]) for x in range(N)]
    targets = np.vstack(targets)

    return inputs, targets


def split_data(input, target, train_percent):
    """
    Split the input and target data into two sets
    :param input: inputs [Nx2] matrix
    :param target: target [Nx1] matrix
    :param train_percent: percentage of the data that should be assigned to training
    :return: train_input, train_target, test_input, test_target
    """
    assert input.shape[0] == target.shape[0], \
        "Number of inputs and targets do not match ({} vs {})".format(input.shape[0], target.shape[0])

    indices = list(range(input.shape[0]))
    np.random.shuffle(indices)

    num_train = int(input.shape[0]*train_percent)
    train_indices = indices[:num_train]
    test_indices = indices[num_train:]

    return input[train_indices, :], target[train_indices,:], input[test_indices,:], target[test_indices,:]


def plot_2D_function_data(ax, inputs, targets, label, color='k'):
    """
    Method that generates scatter plot of inputs and targets
    :param inputs: inputs [Nx2] matrix
    :param targets: target [Nx1] matrix
    :param label: label for the legend
    :param color: points color
    """
    ax.scatter(inputs[:,0], inputs[:, 1], targets, s=10, c=color, label=label)


def plot_train_and_test(train_input, train_target, test_input, test_target, train_label, test_label, title="Scatter Plot"):
    """
    Method that plots two sets of data (train and test)
    :param train_input: inputs [Nx2] matrix
    :param train_target: target [Nx1] matrix
    :param test_input: inputs [Nx2] matrix
    :param test_target: target [Nx1] matrix
    :param train_label: label for the train data
    :param test_label: label for the test data
    :param title: plot title
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plot_2D_function_data(ax, train_input, train_target, train_label, 'k')
    plot_2D_function_data(ax, test_input, test_target, test_label, 'r')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title(title)
    ax.legend()
    plt.show()


def plot_test_predictions(test_input, test_target, predicted_targets, title="Predictions"):
    """
    Method that plots the ground truth targets and predictions
    :param test_input: input values as an [Nx2] matrix
    :param test_target: ground truth target values as a [Nx1] matrix
    :param predicted_targets: predicted targets as a [Nx1] matrix
    :param title: plot title
    """
    fig = plt.figure(figsize=(16, 4))

    ax = fig.add_subplot(131, projection='3d')
    plot_2D_function_data(ax, test_input, test_target, "Ground Truth", 'b')
    plot_2D_function_data(ax, test_input, predicted_targets, "Predicted", 'r')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()
    ax.set_title(title)

    ax = fig.add_subplot(132)
    ax.scatter(test_input[:,0], test_target, s=10, c='b')
    ax.scatter(test_input[:,0], predicted_targets, s=10, c='r')
    ax.set_xlabel('x')
    ax.set_ylabel('z')
    ax.set_title('y=0 projection')

    ax = fig.add_subplot(133)
    ax.scatter(test_input[:,1], test_target, s=10, c='b')
    ax.scatter(test_input[:,1], predicted_targets, s=10, c='r')
    ax.set_xlabel('y')
    ax.set_ylabel('z')
    ax.set_title('x=0 projection')

    plt.tight_layout()
    plt.show()