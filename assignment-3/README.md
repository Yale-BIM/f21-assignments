# Assignment 2

This is the fourth assignment for Yale's CPSC-659 Building Interactive Machines course.

## Table of Contents


## Introduction 
This assignment will provide you practical experience with deep learning. In particular, you'll become
familiar with [TensorFlow's Keras API](https://www.tensorflow.org/guide/keras).


#### System Requirements
As for the prior assignments, you should have access to a computer with `Ubuntu 16.04` and `ROS Kinetic` to complete the homework. 

> NOTE: If you have Ubuntu 18.04, you can also complete this homework 
using ROS Melodic. 

You should also have `git` installed in the machine that you are using to work on your assignment.
You will use git to save your work to your [GitLab](http://www.gitlab.com) repository.


#### Background Knowledge

This assignment assumes that you have already completed the prior assignments and, thus, you
have set up your catkin workspace. You are also expected to have experience with Linux shells 
(e.g., [bash](https://www.gnu.org/software/bash/)), [git](https://git-scm.com/), and
the [Robot Operating System (ROS)](http://www.ros.org/). This includes being familiar with
the `roscore`, `rosrun`, `roslaunch`, `rosbag`, `rostopic`, `rosmsg`, `rosnode`, `rqt_image_view`, 
and `rviz` tools. 

You are also expected to be familiar with the [numpy Python library](http://www.numpy.org/) for linear algebra. 
If you are not, please check [this tutorial](https://docs.scipy.org/doc/numpy/user/quickstart.html) before starting the assignment.

#### Notation
We refer to `vectors` or column matrices with bold lower-case letters (e.g., $`\bold{x}`$).
Other `matrices`, such as linear transformations, and `scalars` are written with regular
font weight. 


#### Deliverables

- **Report:** You are expected to submit a pdf to Canvas with answers to the questions/tasks at 
the end of each part of the assignment. This report should also have any information needed 
to understand and/or run your code, as well as the specific commit SHA of the version of the code
that you would like to be evaluated on. Though not mandatory, it is recommended that you generate this pdf 
with [Overleaf](https://www.overleaf.com/edu/yale#!overview) and this 
[simple assignment template](https://www.overleaf.com/latex/templates/simple-assignment-template/mzkqqqjypzvd) 
in [LaTeX](https://www.latex-project.org/).

- **Code:** Finally, you are expected to push code for this assignment to your 
[GitLab](http://www.gitlab.com) repository as indicated in the [general instructions](../README.md) 
document for CPSC-659 assignments. 


#### Evaluation

You assignment will be evaluated based on the content of your report and your code.


#### Further Reading




## Part I. Set Up TensorFlow Locally

The first thing that you will need to start working on deep learning is installing TensorFlow in the
machine that you are using to develop code. Follow the instructions below, which are based on the [official 
TensorFlow installation page](https://www.tensorflow.org/install/pip?lang=python2), 
to set up TensorFlow v. 1.11 with Python 2.7.

1. Check that [virtualenv](https://virtualenv.pypa.io/en/stable/) is already installed:

    ```bash
    $ virtualenv --version
    ```
    
    If virtualenv is NOT installed in your machine, install it:
    
    ```bash
    sudo pip install -U virtualenv # system-wide install
    ```
    
    And if pip is not installed in your machine, install it as well:
    
    ```bash
    sudo apt install python-dev python-pip
    ```
    
    and try repeating the steps above afterwards so that you install virtualenv.
    
    
2. Create a [virtual environment](https://realpython.com/python-virtual-environments-a-primer/) 
named `venv` with virtualenv:

    ```bash
    $ cd assignment-3 # enter this assignments directory within your assignments private repository
    $ virtualenv --system-site-packages -p python2.7 venv
    ```
    
    > NOTE: The system-site-packages makes your newly created virtual environment inherit packages
    from your python system-wide installation (i.e., wherever your global site-packages directory is).
    If you have installed ROS, the system-site-packages option would include ROS python packages 
    into your virtual environment.
     
3. Activate your virtual environment:

   ```bash
   $ source ./venv/bin/activate
   ```
   
   Now your terminal prompt should start with `(venv)` indicating that you are within the environment.
    
4. Upgrade pip within your virtual environment:
   
   ```bash
   (venv) $ pip install --upgrade pip
   ```
   
5. Install TensorFlow (TF):

    - If your machine has no GPU:
    
        ```bash
        (venv) $ pip install --upgrade tensorflow
        ```
        
    - If your machine has a GPU with CUDA Compute Capability 3.5 or higher and 
    you have CUDA 9.0 installed in your system (see [here](https://www.tensorflow.org/install/gpu) for 
    more details on setting up CUDA for TF):
    
        ```bash
        (venv) $ pip install --upgrade tensorflow-gpu
        ```
        
6. Verify the install:

    ```bash
    (venv) $ python -c "import tensorflow as tf; print(tf.__version__)"
    ```
    
    The command should print "1.11.0".


> NOTE: Because you have installed TF within a virtual environment, you will always
have to activate the environment before starting to run your deep learning code for this
assignment (as in step 3 above). To exit your virtual environment at any time, you can run the command: 
``` (venv) $ deactivate ```.

## Questions / Tasks 

1. In general, committing virtual environments to your repository is bad practice (e.g., paths might differ
in different computers and this may render your environment unusable). Instead, what people generally
do is create a requirements.txt file with all of the dependencies for a project. This file can then be 
used to install all required Python models with pip.

    For this part of the assignment, create a requirements.txt file so that you remember what needs
    to be installed to run your assignment. The file should be placed within the assignment-3 directory
    of your private assignments repository. Below is an example of how the requirements.txt file would 
    look like:
    
    ```text
    tensorflow==1.11.0
    ```
    
    or if you installed `tensorflow-gpu` before,
    
    ```text
    tensorflow-gpu==1.11.0
    ```
    
    Commit the requirements.txt file to your private assignments repository. 
    
    > NOTE: If you ever need to install the packages in a new 
    virtual environment, then you can just run the command `(env) $ pip install -r requirements.txt`
    within your virtual environment.


# Part II. Approximating a Non-Linear Function

Read the [Primer on Universal Function Approximation with Deep Learning](https://cartesianfaith.com/2016/09/23/a-primer-on-universal-function-approximation-with-deep-learning-in-torch-and-r/) 
by Brian Yung Rowe. 

Once you've read the primer, you should complete the tasks below to approximate the 
[monkey saddle surface](https://en.wikipedia.org/wiki/Monkey_saddle)  defined by the equation 
$`z = x^3 - 3xy^2`$. Your code should leverage [TensorFlow's
Keras API](https://www.tensorflow.org/guide/keras).

To get you started, this assignment provides two files within the 
`assignment-3/function_approximation` directory:

- *train_and_test_saddle_function.py:* main file that you will complete in this part of the assignment.
- *saddle_function_utils:* code to generate data and help you visualize results.

If you run the train_and_test_saddle_function.py script (which is incomplete at this point)
with the `visualize_training_data` option, you should be able to visualize the data that the
script generates for you:

```bash
$ cd assignment-3/function_approximation
$ ./train_and_test_saddle_function.py --visualize_training_data
```

When you run the script, a [matplotlib](https://matplotlib.org/) plot of the data should appear, 
as in the figure below:

<img src="docs/training-val-data.png" width="600"/>

## Questions/Tasks

1. Complete the `compute_normalization_parameters()` and `normalize_data_per_row()` functions in
the train_and_test_saddle_function.py script.

    a. For the compute_normalization_parameters() function, you want to compute the mean
    and standard deviation for the data that is input to the function:
    
    ```python
    def compute_normalization_parameters(data):
        """
        Compute normalization parameters (mean, st. dev.)
        :param data: matrix with data organized by rows [N x num_variables]
        :return: mean and standard deviation per variable as row matrices of dimension [1 x num_variables]
        """
        mean = ...  # numpy array with num_variables elements 
        stdev = ... # numpy array with num_varianles elements
        return mean, stdev
    ```
    
    Note that the input `data`
    is organized in a matrix where each row corresponds to an example. The columns of the data
    matrix correspond to features of each of the input examples. The mean and standard deviation 
    should be computed for each example feature independently.
    
    b. For the normalize_data_per_row() function, you want to use the mean and stdev from
    (a) above to apply a whitening transformation to the data:
    
    ```python
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
    
        normalized_data = ... # Complete.
        
        return normalized_data
    ``` 
     
    For example, if an example
    feature is $`x`$, then you want to transform it into $`(x - \mu)/\sigma`$,
    where $`\sigma`$ corresponds to the standard deviation for that feature.
    
2. Complete the `build_model()` function in
the train_and_test_saddle_function.py script. This function should implement
a simple Neural Network model (with one hidden layer) using the [Keras API](https://www.tensorflow.org/guide/keras#functional_api):

    ```python
    input = tf.keras.layers.Input(shape=(num_inputs,), name="inputs")
    hidden1 = tf.keras.layers.Dense(64, use_bias=True)(input)
    output = tf.keras.layers.Dense(1, use_bias=True)(hidden1)
    model = tf.keras.models.Model(inputs=input, outputs=output, name="monkey_model")
    ```

    > In general, we suggest that you use [TF's Keras Functional API](https://www.tensorflow.org/guide/keras#functional_api)
    to build your model as in the script above.

3. Complete the `train_model()` function in
the train_and_test_saddle_function.py script. This function should first normalize the input
features in the training and validation set using the normalize_data_per_row() function from step 1 above. 
Then, train_model() should [compile](https://www.tensorflow.org/api_docs/python/tf/keras/models/Model#compile) 
the neural network model that is passed as input to the function, i.e., 
define the optimizer to be used during training, loss, and relevant metrics. Finally, the train_model() 
function should train the network's weights using the [model's fit function](https://www.tensorflow.org/api_docs/python/tf/keras/models/Model#fit).

    ```python
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
    
        # tensorboard callback
        logs_dir = 'logs/log_{}'.format(datetime.datetime.now().strftime("%m-%d-%Y-%H-%M"))
        tbCallBack = tf.keras.callbacks.TensorBoard(log_dir=logs_dir, write_graph=True)
    
        checkpointCallBack = tf.keras.callbacks.ModelCheckpoint(os.path.join(logs_dir,'best_monkey_weights.h5'),
                                                                monitor='val_loss',
                                                                verbose=0,
                                                                save_best_only=True,
                                                                save_weights_only=False,
                                                                mode='auto',
                                                                period=1)
    
        # do trianing for the specified number of epochs and with the given batch size
        model.fit(norm_train_input, train_target, epochs=epochs, batch_size=batch_size,
                 validation_data=(norm_val_input, val_target),
                 callbacks=[tbCallBack, checkpointCallBack])
    ```

    Note that the example above also adds two callbacks to the fit() function:
    
    - **tf.keras.callbacks.TensorBoard:** [TensorBoard](https://www.tensorflow.org/guide/summaries_and_tensorboard) 
    callback to write TensorBoard logs to a given directory.
    - **tf.keras.callbacks.ModelCheckpoint:** Callback that saves the model after every epoch (see
    more information [here](https://www.tensorflow.org/api_docs/python/tf/keras/callbacks/ModelCheckpoint)).
    Because we set "save_best_only = True", the callback would only save the model if the
    validation loss is smaller than the prior best validation loss.
    
    We suggest that, as a first try, you use the [Adam optimizer](https://www.tensorflow.org/api_docs/python/tf/keras/optimizers/Adam)
    when you train neural works with gradient descent. The optimizer tends to work well for many problems. You can read the
    original paper with full details of how it works here: [Diederik P. Kingma, Jimmy Ba. Adam: A Method for Stochastic Optimization](https://arxiv.org/abs/1412.6980).
    
4. Complete the `test_model()` function in the train_and_test_saddle_function.py script. The function
should output predictions for the given input matrix (test_input) using the `model.predict()` function.
The official documentation for the predict() function can be found [here](https://www.tensorflow.org/api_docs/python/tf/keras/models/Model#predict).

5. Complete the `compute_average_L2_error()` function in the train_and_test_saddle_function.py script.
The function should compute the average L2-norm (or least squares) difference between the ground truth 
(test_target) and the predicted values (predicted_targets) that are input to the function. The average
should be over all of the examples in each of the input matrices.

    ```python
    def compute_average_L2_error(test_target, predicted_targets):
        """
        Compute the average L2 error for the predictions
        :param test_target: matrix with ground truth targets [N x 1]
        :param predicted_targets: matrix with predicted targets [N x 1]
        :return: average L2 error
        """
        
        average_l2_err = ... # Complete.
    
        return average_l2_err
    ```

6. Uncomment the last line in the main() function of the train_and_test_saddle_function.py
script so that you can easily visualize the predictions made by your model:

    ```python
    # visualize the result (uncomment the line below to plot the predictions)
    sfu.plot_test_predictions(test_input, test_target, predicted_targets, title="Predictions")
    ```

    You should now be able to train your network and visualize the results for the test set:

    ```bash
    (venv) $ ./train_and_test_saddle_function.py [--lr 1e-1] [--epochs 10]
    ```    
    
    The optional parameters `lr` and `epochs` correspond to the learning rate and number of 
    epochs to use for training.
    
    Make a screen shot of the plot that you get after training your simple neural network for the
    first time with a learning rate of 1e-2 and for 100 epochs. Add the screen shot to your report.
    
7. Visualize the learning curves and your model using [TensorBoard](https://www.tensorflow.org/guide/summaries_and_tensorboard).
Open a new terminal window, activate your virtual environment, and run:

    ```bash
    (venv) $ cd assignment-3 # go to the assignment-3 directory within your private repository
    (venv) $ tensorboard --logdir function_approximation/logs
    ```
    
    Then, go to the URL that the script provides (e.g., http://localhost:6006) in your favorite
    browser. The `SCALARS` tab of the TensorBoard interface should then show various training curves
    (e.g., epoch_loss for the loss after every epoch in the training set, and epoch_val_loss for the
    loss in the validation set). The `GRAPHS` tab of the TensorBoard interface should show a 
    [computation graph](https://www.tensorflow.org/guide/graph_viz) for your simple neural network model.
    
    Make a screen shot of your computation graph and include it in your project report.
    