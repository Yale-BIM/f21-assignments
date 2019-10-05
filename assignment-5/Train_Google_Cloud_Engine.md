# Training a Neural Network with Google Cloud

There are many ways to train models with Google Cloud, e.g., using a virtual machine in the Compute Engine, using the 
cloud ML Engine, etc. Here, we recommend that you use a Virtual Machine (VM) as that is the closest
setup to working with a local machine.

> All the students that are enrolled in CPSC-459/559 will receive a $50 credit for Google Cloud
to use for assignment 5 or their final project. The credit will be associated
to the student's Yale email. Students should use the resources judiciously. 

## WARNING. Make Sure to Stop Your VM Instances!

Don’t forget to **stop your instance** when you are done 
(by clicking on the stop button at the top of the page showing your instances), 
otherwise you will run out of credits. 

<img src="http://cs231n.github.io/assets/sadpuppy_nocredits.png" width="300"/><br/>
(image from http://cs231n.github.io/gce-tutorial/)

## Getting Started with the Compute Engine of Google Cloud

1. If you don't have a Google Cloud account already, create one. Go to the http://cloud.google.com/
website and clock on "Compute. Build smarter, build faster". In the next page, click on `TRY IT FREE`
and **log in with your Yale email.** If you are creating a new account, select "Individual" as account type.

2. Once you have a Google Cloud account, go to your Console. Create a new project 
for the course assignments.
   
3. Finally, follow the steps in [this tutorial](https://cloud.google.com/compute/docs/quickstart-linux) to create a Virtual Machine. 
To work on the assignment-3, we suggest that you use the following custom configuration for your machine:

    - **Name:** bim-assignments
    - **Region:** us-east1 (South Carolina)
    - **Zone:** us-east1-c
    - **Machine Type:** 4 vCPU, 16 GB Memory
    - **GPUs:** 1 x NVidia Tesla K80
    - **Book disk:** cpsc659-a3-tensorflow (should be found under the project "CPSC659-A3" in the "Custom Images" options)
    - **Boot disk type:** Standard persistent disk (30 GB)
    - **Firewall**: Select "Allow HTTP traffic" and "Allow HTTPS traffic"

    > The cost for such a virtual machine should be around $0.50 per hour.
    
4. You should then be able to connect to your virtual machine over SSH on your browser.

    > To connect directly through a shell in your local machine, you will need to install the
    [gcloud sdk](https://cloud.google.com/sdk/gcloud/) and follow [these instructions](https://cloud.google.com/compute/docs/instances/connecting-to-instance)
    (see the section on "Connecting using the command line interface").
    
## The cpsc659-a3-tensorflow image

You can use the `cpsc659-a3-tensorflow` image in Google Cloud to work on the course's Assignment 5. 
This boot disk image is already set up with:

- Tensorflow 1.11
- CUDA 10
- Opencv
- Matplotlib
    
The image also has the 64x64_data.npz dataset for Part III of the assignment
in the folder /home/marynel_vazquez. You don't need to waste time (and credits) 
downloading the data again into your VM. Instead, move the data to your own home folder
once you SSH into your VM:

```bash
# check the data is there
$ ls /home/marynel_vazquez/ 
64x64_data.npz

# go to your home
$ cd 

# move the data to your home
$ sudo mv /home/marynel_vazquez/64x64_data.npz . 

# change permissions for the data (use your username instead of <user> below)
$ chown <user>:<user> 64x64_data.npz 
```

> Note that you also don't need to create a virtual environment to work on your
assignment in the VM. You have sudo accesss and can install any Python library that you need in the
whole system.

## Working With Your VM in Google Cloud

Once you've created your VM in Google Cloud. You can download your code from Gitlab as usual
and commit back things to your repository as necessary. 

### TensorBoard 
It is also possible to use port forwarding (as discussed in [here](https://stackoverflow.com/questions/45060922/tensorboard-execution-from-google-cloud-machine)) to 
run TensorBoard on a Google Cloud VM and visualize your results on a local browser.
To succeed running the steps below, you will need to install the [Google Cloud (gcloud) sdk](https://cloud.google.com/sdk/gcloud/).

Run the following commands on a terminal in your host machine:

1.  Authenticate.
    ```bash
    $ gcloud auth login
    ```
    
2. Set firewall rules for allowing connections to your VM through the 6006 port.
    ```bash
    $ gcloud compute firewall-rules create tensorboard-port --allow tcp:6006
    ```
    
3. Configure port forwarding.
    ```bash
    $ gcloud compute ssh <vm-name> --ssh-flag="-R" --ssh-flag="6006:localhost:6006"
    ```
    
Then, start tensorboard on the VM in the port 6006:

```bash
$ tensorboard --logdir <path_to_logs> --port 6006
```
    
Finally, get the external IP of your VM from the google cloud VM console. Connect from your
local machine by opening up the address http://<vm-external-ip>:6006/ in a browser.

