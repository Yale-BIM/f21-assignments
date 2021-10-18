# Training a Neural Network with Google Cloud

There are many ways to train models with Google Cloud, e.g., using a virtual machine in the Compute Engine, using the 
cloud ML Engine, etc. Here, we recommend that you use a Virtual Machine (VM) as that is the closest
setup to working with a local machine.

> All the students that are enrolled in CPSC-459/559 will receive a $50 credit for Google Cloud
to use for Assignment 4 or their final project. The credit will be associated
to the student's Yale email. Students should use the resources judiciously. 

## WARNING. Make Sure to Stop Your VM Instances!

Don’t forget to **stop your instance** when you are done 
(by clicking on the stop button at the top of the page showing your instances), 
otherwise you will run out of credits. 

<img src="http://cs231n.github.io/assets/sadpuppy_nocredits.png" width="300"/><br/>
(image from http://cs231n.github.io/gce-tutorial/)

## Getting Started with the Compute Engine of Google Cloud

1. If you don't have a Google Cloud account already, create one. Go to the http://cloud.google.com/
website, click on `Get started for free`, and **log in with your Yale email.** If you are creating a new account, select "Individual" as account type.

2. Once you have a Google Cloud account, go to your Console. Create a new project 
for the course assignments.
   
3. Finally, follow the steps in [this tutorial](https://cloud.google.com/compute/docs/quickstart-linux) to create a Virtual Machine. 
As a reference for Assignment 4, past editions of the course have used Virtual Machines from Google Cloud with the 
following specifications:

    - **Name:** bim-a4
    - **Region:** us-central1 (Iowa)
    - **Zone:** us-central1-c
    - **Machine Family:** General Purpose
    - **Generation:** First (Skylake CPU)
    - **Machine Type:** n1-standard-4 (4 vCPU, 15 GB Memory)
    - **GPU Type:** 1 x NVidia Tesla K80
    - **Book disk:** Ubuntu 20.04
    - **Boot disk type:** SSD persistent disk (50 GB)
    - **Firewall**: Select "Allow HTTP traffic" and "Allow HTTPS traffic"

    > The cost for such a virtual machine should be around $0.5 per hour.
    
4. You should then be able to connect to your virtual machine over SSH on your browser.

    > To connect directly through a shell in your local machine, you will need to install the
    [gcloud sdk](https://cloud.google.com/sdk/gcloud/) and follow [these instructions](https://cloud.google.com/compute/docs/instances/connecting-to-instance)
    (see the section on "Connecting using the command line interface").


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

Then open another terminal and run the command:
```bash
$ gcloud compute ssh <vm-name>  -- -NfL 6006:localhost:6006
```

Finally, in your browser navigate to the IP: 
```
http://localhost:6006/
```
    
<!-- Finally, get the external IP of your VM from the google cloud VM console. Connect from your
local machine by opening up the address http://<vm-external-ip>:6006/ in a browser. -->

