# Training a Neural Network with Amazon Web Services (AWS)

There are many ways to train models with AWS, e.g., using a virtual machine through its Elastic Computing (EC2) services, 
using its machine learning tools like Amazon Sage Maker, etc. Here, we recommend that you use a Virtual Machine (VM) as 
that is the closest setup to working with a local machine.

> All the students that are enrolled in CPSC-459/559 who received a $200 credit for AWS
to use for the course should associate this credit with their student's Yale email. 
Students should use the resources judiciously. 

## WARNING. Make Sure to Stop Your VM Instances!

Don’t forget to **stop your instance** when you are done 
(by clicking on the stop button at the top of the page showing your instances), 
otherwise you will run out of credits. 

## Getting Started with the EC2 services of AWS

1. If you don't have an AWS account already, create one. Go to the http://aws.amazon.com/
website and follow the setup instructions in this [Google doc](https://docs.google.com/document/d/1hrnJktITAjcURVBnWOO5fvQajmOTeLyP6You9JPYY1o/edit?usp=sharing). The document also provides
details on exporting the desktop of a virtual machine in case you need that.

2. Once you have an AWS account, go to Services>EC2. Create a new EC2 instance, e.g., by clicking on
the orange button that says "Launch instance".
   
3. Finally, follow the steps in [this tutorial](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/EC2_GetStarted.html) 
to create a Virtual Machine. To work on the assignment-5, we suggest that you use 
a machine with a gpu (like P2 or G4 instances). See [this page](https://docs.aws.amazon.com/dlami/latest/devguide/gpu.html)
for more details.
    
4. Once your EC2 instance is created, you should then be able to connect to your virtual machine over SSH on your browser.

<!--
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
-->
