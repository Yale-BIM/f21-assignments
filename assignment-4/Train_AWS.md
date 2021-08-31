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

## Working With Your EC2 Instance

Once you've created your EC2 instance, you can download your code from Gitlab as usual
and commit back things to your repository as necessary. 

### TensorBoard 

To set up TensorBoard, we will need to find the public ip address of your new instance and enable port forwarding to 

#### Find External IP of Instance

First, we need to set up the AWS command-line interface (CLI) to enable additional features around your new instance.

1. Install the AWS CLI by following [these instructions](https://docs.aws.amazon.com/cli/latest/userguide/install-cliv2.html), choosing the option for your operating system.

2. Get the name of your instance.
   - Go to the [AWS EC2 console](https://console.aws.amazon.com/ec2/v2/home?region=us-east-1#Home:).
   - Click on Instances.
   - Find the instance you just created in the table.
   - Copy the entry under Instance ID.

3. Get public IP of instance by running
   ```bash
   aws ec2 describe-instances --instance-ids <instance_id> --query 'Reservations[*].Instances[*].PublicIpAddress' --output text
   ```

#### Opening Port for Visualization

We need top open our chosen port so that tensorboard will be visable from your local machine.

It is also possible to use port forwarding (as discussed [here](https://aws.amazon.com/blogs/mt/amazon-ec2-instance-port-forwarding-with-aws-systems-manager/)) to 
run TensorBoard on an EC2 instance and visualize your results on a local browser.

To set this up for your instance, follow these steps:

1.  Open your EC2 instance in the [Amazon EC2console](https://console.aws.amazon.com/ec2/).

2. In the Amazon EC2 console, choose Network & Security, then choose Security Groups.

3. For Security Group, , choose the one that was created most recently (see the time stamp in the description).

4. Choose the Inbound tab, and choose Edit.

5. Choose Add Rule.

6. In the new row, type the followings:

   **Type** : Custom TCP Rule

   **Protocol**: TCP

   **Port Range**: 6006
   
   **Source**: <public_ip_of_your_machine>
   
    
Then, start tensorboard on the instance in the port 6006:

```bash
$ tensorboard --logdir <path_to_logs> --port 6006
```
    
Finally, connect to your instance from your local machine by opening up the address http://<instance-external-ip>:6006/ in a browser.

### Additional Tutorials

For additional, detailed information on how to set up tools for deep learning on AWS, refer to the [website](https://docs.aws.amazon.com/dlami/index.html) for setting up deep lerning Amazon Machine Instances (AMIs) or [this document](https://docs.aws.amazon.com/dlami/latest/devguide/dlami-dg.pdf#setup-jupyter), which has the tutorials in a single pdf. 
