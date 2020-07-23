# Assignment 1

This is the first assignment for Yale's CPSC 459/559 Building Interactive Machines course. 

## Table of Contents

  * [Introduction](#introduction)
    * [System Requirements](#system-requirements)
    * [Background Knowledge](#background-knowledge)
    * [Deliverables](#deliverables)
    * [Evaluation](#evaluation)
  * [Part I\. Set up Gitlab Repository](#part-i-set-up-gitlab-repository)
    * [Tasks](#tasks)
  * [Working on your assignment](#working-on-your-assignment)
    * [Tasks](#tasks-1)

## Introduction

This assignment will teach you how to setup your main [git repository](https://git-scm.com/) for the course, and walk you through the process of creating an assignment report with [LaTeX](https://www.latex-project.org/) - a typesetting system often used for writing academic papers. These two tasks are essential for all future programming assignments. 

### System Requirements
You should have access to a computer with [git](https://git-scm.com/). The instructions below assume that you are using a [bash shell](https://en.wikipedia.org/wiki/Bash_(Unix_shell)) to do the assignment.


### Background Knowledge

If you do not have much experience working with a Linux shell, please read this 
[introduction to bash](http://cs.lmu.edu/~ray/notes/bash/) by R. Toal. At the bare minimum, you should be familiar with the commands `cd`, `ls`, `rm` before starting Part I of the assignment.

### Deliverables

* **Code:** You are expected to push code for this assignment to your
GitLab repository as indicated in Part I.

* **Report:** You are expected to submit a pdf to Canvas with answers to the questions/tasks in Part II of the assignment. This report should have the specific commit SHA of the version of the code that you would like to be evaluated on. 

### Evaluation

You assignment will be evaluated based on the content of your repository and your report:

* Part I (1 pt)
* Part II (1 pt)

## Part I. Set up Gitlab Repository

1. Create a free account in [GitHub](https://github.com/) using your Yale email. 

	> If you already have a GitHub account associated to your Yale email, you can skip this step.

2. Setup SSH keys for your GitHub account so that you can download and commit your code without having to provide your password all the time. Follow [these instructions](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
to generate a new SSH key in your machine and [upload your public key](https://docs.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account) to GitHub.

	> If you already setup an SSH key for your GitHub account, you can skip this step.

3. Create an empty, `private repository` in GitHub called *cpsc459-assignments* or  *cpsc559-assignments* based on whether you are fulfilling the undergrad (CPSC 459) or graduate (CPSC 559) requirements for the course. The GiLab project will be used to store your assignment's code.

4. Open the repository page, copy the SSH URL, and paste it in a terminal to `clone your project` into your machine. For example:

    ```bash
    # Example
    $ git clone https://github.com/<username>/cpsc459-assignments.git <username>-cpsc459-assignments
    ```

	where \<username\> is your GitHub username.

5. Change your directory to your cloned repository.

    ```bash
    # Example
    $ cd <username>-cpsc459-assignments
 
    # and check the status of your repository (you should be in the master branch of your repository)
    $ git status
    On branch master
    (...)
    ```

6. Create a `new remote called upstream` that points to the 
[assignments repository](https://github.com/Yale-BIM/f20-assignments.git)
which contains the set of assignments and starter code.

    ```bash
    # Example
    $ git remote add upstream https://github.com/Yale-BIM/f20-assignments.git
    ```

    > NOTE: A Git remote is a pair of alias and URL (link) to another Git repository.
    > By default, your freshly cloned local repository will have a remote named 
    > origin that points to your online Git repository in GitLab. 
    > The example above tells your local Git repository to track another remote Git 
    > repository located at https://gitlab.com/cpsc459-bim/assignments/f19-assignments.git
    > and name it as upstream.
    
7. Get the latest `commits from upstream` and merge them into your own local repository.

    ```bash
    # Example
    $ git pull upstream master
    ```
    
	You should then have separate folders for each of the assignments in your repository.

8. Synchronize commits from your local repository to your remote Git repository in github.com.

    ```bash
    # Example
    $ git push origin master
    ```

7. Verify that the commits from upstream are now present in your own GitHub project by checking the project's page in GitLab.

8. Ensure that your repository in GitLab has visibility set to **private** in 
Settings -> Manage Access.

9. Add the course instructor and the T.F. as members of your GitLab repository in Settings -> Manage Access. In the Manage access section, choose "Invite a collaborator", and add the Yale usernames for the instructor and teaching fellow(s).

	> The instructor's email is `marynel.vazquez _at_ yale.edu`, and the teaching fellows' emails are `debasmita.ghose _at_ yale.edu` and `sydney.thompson _at_ yale.edu` with `_at_` meaning `@`.
    
### Tasks

1. Once you've finished setting up your repository, fill and submit this [Repository Record Form](	https://forms.gle/5sHvKwZJFfeopTqa8). Your response will be used to check that your repository exists, has been setup properly, and that the instructor and T.F. have access to it.

## Working on your assignment

Once your repository is setup, you will need to follow the steps below when working on an assignment:

1. Go to the directory containing the assignment, e.g., `assignment-1`
2. Pull updates from upstream:

	```bash
	$ git pull upstream master
	```
	
	If a merge conflict happens, always use latest
	commit from upstream. Your work is safe as long you commit and push 
	your code to GitLab regularly. 
	
	Once you have resolved any merge conflicts and all commits from
	upstream are merged successfully to your own master branch, push it 
	back to your own GitLab repository:
	
	```bash
	$ git push origin master
	```
	
	> NOTE: Please consult your T.F. if you have difficulty resolving merge conflict(s).

3. Read the README.md file inside the assignments folder for the instructions on what you have to do.
4. Do the assignment. 
5. Whenever you want to save your code, commit your work.

    ```bash
    # Example
    $ git add <list-of-files-to-be-saved>
    $ git commit -m "<commit-message>"
    ```
6. Push your work to GitLab to keep a remote copy.

    ```bash
    # Example
    $ git push
    ```

    > Your final version of the assignment should be pushed to GitLab before the deadline!
    
7. Write the corresponding assignment report, 
e.g., using [Overleaf](https://www.overleaf.com/edu/yale#!overview). 

    > A LaTeX template is provided below to help you get
    started with your assignment reports. Feel free to edit this template and
    modify it to suit your needs for this.
    
8. Repeat steps 3-6 as many times as necessary while you work on the assignment.

9. When you are ready to submit your assignment,
    
    i. Gather the specific `commit hash` (SHA) of the version of the code in GitLab
    that you would like to be evaluated on. Add it to the top of your report. 
    
    > To see all of your commits, go to your repository's GitLab page and open 
    Repository -> Commits. Copy the commit SHA for the final version of the code
    and paste it in the report.
        
    ii. Generate a `pdf of your LaTeX report`.
    
    iii. Submit the report (with the commit SHA) as a pdf to [Canvas](https://yale.instructure.com/courses/51663). If the assignment
    asks for other documents, submit them to Canvas as wel.

### Tasks

1. Edit the title of the README.md at the top-level of your repository to add your name. That is, change "# CPSC-459/559 Fall 2020. Assignments" to "# CPSC-459/559 Fall 2020. Assignments - \<name\>" where \<name\> is your full name. Commit the file and push to your remote GitHub repository.

	> After pushing, you should be able to see the change in the README.md within the GitHub web interface.

2. Create a pdf report for this assignment using this [template](https://www.overleaf.com/latex/templates/assignment-template-for-yale-cpsc-459-559/nbysxxygbysk) in Overleaf.
	
	* Change the header of the template to add your name, id, email. Update the assignment number (i.e., change "X" for "1").
	* Once you've pushed your assignment to your own repository in GitHub, add the commit hash of your repository to the report (e.g., add it below the title).
	* Export the report to `pdf` and upload to Canvas.


	
	

