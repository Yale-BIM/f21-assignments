# Setting Up Your GitHub Repo For CSPC459/559 Assignments

## Table of Contents

  * [Introduction](#introduction)
    * [System Requirements](#system-requirements)
    * [Background Knowledge](#background-knowledge)
  * [Part I\. Set up Github Repository](#part-i-set-up-github-repository)
  * [Part II\. Working on your assignment](#part-ii-working-on-your-assignment)

## Introduction
The instructions below will teach you how to setup your main [git repository](https://git-scm.com/) for the course. Following these instructions early is essential for all future programming assignments. 

### System Requirements
You should have access to a computer with [git](https://git-scm.com/). The instructions below assume that you are using a [bash shell](https://en.wikipedia.org/wiki/Bash_(Unix_shell)) to do the assignment.


### Background Knowledge

If you do not have much experience working with a Linux shell, please read this 
[introduction to bash](http://cs.lmu.edu/~ray/notes/bash/) by R. Toal. At the bare minimum, you should be familiar with the commands `cd`, `ls`, `rm` before starting Part I of the assignment.

## Part I. Set up GitHub Repository

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
[assignments repository](https://github.com/Yale-BIM/f21-assignments.git)
which contains the set of assignments and starter code.

    ```bash
    # Example
    $ git remote add upstream https://github.com/Yale-BIM/f21-assignments.git
    ```

    > NOTE: A Git remote is a pair of alias and URL (link) to another Git repository.
    > By default, your freshly cloned local repository will have a remote named 
    > origin that points to your online Git repository in GitHub. 
    > The example above tells your local Git repository to track another remote Git 
    > repository located at https://github.com/Yale-BIM/f20-assignments.git
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

7. Verify that the commits from upstream are now present in your own GitHub project by checking the project's page in GitHub.

8. Ensure that your repository in GitHub has visibility set to **private** in 
Settings -> Manage Access.

9. Add the course instructor and the T.F.(s) as members of your GitHub repository in Settings -> Manage Access. In the Manage access section, choose "Invite a collaborator", and add the Yale usernames for the instructor and teaching fellow(s).

	> The instructor's email is `marynel.vazquez _at_ yale.edu`, and the teaching fellows' emails are `nathan.tsoi _at_ yale.edu` with `_at_` meaning `@`.

10. Once you have finished setting up your repository, fill and submit this [Repository Record Form](https://forms.gle/QkChQbbVdR8b3xNx5). Your response will be used to check that your repository exists, has been setup properly, and that the instructor and T.F.(s) have access to it.

## Part II. Working on your assignment

Once your repository is setup, you will need to follow the steps below when working on a given assignment:

1. Go to the directory containing the assignment, e.g., `assignment-1`
2. Pull updates from upstream:

	```bash
	$ git pull upstream master
	```
	
	If a merge conflict happens, always use latest
	commit from upstream. Your work is safe as long you commit and push 
	your code to GitHub regularly and only work on the assignments once they are 
    officially "out" per the course [schedule](https://cpsc459-bim.gitlab.io/f21/schedule/). 
	
	Once you have resolved any merge conflicts and all commits from
	upstream are merged successfully to your own master branch, push it 
	back to your own GitHub repository:
	
	```bash
	$ git push origin master
	```
	
	> NOTE: Please consult your T.F.(s) if you have difficulty resolving merge conflict(s).

3. Read the README.md file inside the assignments folder for the instructions on what you have to do.
4. Do the assignment. 
5. Whenever you want to save your code, commit your work.

    ```bash
    # Example
    $ git add <list-of-files-to-be-saved>
    $ git commit -m "<commit-message>"
    ```
6. Push your work to GitHub to keep a remote copy.

    ```bash
    # Example
    $ git push
    ```

    > Your final version of the assignment should be pushed to GitHub before the deadline!
    
7. Write the corresponding assignment report if one needs to be submitted.
    
8. Repeat steps 3-6 as many times as necessary while you work on the assignment.

9. When you are ready to submit your assignment,
    
    i. Gather the specific `commit hash` (SHA) of the version of the code in GitHub
    that you would like to be evaluated on and add it to the top of your report if a report is requested for the assignment. 
    
    > To see all of your commits, go to your repository's GitHub page and click where it says "X commits" (right below the download "Code" button in the interface). Copy the commit SHA for the final version of the code and paste it in the report.
    
    ii. Submit the report (with the commit SHA) as a pdf to [Gradescope](https://www.gradescope.com/courses/299261). If the assignment
    asks for other documents, submit them to Gradescope as well.

    > Most assignments will appear twice in Gradescope: once for the submission of the report, once for the submission of code. The report and any other extra file that is requested should be submitted to the Report assignment; your code in GitHub should be submitted to the Code assignment. 
 
Note that assignments may have public tests to familiarize you with the process of submitting code through Gradescope. They all have private tests to
evaluate your submission. These private tests are tests for which you won't get access to until after the submission date. 
	
	

