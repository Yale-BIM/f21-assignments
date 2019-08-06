# CPSC-459/559 Fall 2019. Assignments

Assignment repository for Yale's CPSC-459/559 Building Interactive Machines (Fall 2019).

## Set up (Assignment -1)

Follow the instructions below to get started with your assignments. 

1. Create an empty, `private project` in GitLab called *cpsc659-assignments*. 
This project will be used to store your assignment's code.

2. Open the project page, copy the HTTPS URL, and paste it in a terminal to `clone your project` 
into your machine. 

    ```bash
    # Example
    $ git clone https://gitlab.com/<username>/cpsc659-assignments.git <username>-cpsc659-assignments
    ```

where \<username\> is your GitLab username.


3. Change your directory to your cloned repository.

    ```bash
    # Example
    $ cd <username>-cpsc659-assignments
 
    # and check the status of your repository (you should be in the master branch of your repository)
    $ git status
    On branch master
    (...)
    ```

4. Create a `new remote called upstream` that points to the 
[assignments repository](https://gitlab.com/cpsc659-bim/assignments/f18-assignments.git)
which contains the set of assignments and starter code.
    ```bash
    # Example
    $ git remote add upstream https://gitlab.com/cpsc659-bim/assignments/f18-assignments.git
    ```

    > NOTE: A Git remote is a pair of alias and URL (link) to another Git repository.
    > By default, your freshly cloned local repository will have a remote named 
    > origin that points to your online Git repository in GitLab. 
    > The example above tells your local Git repository to track another remote Git 
    > repository located at https://gitlab.com/cpsc659-bim/assignments/f18-assignments.git
    > and name it as upstream.
    
5. Get the latest `commits from upstream` and merge them into your own local repository.
    ```bash
    # Example
    $ git pull upstream master
    ```
You should then have separate folders for each of the assignments in your repository.

6. Synchronize commits from your local repository to your online Git repository in gitlab.com.
    ```bash
    # Example
    $ git push origin master
    ```

7. Verify that the commits from upstream are now present in your own GitLab project by 
checking the project's page in GitLab.

8. Ensure that your repository in GitLab has visibility set to **private** in 
Settings -> General -> Permissions.

9. Add the course instructor and the T.F. as members of your GitLab repository in Settings -> Members. In the role
permission field, choose the maintainer role for the instructor and the T.A.
    

## Working on your assignment

You should follow the steps below when working on an assignment:

1. Go to the directory containing the assignment, e.g., `assignment-0`
2. Pull updates from upstream (see the next section of this README file)
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

    > A LaTeX template is provided as part of this repository to help you get
    started with your assignment reports. Feel free to edit this template and
    modify it to suit your needs.
    
8. Repeat steps 3-6 as many times as necessary while you work on the assignment.

9. When you are ready to submit your assignment,
    
    i. Gather the specific `commit hash` (SHA) of the version of the code in GitLab
    that you would like to be evaluated on. Add it to the top of your report. 
    
    > To see all of your commits, go to your repository's GitLab page and open 
    Repository -> Commits. Copy the commit SHA for the final version of the code
    and paste it in the email that you will send to the T.A. to submit your assignment.
        
    ii. Generate a `pdf of your LaTeX report`.
    
    iii. Submit the report (with the commit SHA) as a pdf to [Canvas](https://yale.instructure.com/courses/41970). If the assignment
    asks for other documents, submit them to Canvas as wel.
    
## Pulling updates from upstream

If there are any updates in the homework assignments from `upstream`, 
you can get the latest commits and integrate them into your local Git repository with:
```bash
$ git pull upstream master
```

If a merge conflict happens, please always use latest
commit from upstream. Your work is safe as long you commit and push your code
to GitLab regularly. 

Once you have resolved any merge conflicts and all commits from
upstream are merged successfully to your own master branch, push it 
back to your own GitLab repository:
```bash
$ git push origin master
```

> NOTE: Please consult your T.A. if you have difficulty resolving merge conflict(s).
    
    
## Acknowledgements

The instructions above are based on the 
[Assignment repository](https://gitlab.com/DDP2-CSUI/assignment/blob/master/README.md)
 of the Foundation of Programming 2 (CSGE601021) Course at Universitas Indonesia.
