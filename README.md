# CPSC-659 Fall 2018. Assignments

Assignment repository for Yale's CPSC-659 Building Interactive Machines (Fall 2018).

## Set up

Follow the instructions below to get started with your assignments. 

1. Create an empty, `private project` in GitLab called *cpsc659-assignments*. 
This project will be used to store your assignment's code.

2. Open the project page, copy the HTTPS URL, and paste it in a terminal to `clone your project` 
into your machine. 

    ```bash
    # Example
    $ git clone https://gitlab.com/<username>/cpsc659-assignments.git <path>
    ```

3. Change your directory to your cloned repository.

    ```bash
    $ cd cpsc659-assignments
    # check the status of your repository
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

6. Synchronize commits from your local repository to your online Git repository in Gitlab.
    ```bash
    # Example
    $ git push origin master
    ```

7. Verify that the commits from upstream are now present in your own GitLab project by 
checking the project's page in GitLab.

8. Ensure that your repository in GitLab has visibility set to **private** in 
Settings -> General -> Permissions.

9. Add your T.A. as a member of your GitLab repository in Settings -> Members. As role
permission, choose the maintainer role for the T.A.
    

## Working on your assignment

The general process that you should follow to work on an assignment is as follows:
1. Go to the directory containing the assignment, e.g., `assignment-0`
2. Read the README.md file and assignment instructions pdf.
3. Do the assignment. 
4. Use the LaTeX template in the assignment directory to write the corresponding report, e.g., using [Overleaf](https://www.overleaf.com/edu/yale#!overview).. 
5. Whenever you want to save your progress (and your assignment report), commit your work.
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
    
6. Repeat steps 3-6 as many times as necessary while you work on your assignment.

7. When you are ready to submit your assignment,
    
    i. Gather the specific `commit hash` (SHA) of the version of the code 
    that you would like to be evaluated on from GitLab. 
    
        > To see all of your commits, go to your repository GitLab page and open 
        > Repository -> Commits. Copy the commit SHA for your final version of the code
        > and paste it in the email that you will send to the T.A. to submit your assignment.
        
    ii. Generate a `pdf of your LaTeX report`.
    
    iii. Email the commit SHA and pdf report to your T.A.
    
## Pulling updates from upstream

If there are any updates in the homework assignments from `upstream`, 
you can get the latest commits and integrate them into your local Git repository with:
```bash
$ git pull upstream master
```

If merge conflict happens, please always use latest
commit from upstream. Your work is safe as long you commit and push your code
to GitLab regularly. 

Once you have resolved any merge conflicts and all commits from
upstream are merged successfully to your own master branch, push it 
back to your own GitLab repository. Use the Git command:
```bash
$ git push origin master
```

    > Note: Please consult your T.A. if you have difficulty resolving merge conflict(s).
    
    
## Acknowledgements

The set up instructions above were based on the 
[Assignment repository](https://gitlab.com/DDP2-CSUI/assignment/blob/master/README.md)
 of the Foundation of Programming 2 (CSGE601021) Course at Universitas Indonesia.
