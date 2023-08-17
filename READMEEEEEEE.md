# **GIT TUTORIAL AND HOW TO**

[//]: <> (Press ctrl+shift+v to view this using markdown!)
[//]: <> (Jump to the Workflow section to skip the beginner stuff)

---

## Git GUI (Graphical User Interface)

- I recommend using [SourceTree](https://www.sourcetreeapp.com/) as your local Git manager/GUI.  It is just as powerful and can connect to github or any other repo host but I find it much simpler than other options.
- You shouldn't need to sign in(?), but will need to set your access credentials to be allowed to pull the code. This should be the similar to other Git GUI's
- To add a new repository (code base), Clone it to your local machine from a URL obtained from the Github (not the page url, there is a button to generate it)
- You can then open vscode to the folder you selected above and can modify your code as normal.

- Alternativly you can use github gui or terminal/commandline

## Terminology

- **branch**: A mostly independent copy of the code.

- **checkout**: change branches (double click on a branch in the list)
  - You can checkout remote branches in as well to look at code from online.
  - Note: If there is a conflict see below

- **stage**: select which changed files you want to upload.

- **merge conflict**: Conflicts happen when merging branches that both have changes to similar files in them.
  - Note: See below for resolutions

- **Pull Request**: A request to merge code from 1 branch to another
  - Note: You can do this with the merge button, but this is much safer and easier to understand the consequences

- **Git-Flow**: The branching hierarchy that's used to ensure:
    1. Theres always a stable version of the code that can be used.
    2. Any new code developed starts from functional code (no need to debug others issues before your own)
    3. Any new code developed can still have the latest untested features.

## Git-flow Hierarchy Lite

- branch off *develop* to get a *feature*
- merge *feature* into *develop* when ready
- merge *develop* into *master* when tested

![Git-Flow Diagram](https://www.bitbull.it/blog/git-flow-come-funziona/gitflow-1.png)

### Branch Naming and purpose

- **remote branch**: the cloud server where your branches are stored
- **local branch** : a branch on your machine
- **current branch**: the currently selected branch on your local machine


- **master branch**: ==This code must always be stable==, it is fully tested and ready to be used by the drivers
- **develop branch**: ==This code mus always be functional== it's where testing can be done, but only code that is functionaly complete should be put here.
  - Note: **Shame on the developer that pushes broken code to develop!**
- **feature branch**: ==Used for developing new features==, no matter how big or small.
  - Note: You can merge code in whatever state you want here.
- **hotfix branch, etc:** You can use these as you want, your project isn't complex enough to need proper git-flow.  I just added them so you know they exist

### Sourctree Buttons and command line alternatives

- **Checkout**: Change branches
  - `git checkout "<branchName>"`
  - #Potential conflict if you have local changes

- **Commit**: save changed files for uploading
  - `git commit -m "<message>"`
- **Push**: upload commits for current branch
  - `git push`
- **Pull**: get changes from remote and merge into local branch
  - `git pull <remoteBranch>`
- **Fetch**: check for updates from remote (Pull will fetch before pulling)
  - `git git fetch --all`

- **Branch**: Split off from current branch to a new one
  - `git checkout -b "<newBranchName>"`
  - Note: you should never be working directly on either *master* or *develop*, always a local *feature* branch
  - Warning: DO NOT USE THE BUTTON FOR THIS - it can break the automatic git-flow relationships if done wrong
- **Merge**: Merge current branch into other one
  - `git commit -m <message>`
  - Note: Never use this locally and even then only use this for conflict fixes
  - Warning: DO NOT USE THE BUTTON FOR THIS - Very dangerous, use the PR method instead so you can revert mistakes.

- **Stash**: Take local changes and store them.  This puts the current branch in it's initial state
  - `git stash`
- **Pop/Apply Stash**: Take stashed changes and apply them to the current branch (See list at bottom left)
  - `git stash >`
- **Discard**: get rid of local changes permanently
  - `git clean -fxd`
- **Tag**: label branch for organization (not needed)

- **Status**: All uncommited changes made. Shown in the interface.
  - `git status`


- **Git Flow**: The better way to branch consistently.  Using this you can make a feature or hotfix branch from develop
- **Remote**: Takes you to the browser page where your code is pushed to
- **Terminal**: opens a cmd terminal window (normally not used)

![Sourcetree](https://blog.sourcetreeapp.com/files/2017/01/win_2_header.png)

---

## Workflow - How to do what I want

### Local Branch Setup

1. checkout *develop*
2. pull *develop* to get the up-to-date code
3. create branch called *feature/coolNewThing*
   - Note: you are now on a local branch feature/coolNewThing
4. /\* Do code stuff \*/

### At end of day

1. stage all changes you want to keep
2. commit and push
   - Note: this way you can access *feature/coolNewThing* from other computers, but it's not merged with everyone else's code as yours isn't ready and *develop* must allways be functional.
   - Remember: In case of fire: 1. `git commit`, 2. `git push`, 3. Exit Building

### If someone else made changes to develop that you want in your branch

1. stash your changes
2. checkout *develop*
3. pull *develop*
4. checkout feature/coolNewThing
   - Note: If branching is done correctly, this should contain all changes from *develop*
   - Note: If not, open the terminal at the top right and type `git merge develop`
5. Apply you're stashed changes.
   - #Potential conflict!

### When youre ready to merge

1. stage all changes you want to keep
2. commit + push
3. Go to the browser
4. Create a Pull Request to merge *feature/coolNewThing* to *develop*
   - Note: If it isn't in the list, you probably never pushed your local branch
5. Review the change list to make sure it contains all the stuff and only the stuff you want
   - #Potential conflict!
   - Note: You can still push code to a remote branch while the PR is open
6. Ideally, this is where your code can be peer reviewed and potentially approved.
7. After your code's been approved for merging, you can merge the PR.
   - Note: When merging *develop* into *master*, you can follow the same steps
8. Once the pr is merged, you can safely delete your *feature/coolNewThing* branch

### Running the FRC bot

1. Checkout the branch you want to run
   - Use *master* for driver practice
   - Use *develop* for programmer and driver testing
   - Use *feature* for programmers making new things spin in the right directions
2. Deploy/Run code as normal

### Fixing a merge conflict

1. stash your changes
2. checkout *develop*
3. pull *develop*  
   - this ensure your local copies are up to date
4. checkout *feature/coolNewThing*
5. Resolve the conflict by going into vscode and selecting which version of the code to use, or combine them yourself.  
6. Save it and git should be happy.
7. If you were trying to merge, push the changes.
   - Note: The commit message will be auto-generated
