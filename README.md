#### Git tutorial:

##### Step 1 [initial]. Cloning a New Project into Your Workspace, go to `src`

    git clone -b main https://github.com/phatcvo/Beebot.git   # cloning files from `main` branch
    git branch <your branch name>                             # create <your branch name>
    git checkout -b <your branch name>                        # create and switch <your branch name>
    git push -u origin <your branch name>                     # pull your branch to origin

##### Step 2. Check and switch

    git branch # check your current branch
    git checkout <your branch name> # switch to <your branch name>

##### Step 3. Update your files

    git add .
    git commit -m "comments"
    git push                        # git push -u origin <your branch name>

##### Step 4 [optional]. If `develop` is old version, you need to sync files

    git checkout develop            # switch to develop branch
    git pull                        # pull files from origin, latest version of develop branch at that time
    git checkout <your branch name> # switch to <your branch name>
    git merge develop               # merge <develop> => <your branch name>

go back to update in Step 4

##### Step 5 [optional]. Merger to `develop` branch

    git checkout develop
    git merge <your branch name> # merge <your branch name> => develop
    git push

##### Step 6 [optional]. Delete

    git branch -d <your branch name> # delete in local
    git push origin --delete <your branch name> # delete in remote

##### Add tags

    git tag -a <v2.*> -m "<comments>"
    git push origin <v2.*>
