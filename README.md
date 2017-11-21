# For the group
- First cd to your workspace folder
- cd into src/ 
- then copy the url from this repo by clicking on clone or download and clicking the copy button
- then run `git clone <url_here>` from the terminal in the src folder

# quick and dirty git commands
- after you have changed the code you are working on and want to save changes you can commit by running `git add .` then `git commit -m "<meaningful_commit_message_here>"`.
- once you have made a commit you can push your code to the git repo by typing `git push origin master`
- to check to see if there is any code you need to download you can run `git fetch origin master` and if you get any text after running that command you can pull those changes in by running `git pull origin master`
- to check your current commit status run `git status`
- to see a history of commits on the command line you can run `git log`