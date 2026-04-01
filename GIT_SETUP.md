# Git & GitHub Setup

Run these commands once in PyCharm's Terminal (Alt+F12) or PowerShell:

```powershell
# 1. Navigate to project (if not already there)
cd C:\Users\NIKHIL\PycharmProjects\stream_motion_interface

# 2. Initialize git and stage everything
git init
git branch -m main
git config user.email "nikhilkumar2092@gmail.com"
git config user.name "Nikhil"
git add .

# 3. Make first commit
git commit -m "Initial project: FANUC Stream Motion UDP client"

# 4. Create a private GitHub repo (using GitHub CLI - install from https://cli.github.com if needed)
gh auth login
gh repo create stream_motion_interface --private --source=. --remote=origin --push

# -- OR -- if you prefer the website:
# Go to https://github.com/new, name it "stream_motion_interface", set Private, DON'T add README
# Then run:
# git remote add origin https://github.com/nikhilkumar2092/stream_motion_interface.git
# git push -u origin main
```

## Verify the repo pushed
```powershell
git log --oneline
git remote -v
```
