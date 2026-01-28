## Save and Commit your changes locally
```
git add .
git commit -m "Your commit message describing your changes"
```

## Pull with rebase
```
git pull --rebase origin main
```

Note: With --rebase: Your commits will be reapplied on top of your workmate's updates, creating a cleaner history

Resolve conflicts - Git will pause and show you which files have conflicts

Open the conflicting files
Look for conflict markers (<<<<<<<, =======, >>>>>>>)
Choose which changes to keep or combine them manually
Remove the conflict markers

## After resolving conflicts:
```
git add <resolved-files>
git rebase --continue   # if you used rebase
# or just commit if you used regular pull
```

## Check the conflict
```
git status
```

## Mark the conflict as resolved
```
git add .env
```

## Continue the rebase
```
git rebase --continue
```

## If you get stuck or want to start over:
```
git rebase --abort
```

## Push your changes
```
git push origin main
```
