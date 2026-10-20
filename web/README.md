# opensurgbot pages

GitHub pages for opensurgbot.

## Deploy

Before deploying, ensure all changes have been committed to the `main` branch. Then perform the following actions:

1. Delete the `gh-pages` branch (if exists)
2. Create a new orphan `gh-pages` branch
3. Add changes to the `gh-pages` branch
4. Commit changes to the `gh-pages` branch
5. Push changes to the remote repository
6. Go back to the `master` branch

To perform these actions, run the following commands:
```bash
git branch -D gh-pages
git checkout --orphan gh-pages
git --work-tree ./web/ add --all
git --work-tree ./web/ commit -m "gh-pages"
git push origin HEAD:gh-pages --force
git checkout -f master
```