# 🚀 Quick GitHub Publishing Guide

## Option 1: Automated Setup (Recommended)

Run the setup script which will guide you through the process:

```bash
./setup_github_repo.sh
```

The script will:
- ✅ Check your repository status
- ✅ Optionally replace README with team version
- ✅ Update git remote to your repository
- ✅ Stage and commit all changes
- ✅ Provide next steps for pushing

---

## Option 2: Manual Setup

### Step 1: Create GitHub Repository

1. Go to: https://github.com/new
2. Repository name: `HAKUNA_MATATA_NXP_AIM_2025` (or your choice)
3. Make it Public or Private
4. **DO NOT** initialize with README, .gitignore, or license
5. Click "Create repository"

### Step 2: Update Git Remote

```bash
# Remove old remote
git remote remove origin

# Add your new repository (replace with your details)
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
```

### Step 3: Prepare Files (Optional)

If you want to use the team-customized README:

```bash
# Backup original README
mv README.md ORIGINAL_README.md

# Use team README as main
mv TEAM_README.md README.md
```

Or keep both files (TEAM_README.md will be visible in the repo).

### Step 4: Commit Changes

```bash
# Add all files
git add .

# Commit with a descriptive message
git commit -m "Initial commit: Team HAKUNA MATATA NXP AIM India 2025 Solution"
```

### Step 5: Push to GitHub

```bash
# Option A: Push current branch
git push -u origin nxp_aim_india_2025_simulation

# Option B: Rename to main and push
git branch -M main
git push -u origin main
```

---

## 📋 Files Prepared

✅ `.gitignore` - Excludes Python cache, build files, logs
✅ `TEAM_README.md` - Custom README highlighting your team's work
✅ `setup_github_repo.sh` - Automated setup script
✅ All your code and competition files

---

## 🔒 Repository Visibility

- **Public**: Anyone can see your code (good for portfolio)
- **Private**: Only you and collaborators can see it

Choose based on:
- Competition rules (check if code sharing is allowed during competition)
- Your preference for portfolio visibility
- Team agreement

---

## 📝 Important Notes

1. **License**: Apache 2.0 - allows others to use with attribution
2. **Original Attribution**: NXP HoverGames starter code is credited
3. **Your Work**: All modifications are clearly yours
4. **Competition**: Ensure compliance with competition rules

---

## 🆘 Troubleshooting

### "Repository not found" error when pushing
- Make sure you created the repository on GitHub first
- Check the repository name matches exactly
- Verify your GitHub username is correct

### Authentication issues
```bash
# Use personal access token instead of password
# Generate token at: https://github.com/settings/tokens
```

### Want to undo changes?
```bash
# Reset to original state
git reset --hard HEAD~1
git remote add origin https://github.com/NXPHoverGames/NXP_AIM_INDIA_2025
```

---

## 📞 Need Help?

Check:
- GitHub Docs: https://docs.github.com/en/get-started
- Git Basics: https://git-scm.com/doc
- ROS 2 Docs: https://docs.ros.org/

---

**Team HAKUNA MATATA - 1535**
*No Worries, We Got This! 🦁*
