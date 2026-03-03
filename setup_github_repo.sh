#!/bin/bash

# Script to prepare and publish HAKUNA MATATA's NXP Competition repository
# Team ID: 1535

echo "🦁 HAKUNA MATATA - Repository Setup Script"
echo "==========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo -e "${RED}Error: package.xml not found. Please run this script from the project root.${NC}"
    exit 1
fi

echo -e "${YELLOW}Step 1: Checking repository status...${NC}"
git status

echo ""
echo -e "${YELLOW}Step 2: Do you want to use TEAM_README.md as your main README?${NC}"
echo "This will replace the original README.md with a custom one highlighting your team's work."
read -p "Replace README.md? (y/n): " replace_readme

if [ "$replace_readme" = "y" ] || [ "$replace_readme" = "Y" ]; then
    mv README.md ORIGINAL_README.md
    mv TEAM_README.md README.md
    echo -e "${GREEN}✓ README.md replaced with team version${NC}"
    echo -e "  Original saved as ORIGINAL_README.md"
else
    echo -e "${GREEN}✓ Keeping both README files${NC}"
fi

echo ""
echo -e "${YELLOW}Step 3: Enter your GitHub username:${NC}"
read -p "GitHub username: " github_username

if [ -z "$github_username" ]; then
    echo -e "${RED}Error: GitHub username is required${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 4: Enter your repository name:${NC}"
echo "  Suggested: HAKUNA_MATATA_NXP_AIM_2025 or NXP_AIM_India_2025_Solution"
read -p "Repository name: " repo_name

if [ -z "$repo_name" ]; then
    echo -e "${RED}Error: Repository name is required${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 5: Updating git remote...${NC}"
git remote remove origin 2>/dev/null || true
git remote add origin "https://github.com/$github_username/$repo_name.git"
echo -e "${GREEN}✓ Remote set to: https://github.com/$github_username/$repo_name.git${NC}"

echo ""
echo -e "${YELLOW}Step 6: Adding files to git...${NC}"
git add .
echo -e "${GREEN}✓ Files staged${NC}"

echo ""
echo -e "${YELLOW}Step 7: Creating commit...${NC}"
git commit -m "Initial commit: Team HAKUNA MATATA NXP AIM India 2025 Solution

- Implemented warehouse navigation logic
- Added QR code detection and decoding
- Integrated YOLO object recognition
- Custom path planning with heuristics
- Complete competition solution

Team ID: 1535
Competition: NXP AIM India 2025"

echo -e "${GREEN}✓ Commit created${NC}"

echo ""
echo "==========================================="
echo -e "${GREEN}✨ Repository prepared successfully!${NC}"
echo ""
echo -e "${YELLOW}NEXT STEPS:${NC}"
echo ""
echo "1. Go to: ${GREEN}https://github.com/new${NC}"
echo "2. Repository name: ${GREEN}$repo_name${NC}"
echo "3. Make it Public or Private (your choice)"
echo "4. DON'T initialize with README, .gitignore, or license"
echo "5. Click 'Create repository'"
echo ""
echo "6. Then run this command to push:"
echo -e "   ${GREEN}git push -u origin $(git branch --show-current)${NC}"
echo ""
echo "   Or if you want to rename branch to 'main':"
echo -e "   ${GREEN}git branch -M main && git push -u origin main${NC}"
echo ""
echo "==========================================="
echo -e "${YELLOW}🦁 Hakuna Matata - No Worries!${NC}"
