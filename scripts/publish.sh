#!/usr/bin/env bash
# Publish this prepared project to the owner's GitHub account.
set -euo pipefail
cd "$(dirname "$0")/.."
command -v gh >/dev/null || { echo 'Install the GitHub CLI (gh) first.' >&2; exit 1; }
if ! gh auth status >/dev/null 2>&1; then
    gh auth login --hostname github.com --web --git-protocol https
fi
gh auth setup-git
if git remote get-url origin >/dev/null 2>&1; then
    robotics_remote="$(git remote get-url origin)"
    case "$robotics_remote" in
        https://github.com/pietroforni/vision-navigation.git|https://github.com/pietroforni/vision-navigation|git@github.com:pietroforni/vision-navigation.git)
            git push -u origin main ;;
        *) echo "Unexpected origin: $robotics_remote. Stopping." >&2; exit 1 ;;
    esac
else
    gh repo create pietroforni/vision-navigation --public \
        --description 'Overhead-camera RoboMaster navigation with OpenCV, A* and ROS 2' \
        --source . --remote origin --push
fi
printf '\nPublished: https://github.com/pietroforni/vision-navigation\n'
