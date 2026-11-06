#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
robotics_expected='https://github.com/pietroforni/Vision-Based-Autonomous-Robot-Navigation.git'
if ! git remote get-url origin >/dev/null 2>&1; then
    git remote add origin "$robotics_expected"
fi
if [[ "$(git remote get-url origin)" != "$robotics_expected" ]]; then
    echo 'Unexpected origin; stopping before push.' >&2
    exit 1
fi
git push -u origin main
