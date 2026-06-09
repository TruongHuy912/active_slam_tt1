#!/usr/bin/env bash
set -euo pipefail

echo "=== Active SLAM repo snapshot ==="
pwd

echo "=== Git status ==="
git status --short || true

echo "=== Important files ==="
find . -maxdepth 5 -type f \
  \( -name "*.py" -o -name "*.yaml" -o -name "*.yml" -o -name "*.launch.py" -o -name "package.xml" -o -name "CMakeLists.txt" -o -name "*.md" \) \
  | sort

echo "=== active_slam_node.py locations ==="
find . -name "active_slam_node.py" -type f | sort

echo "=== Tool checks ==="
command -v codegraph || true
command -v repomix || true
command -v open-code-review || true
command -v ocr || true
command -v graphify || true

echo "=== Repomix check ==="
if command -v repomix >/dev/null 2>&1; then
  echo "repomix found. You can run: tools/run_repomix_snapshot.sh"
else
  echo "repomix not found."
fi
