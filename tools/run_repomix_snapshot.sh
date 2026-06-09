#!/usr/bin/env bash
set -euo pipefail

if ! command -v repomix >/dev/null 2>&1; then
  echo "repomix not found. Install with: npm install -g repomix"
  exit 1
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="$repo_root/tools/snapshots"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_file="$output_dir/active_slam_context_$timestamp.xml"

mkdir -p "$output_dir"
cd "$repo_root"

repomix . \
  --output "$output_file" \
  --style xml \
  --compress \
  --top-files-len 20 \
  --ignore "Bumper-Bot-main/bumperbot_description/models/**,Bumper-Bot-main/bumperbot_description/meshes/**,Bumper-Bot-main/bumperbot_description/photos/**,Bumper-Bot-main/bumperbot_description/worlds/*.world,Bumper-Bot-main/bumperbot_hardware/**,Bumper-Bot-main/bumperbot_description/rviz/**,Bumper-Bot-main/bumperbot_mapping/rviz/**,Bumper-Bot-main/bumperbot_localization/rviz/**"

echo "Repomix snapshot written to: $output_file"
