#!/usr/bin/env bash
set -u

echo "== Active SLAM Collision Diagnostics =="

print_cmd() {
  echo
  echo "## $*"
  "$@" || true
}

print_cmd ros2 action list
print_cmd ros2 action info /compute_path_to_pose
print_cmd ros2 action info /navigate_to_pose

echo
echo "## /scan sample"
timeout 5 ros2 topic echo /scan --once || true

echo
echo "## /scan hz"
timeout 5 ros2 topic hz /scan || true

echo
echo "## /global_costmap/costmap info"
ros2 topic info /global_costmap/costmap -v || true

echo
echo "## /local_costmap/costmap info"
ros2 topic info /local_costmap/costmap -v || true

echo
echo "## /global_costmap/costmap sample header/info"
timeout 5 ros2 topic echo /global_costmap/costmap --once --field header || true
timeout 5 ros2 topic echo /global_costmap/costmap --once --field info || true

echo
echo "## /local_costmap/costmap sample header/info"
timeout 5 ros2 topic echo /local_costmap/costmap --once --field header || true
timeout 5 ros2 topic echo /local_costmap/costmap --once --field info || true

echo
echo "## Nav2 lifecycle"
for node in /bt_navigator /planner_server /controller_server /behavior_server /smoother_server; do
  ros2 lifecycle get "${node}" || true
done

echo
echo "## Config files to inspect if doors/walls are missing from costmaps"
echo "Bumper-Bot-main/bumperbot_navigation/config/planner_server.yaml"
echo "Bumper-Bot-main/bumperbot_navigation/config/controller_server.yaml"
