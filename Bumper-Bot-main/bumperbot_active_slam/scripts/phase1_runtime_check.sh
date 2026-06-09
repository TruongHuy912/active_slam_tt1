#!/usr/bin/env bash

set -u

print_section() {
  printf '\n========== %s ==========\n' "$1"
}

run_cmd() {
  printf '\n$ %s\n' "$*"
  "$@"
  return $?
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

node_exists() {
  ros2 node list 2>/dev/null | grep -Fxq "$1"
}

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

SCAN_HAS_DATA=0
SCAN_HAS_RATE=0
CLOCK_HAS_DATA=0
CLOCK_HAS_RATE=0
MAP_HAS_DATA=0
MAP_VALID=0
TF_AVAILABLE=0
MARKERS_HAS_PUBLISHER=0

print_section "ROS nodes"
run_cmd ros2 node list

print_section "ROS topics"
run_cmd ros2 topic list

print_section "/scan"
run_cmd ros2 topic info /scan -v
timeout 5 ros2 topic echo /scan --once > "$TMP_DIR/scan_echo.txt" 2>&1
SCAN_ECHO_RC=$?
cat "$TMP_DIR/scan_echo.txt"
if [ "$SCAN_ECHO_RC" -eq 0 ] && [ -s "$TMP_DIR/scan_echo.txt" ]; then
  SCAN_HAS_DATA=1
fi
timeout 5 ros2 topic hz /scan > "$TMP_DIR/scan_hz.txt" 2>&1
cat "$TMP_DIR/scan_hz.txt"
if grep -q "average rate" "$TMP_DIR/scan_hz.txt"; then
  SCAN_HAS_RATE=1
fi

print_section "/clock"
run_cmd ros2 topic info /clock -v
timeout 5 ros2 topic echo /clock --once > "$TMP_DIR/clock_echo.txt" 2>&1
CLOCK_ECHO_RC=$?
cat "$TMP_DIR/clock_echo.txt"
if [ "$CLOCK_ECHO_RC" -eq 0 ] && [ -s "$TMP_DIR/clock_echo.txt" ]; then
  CLOCK_HAS_DATA=1
fi
timeout 5 ros2 topic hz /clock > "$TMP_DIR/clock_hz.txt" 2>&1
cat "$TMP_DIR/clock_hz.txt"
if grep -q "average rate" "$TMP_DIR/clock_hz.txt"; then
  CLOCK_HAS_RATE=1
fi

print_section "/map"
run_cmd ros2 topic info /map -v
timeout 5 ros2 topic echo /map --once --qos-reliability reliable --qos-durability transient_local > "$TMP_DIR/map_echo.txt" 2>&1
MAP_ECHO_RC=$?
cat "$TMP_DIR/map_echo.txt"
if [ "$MAP_ECHO_RC" -eq 0 ] && [ -s "$TMP_DIR/map_echo.txt" ]; then
  MAP_HAS_DATA=1
fi
MAP_WIDTH="$(awk '/^[[:space:]]*width:/ {print $2; exit}' "$TMP_DIR/map_echo.txt")"
MAP_HEIGHT="$(awk '/^[[:space:]]*height:/ {print $2; exit}' "$TMP_DIR/map_echo.txt")"
MAP_WIDTH="${MAP_WIDTH:-0}"
MAP_HEIGHT="${MAP_HEIGHT:-0}"
if [ "$MAP_WIDTH" -gt 0 ] 2>/dev/null && [ "$MAP_HEIGHT" -gt 0 ] 2>/dev/null; then
  MAP_VALID=1
fi

print_section "TF map -> base_link"
timeout 8 ros2 run tf2_ros tf2_echo map base_link > "$TMP_DIR/tf_echo.txt" 2>&1
cat "$TMP_DIR/tf_echo.txt"
if grep -q "Translation:" "$TMP_DIR/tf_echo.txt"; then
  TF_AVAILABLE=1
fi

print_section "active_slam markers"
ros2 topic list | grep active_slam || true
if topic_exists "/active_slam/markers"; then
  run_cmd ros2 topic info /active_slam/markers -v
  if ros2 topic info /active_slam/markers -v 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    MARKERS_HAS_PUBLISHER=1
  fi
  timeout 5 ros2 topic echo /active_slam/markers --once > "$TMP_DIR/markers_echo.txt" 2>&1
  cat "$TMP_DIR/markers_echo.txt"
else
  echo "/active_slam/markers topic does not exist."
fi

print_section "Conclusion"
if [ "$SCAN_HAS_DATA" -eq 1 ]; then
  echo "OK: /scan produced at least one LaserScan sample."
else
  echo "FAIL: /scan did not produce a LaserScan sample within 5 seconds."
fi

if [ "$SCAN_HAS_RATE" -eq 1 ]; then
  echo "OK: /scan has measured rate."
else
  echo "WARN: /scan rate was not measured within 5 seconds."
fi

if [ "$CLOCK_HAS_DATA" -eq 1 ]; then
  echo "OK: /clock produced at least one sample."
else
  echo "FAIL: /clock did not produce a sample within 5 seconds."
fi

if [ "$CLOCK_HAS_RATE" -eq 1 ]; then
  echo "OK: /clock has measured rate."
else
  echo "WARN: /clock rate was not measured within 5 seconds."
fi

if [ "$MAP_HAS_DATA" -eq 1 ]; then
  echo "OK: /map produced an OccupancyGrid sample."
else
  echo "FAIL: /map did not produce an OccupancyGrid sample within 5 seconds."
fi

if [ "$MAP_VALID" -eq 1 ]; then
  echo "OK: /map width/height are greater than zero (${MAP_WIDTH}x${MAP_HEIGHT})."
else
  echo "FAIL: /map is empty or invalid (${MAP_WIDTH}x${MAP_HEIGHT}); frontier markers cannot exist until SLAM publishes a valid map."
fi

if [ "$TF_AVAILABLE" -eq 1 ]; then
  echo "OK: TF map -> base_link is available."
else
  echo "FAIL: TF map -> base_link was not available within 8 seconds."
fi

if [ "$MARKERS_HAS_PUBLISHER" -eq 1 ]; then
  echo "OK: /active_slam/markers has a publisher."
else
  echo "WARN: /active_slam/markers has no publisher. Start active_slam_explorer or check marker_topic config."
fi

if node_exists "/active_slam_explorer"; then
  echo "OK: /active_slam_explorer node is running."
else
  echo "WARN: /active_slam_explorer node is not running."
fi

if [ "$MAP_VALID" -ne 1 ]; then
  echo "NOTE: With map=${MAP_WIDTH}x${MAP_HEIGHT}, active_slam_explorer should only publish clear/empty markers and wait."
fi
