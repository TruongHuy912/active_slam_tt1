#!/usr/bin/env bash

set -u

print_section() {
  printf '\n========== %s ==========\n' "$1"
}

ok() {
  echo "OK: $1"
}

warn() {
  echo "WARN: $1"
}

fail() {
  echo "FAIL: $1"
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

node_exists() {
  ros2 node list 2>/dev/null | grep -Fxq "$1"
}

param_get() {
  local node="$1"
  local param="$2"
  ros2 param get "$node" "$param" 2>/dev/null | sed 's/^[A-Za-z]* value is: //'
}

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

SCAN_EXISTS=0
SCAN_HZ_OK=0
SCAN_HAS_FINITE=0
CLOCK_HZ_OK=0
SLAM_NODE_OK=0
MAP_VALID=0
TF_MAP_BASE_OK=0
TF_ODOM_BASE_OK=0
TF_BASE_LASER_OK=0

SCAN_FRAME=""
MAP_WIDTH=0
MAP_HEIGHT=0

print_section "Node and topic presence"
if node_exists "/slam_toolbox"; then
  SLAM_NODE_OK=1
  ok "/slam_toolbox node exists"
else
  fail "/slam_toolbox node does not exist"
fi

if topic_exists "/scan"; then
  SCAN_EXISTS=1
  ok "/scan topic exists"
else
  fail "/scan topic does not exist"
fi

if topic_exists "/clock"; then
  ok "/clock topic exists"
else
  fail "/clock topic does not exist"
fi

if topic_exists "/map"; then
  ok "/map topic exists"
else
  fail "/map topic does not exist"
fi

print_section "/scan"
ros2 topic info /scan -v || true
timeout 5 ros2 topic echo /scan --once > "$TMP_DIR/scan.txt" 2>&1
cat "$TMP_DIR/scan.txt" | sed -n '1,90p'
SCAN_FRAME="$(awk '/frame_id:/ {print $2; exit}' "$TMP_DIR/scan.txt")"
SCAN_FRAME="${SCAN_FRAME:-}"
if [ -n "$SCAN_FRAME" ]; then
  ok "/scan frame_id is ${SCAN_FRAME}"
else
  fail "Could not read /scan frame_id"
fi

FINITE_COUNT="$(grep -E '^- [0-9]+(\.[0-9]+)?$' "$TMP_DIR/scan.txt" | wc -l | tr -d ' ')"
if [ "${FINITE_COUNT:-0}" -gt 0 ] 2>/dev/null; then
  SCAN_HAS_FINITE=1
  ok "/scan has finite range samples (${FINITE_COUNT})"
else
  warn "/scan sample appears to contain no finite ranges. In an empty world this prevents useful SLAM map growth."
fi

timeout 5 ros2 topic hz /scan > "$TMP_DIR/scan_hz.txt" 2>&1
cat "$TMP_DIR/scan_hz.txt"
if grep -q "average rate" "$TMP_DIR/scan_hz.txt"; then
  SCAN_HZ_OK=1
  ok "/scan hz measured"
else
  fail "/scan hz was not measured"
fi

print_section "/clock"
timeout 5 ros2 topic hz /clock > "$TMP_DIR/clock_hz.txt" 2>&1
cat "$TMP_DIR/clock_hz.txt"
if grep -q "average rate" "$TMP_DIR/clock_hz.txt"; then
  CLOCK_HZ_OK=1
  ok "/clock hz measured"
else
  fail "/clock hz was not measured"
fi

print_section "slam_toolbox parameters"
if [ "$SLAM_NODE_OK" -eq 1 ]; then
  for p in mode scan_topic base_frame odom_frame map_frame use_sim_time transform_publish_period map_update_interval minimum_travel_distance minimum_travel_heading; do
    printf '%s: %s\n' "$p" "$(param_get /slam_toolbox "$p")"
  done
else
  warn "Skipping slam_toolbox params because /slam_toolbox is not running"
fi

print_section "use_sim_time parameters"
for node in /slam_toolbox /robot_state_publisher /active_slam_explorer /bt_navigator /controller_server /planner_server; do
  if node_exists "$node"; then
    printf '%s use_sim_time: %s\n' "$node" "$(param_get "$node" use_sim_time)"
  else
    printf '%s use_sim_time: node not running\n' "$node"
  fi
done

print_section "/map"
ros2 topic info /map -v || true
timeout 5 ros2 topic echo /map --once --qos-reliability reliable --qos-durability transient_local > "$TMP_DIR/map.txt" 2>&1
cat "$TMP_DIR/map.txt" | sed -n '1,90p'
MAP_WIDTH="$(awk '/^[[:space:]]*width:/ {print $2; exit}' "$TMP_DIR/map.txt")"
MAP_HEIGHT="$(awk '/^[[:space:]]*height:/ {print $2; exit}' "$TMP_DIR/map.txt")"
MAP_WIDTH="${MAP_WIDTH:-0}"
MAP_HEIGHT="${MAP_HEIGHT:-0}"
if [ "$MAP_WIDTH" -gt 0 ] 2>/dev/null && [ "$MAP_HEIGHT" -gt 0 ] 2>/dev/null; then
  MAP_VALID=1
  ok "/map is valid (${MAP_WIDTH}x${MAP_HEIGHT})"
else
  fail "/map is empty or invalid (${MAP_WIDTH}x${MAP_HEIGHT})"
fi

print_section "TF checks"
timeout 6 ros2 run tf2_ros tf2_echo map base_link > "$TMP_DIR/tf_map_base.txt" 2>&1
cat "$TMP_DIR/tf_map_base.txt" | sed -n '1,80p'
if grep -q "Translation:" "$TMP_DIR/tf_map_base.txt"; then
  TF_MAP_BASE_OK=1
  ok "TF map -> base_link is available"
else
  fail "TF map -> base_link is not available"
fi

timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint > "$TMP_DIR/tf_odom_base.txt" 2>&1
cat "$TMP_DIR/tf_odom_base.txt" | sed -n '1,80p'
if grep -q "Translation:" "$TMP_DIR/tf_odom_base.txt"; then
  TF_ODOM_BASE_OK=1
  ok "TF odom -> base_footprint is available"
else
  fail "TF odom -> base_footprint is not available"
fi

if [ -n "$SCAN_FRAME" ]; then
  timeout 6 ros2 run tf2_ros tf2_echo base_link "$SCAN_FRAME" > "$TMP_DIR/tf_base_laser.txt" 2>&1
  cat "$TMP_DIR/tf_base_laser.txt" | sed -n '1,80p'
  if grep -q "Translation:" "$TMP_DIR/tf_base_laser.txt"; then
    TF_BASE_LASER_OK=1
    ok "TF base_link -> ${SCAN_FRAME} is available"
  else
    fail "TF base_link -> ${SCAN_FRAME} is not available"
  fi
else
  warn "Skipping base_link -> laser TF check because scan frame is unknown"
fi

print_section "Summary"
[ "$SCAN_EXISTS" -eq 1 ] && ok "/scan exists" || fail "/scan missing"
[ "$SCAN_HZ_OK" -eq 1 ] && ok "/scan hz OK" || fail "/scan hz missing"
[ "$SCAN_HAS_FINITE" -eq 1 ] && ok "/scan has finite obstacle returns" || warn "/scan has no finite obstacle returns"
[ "$CLOCK_HZ_OK" -eq 1 ] && ok "/clock hz OK" || fail "/clock hz missing"
[ "$SLAM_NODE_OK" -eq 1 ] && ok "slam_toolbox node exists" || fail "slam_toolbox node missing"
[ "$MAP_VALID" -eq 1 ] && ok "/map width/height > 0" || fail "/map remains ${MAP_WIDTH}x${MAP_HEIGHT}"
[ "$TF_MAP_BASE_OK" -eq 1 ] && ok "TF map -> base_link OK" || fail "TF map -> base_link missing"
[ "$TF_ODOM_BASE_OK" -eq 1 ] && ok "TF odom -> base_footprint OK" || fail "TF odom -> base_footprint missing"
if [ -n "$SCAN_FRAME" ]; then
  [ "$TF_BASE_LASER_OK" -eq 1 ] && ok "TF base_link -> ${SCAN_FRAME} OK" || fail "TF base_link -> ${SCAN_FRAME} missing"
fi

if [ "$MAP_VALID" -ne 1 ]; then
  warn "If TF and /scan are OK but /map remains 0x0, relaunch in a non-empty world: world_name:=small_house or world_name:=small_warehouse."
fi
