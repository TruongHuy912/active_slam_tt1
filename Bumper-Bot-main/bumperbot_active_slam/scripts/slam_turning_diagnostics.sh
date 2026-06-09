#!/usr/bin/env bash
set -u

print_section() {
  printf '\n== %s ==\n' "$1"
}

ok() {
  printf 'OK: %s\n' "$1"
}

warn() {
  printf 'WARN: %s\n' "$1"
}

fail() {
  printf 'FAIL: %s\n' "$1"
}

have_topic() {
  ros2 topic list 2>/dev/null | grep -qx "$1"
}

have_node() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

topic_hz_once() {
  local topic="$1"
  timeout 6 ros2 topic hz "$topic" 2>/dev/null | sed -n 's/.*average rate: //p' | head -n 1
}

print_section "Core nodes"
if have_node "/slam_toolbox"; then ok "/slam_toolbox node exists"; else fail "/slam_toolbox node missing"; fi
if have_node "/controller_server"; then ok "/controller_server node exists"; else warn "/controller_server node missing"; fi
if have_node "/robot_state_publisher"; then ok "/robot_state_publisher node exists"; else warn "/robot_state_publisher node missing"; fi

print_section "Scan"
if have_topic "/scan"; then
  ok "/scan topic exists"
  ros2 topic info /scan -v 2>/dev/null | sed -n '1,18p'
  scan_frame=$(timeout 5 ros2 topic echo /scan --once 2>/dev/null | sed -n 's/.*frame_id: //p' | tr -d '"' | head -n 1)
  if [ -n "$scan_frame" ]; then ok "/scan frame_id=$scan_frame"; else warn "could not read /scan frame_id"; fi
  scan_hz=$(topic_hz_once /scan)
  if [ -n "$scan_hz" ]; then ok "/scan average rate: $scan_hz Hz"; else warn "could not measure /scan hz"; fi
else
  fail "/scan topic missing"
fi

print_section "Odometry"
if have_topic "/bumperbot_controller/odom"; then
  ok "/bumperbot_controller/odom topic exists"
  odom_header=$(timeout 5 ros2 topic echo /bumperbot_controller/odom --once 2>/dev/null)
  frame_id=$(printf '%s\n' "$odom_header" | sed -n 's/.*frame_id: //p' | tr -d '"' | head -n 1)
  child_frame_id=$(printf '%s\n' "$odom_header" | sed -n 's/.*child_frame_id: //p' | tr -d '"' | head -n 1)
  if [ -n "$frame_id" ]; then ok "odom frame_id=$frame_id"; else warn "could not read odom frame_id"; fi
  if [ -n "$child_frame_id" ]; then ok "odom child_frame_id=$child_frame_id"; else warn "could not read odom child_frame_id"; fi
  odom_hz=$(topic_hz_once /bumperbot_controller/odom)
  if [ -n "$odom_hz" ]; then ok "/bumperbot_controller/odom average rate: $odom_hz Hz"; else warn "could not measure odom hz"; fi
else
  fail "/bumperbot_controller/odom topic missing"
fi

print_section "TF checks"
if timeout 5 ros2 run tf2_ros tf2_echo map base_link >/tmp/slam_turning_tf_map_base.txt 2>&1; then
  ok "TF map -> base_link available"
else
  warn "TF map -> base_link unavailable"
  sed -n '1,4p' /tmp/slam_turning_tf_map_base.txt
fi

if timeout 5 ros2 run tf2_ros tf2_echo odom base_link >/tmp/slam_turning_tf_odom_base.txt 2>&1; then
  ok "TF odom -> base_link available"
else
  warn "TF odom -> base_link unavailable"
  sed -n '1,4p' /tmp/slam_turning_tf_odom_base.txt
fi

if [ -n "${scan_frame:-}" ]; then
  if timeout 5 ros2 run tf2_ros tf2_echo base_link "$scan_frame" >/tmp/slam_turning_tf_base_scan.txt 2>&1; then
    ok "TF base_link -> $scan_frame available"
  else
    warn "TF base_link -> $scan_frame unavailable"
    sed -n '1,4p' /tmp/slam_turning_tf_base_scan.txt
  fi
fi

print_section "SLAM Toolbox parameters"
if have_node "/slam_toolbox"; then
  for param in odom_frame map_frame base_frame scan_topic map_update_interval minimum_time_interval minimum_travel_distance minimum_travel_heading throttle_scans transform_publish_period use_scan_matching; do
    value=$(ros2 param get /slam_toolbox "$param" 2>/dev/null || true)
    if [ -n "$value" ]; then ok "slam_toolbox.$param: $value"; else warn "could not read slam_toolbox.$param"; fi
  done
  map_update=$(ros2 param get /slam_toolbox map_update_interval 2>/dev/null | awk '{print $NF}')
  min_time=$(ros2 param get /slam_toolbox minimum_time_interval 2>/dev/null | awk '{print $NF}')
  min_dist=$(ros2 param get /slam_toolbox minimum_travel_distance 2>/dev/null | awk '{print $NF}')
  min_heading=$(ros2 param get /slam_toolbox minimum_travel_heading 2>/dev/null | awk '{print $NF}')
  if [ "$map_update" = "1.0" ] && [ "$min_time" = "0.2" ] && [ "$min_dist" = "0.2" ] && [ "$min_heading" = "0.15" ]; then
    ok "SLAM profile appears to be turning_stable"
  elif [ "$map_update" = "2.0" ] && [ "$min_time" = "0.5" ] && [ "$min_dist" = "0.5" ] && [ "$min_heading" = "0.5" ]; then
    warn "SLAM profile appears to be default, not turning_stable"
  else
    warn "SLAM profile values do not exactly match default or turning_stable"
  fi
fi

print_section "Controller parameters"
if have_node "/controller_server"; then
  for param in FollowPath.desired_linear_vel FollowPath.lookahead_dist FollowPath.min_lookahead_dist FollowPath.max_lookahead_dist FollowPath.rotate_to_heading_angular_vel FollowPath.rotate_to_heading_min_angle FollowPath.max_angular_accel; do
    value=$(ros2 param get /controller_server "$param" 2>/dev/null || true)
    if [ -n "$value" ]; then ok "controller_server.$param: $value"; else warn "could not read controller_server.$param"; fi
  done
fi

print_section "Map"
if have_topic "/map"; then
  map_sample=$(timeout 5 ros2 topic echo /map --once --qos-reliability reliable --qos-durability transient_local 2>/dev/null)
  width=$(printf '%s\n' "$map_sample" | sed -n 's/.*width: //p' | head -n 1)
  height=$(printf '%s\n' "$map_sample" | sed -n 's/.*height: //p' | head -n 1)
  if [ -n "$width" ] && [ -n "$height" ] && [ "$width" != "0" ] && [ "$height" != "0" ]; then
    ok "/map size is ${width}x${height}"
  else
    warn "/map is empty or could not be read"
  fi
else
  fail "/map topic missing"
fi

print_section "Interpretation"
printf '%s\n' "If map skew appears mainly during sharp turns, compare this output while using:"
printf '%s\n' "  1. default slam_toolbox.yaml"
printf '%s\n' "  2. slam_toolbox_turning_stable.yaml"
printf '%s\n' "Check for scan/odom drops, TF failures, and aggressive controller angular settings."
