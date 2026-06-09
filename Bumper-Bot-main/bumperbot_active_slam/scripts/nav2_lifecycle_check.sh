#!/usr/bin/env bash
set -u

ACTION_NAME="/navigate_to_pose"
LIFECYCLE_MANAGER="/lifecycle_manager_navigation"
LIFECYCLE_NODES=(
  "/bt_navigator"
  "/planner_server"
  "/controller_server"
  "/behavior_server"
  "/smoother_server"
)

ok() {
  printf 'OK: %s\n' "$1"
}

warn() {
  printf 'WARN: %s\n' "$1"
}

fail() {
  printf 'FAIL: %s\n' "$1"
}

section() {
  printf '\n== %s ==\n' "$1"
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

if ! command_exists ros2; then
  fail "ros2 command not found. Source the ROS 2 and workspace setup files first."
  exit 1
fi

section "duplicate nodes"
NODE_LIST="$(ros2 node list 2>/dev/null || true)"
if [ -z "$NODE_LIST" ]; then
  fail "ros2 node list returned no nodes"
else
  DUPLICATES="$(printf '%s\n' "$NODE_LIST" | sort | uniq -d)"
  if [ -z "$DUPLICATES" ]; then
    ok "no duplicate node names detected"
  else
    warn "duplicate node names detected:"
    printf '%s\n' "$DUPLICATES"
  fi
fi

section "lifecycle manager services"
SERVICE_LIST="$(ros2 service list 2>/dev/null || true)"
for srv in "${LIFECYCLE_MANAGER}/is_active" "${LIFECYCLE_MANAGER}/manage_nodes"; do
  if printf '%s\n' "$SERVICE_LIST" | grep -qx "$srv"; then
    ok "$srv exists"
  else
    fail "$srv missing"
  fi
done

if printf '%s\n' "$SERVICE_LIST" | grep -qx "${LIFECYCLE_MANAGER}/is_active"; then
  section "lifecycle manager is_active"
  IS_ACTIVE_OUTPUT="$(
    timeout 5 ros2 service call "${LIFECYCLE_MANAGER}/is_active" std_srvs/srv/Trigger "{}" 2>&1 || true
  )"
  printf '%s\n' "$IS_ACTIVE_OUTPUT"
  if printf '%s\n' "$IS_ACTIVE_OUTPUT" | grep -q "success=True"; then
    ok "${LIFECYCLE_MANAGER}/is_active returned success=True"
  elif printf '%s\n' "$IS_ACTIVE_OUTPUT" | grep -q "success=False"; then
    warn "${LIFECYCLE_MANAGER}/is_active returned success=False"
  else
    warn "could not parse ${LIFECYCLE_MANAGER}/is_active response"
  fi
fi

section "lifecycle states"
for node in "${LIFECYCLE_NODES[@]}"; do
  STATE_OUTPUT="$(timeout 5 ros2 lifecycle get "$node" 2>&1 || true)"
  if printf '%s\n' "$STATE_OUTPUT" | grep -q "active \\[3\\]"; then
    ok "$node active [3]"
  else
    fail "$node state: $STATE_OUTPUT"
  fi
done

section "navigate_to_pose action"
ACTION_OUTPUT="$(ros2 action info "$ACTION_NAME" 2>&1 || true)"
printf '%s\n' "$ACTION_OUTPUT"
SERVER_COUNT="$(printf '%s\n' "$ACTION_OUTPUT" | awk '/Action servers:/ {print $3; exit}')"
if [ -n "$SERVER_COUNT" ] && [ "$SERVER_COUNT" -ge 1 ] 2>/dev/null; then
  ok "$ACTION_NAME has $SERVER_COUNT action server(s)"
else
  fail "$ACTION_NAME has no action server"
fi

section "recent Nav2 log errors"
LOG_ROOT="${ROS_LOG_DIR:-$HOME/.ros/log}"
if [ -L "$LOG_ROOT/latest" ] || [ -d "$LOG_ROOT/latest" ]; then
  LATEST_DIR="$(readlink -f "$LOG_ROOT/latest")"
else
  LATEST_DIR="$(find "$LOG_ROOT" -maxdepth 1 -type d -name '20*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}')"
fi

if [ -z "${LATEST_DIR:-}" ] || [ ! -d "$LATEST_DIR" ]; then
  warn "no latest ROS log directory found under $LOG_ROOT"
  exit 0
fi

ok "using log directory: $LATEST_DIR"
ERROR_LINES="$(
  {
    find "$LATEST_DIR" -maxdepth 1 -type f -name '*.log' -print 2>/dev/null
    find "$LOG_ROOT" -maxdepth 1 -type f \
      \( -name 'bt_navigator_*.log' -o -name 'planner_server_*.log' \
         -o -name 'controller_server_*.log' -o -name 'behavior_server_*.log' \
         -o -name 'smoother_server_*.log' -o -name 'lifecycle_manager_navigation_*.log' \) \
      -mmin -30 -print 2>/dev/null
  } | sort -u | xargs -r grep -H -E \
    'ERROR|FATAL|Exception|Failed|failed|Could not|not available|process has died|already been added to an executor' \
    2>/dev/null | tail -40
)"

if [ -z "$ERROR_LINES" ]; then
  ok "no recent Nav2 error lines found"
else
  warn "recent Nav2 error lines:"
  printf '%s\n' "$ERROR_LINES"
fi
