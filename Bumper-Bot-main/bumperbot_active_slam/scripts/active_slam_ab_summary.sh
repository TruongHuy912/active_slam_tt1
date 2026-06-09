#!/usr/bin/env bash
set -euo pipefail

LOG_TARGET="${1:-${HOME}/.ros/log/latest}"

if [ ! -e "${LOG_TARGET}" ]; then
  echo "FAIL: log path does not exist: ${LOG_TARGET}"
  echo "Usage: $0 [log_file_or_log_directory]"
  exit 1
fi

count_pattern() {
  local pattern="$1"
  if [ -d "${LOG_TARGET}" ]; then
    grep -R -h -E "${pattern}" "${LOG_TARGET}" 2>/dev/null | wc -l | tr -d ' '
  else
    grep -E "${pattern}" "${LOG_TARGET}" 2>/dev/null | wc -l | tr -d ' '
  fi
}

sum_rejected_field() {
  local field="$1"
  local matches
  if [ -d "${LOG_TARGET}" ]; then
    matches="$(grep -R -h -E "${field}=[0-9]+" "${LOG_TARGET}" 2>/dev/null || true)"
  else
    matches="$(grep -E "${field}=[0-9]+" "${LOG_TARGET}" 2>/dev/null || true)"
  fi
  if [ -z "${matches}" ]; then
    echo 0
    return
  fi
  echo "${matches}" | sed -E "s/.*${field}=([0-9]+).*/\\1/" | awk '{sum += $1} END {print sum + 0}'
}

echo "Active SLAM A/B summary"
echo "Log target: ${LOG_TARGET}"
echo
echo "NavigateToPose SUCCEEDED: $(count_pattern 'NavigateToPose result: SUCCEEDED')"
echo "NavigateToPose FAILED: $(count_pattern 'NavigateToPose result: FAILED|NavigateToPose result:.*FAILED')"
echo "Efficient utility log count: $(count_pattern 'Efficient utility:|Phase5 utility: enable_efficient_utility=True')"
echo "Goal selection safe_viewpoint count: $(count_pattern 'Goal selection: mode=safe_viewpoint')"
echo "Goal selection efficient_entropy_utility count: $(count_pattern 'Goal selection: mode=efficient_entropy_utility')"
echo "Planner validation accepted count: $(count_pattern 'Planner validation accepted')"
echo "Rejected clearance total: $(sum_rejected_field 'rejected_clearance')"
echo "Rejected no_path total: $(sum_rejected_field 'rejected_no_path')"
echo "High-cost escape count: $(count_pattern 'high_cost_escape')"
echo "No planner-valid candidate count: $(count_pattern 'no planner-valid candidate')"
