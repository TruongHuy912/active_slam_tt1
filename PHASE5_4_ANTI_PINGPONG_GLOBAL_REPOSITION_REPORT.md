# Phase 5.4 Anti-Pingpong Global Reposition Report

## 1. Summary

Phase 5.4 adds anti-ping-pong control around `global_reposition`. Runtime showed the robot alternating between repeated fallback goals such as:

```text
(-5.98, -4.89)
(-8.11, -3.30)
(-5.95, -4.91)
(-8.08, -3.31)
```

Those goals were `source=global_reposition` with `information_gain=0.000`, so the fallback was acting like the main exploration policy.

The fix adds recent-region memory, global reposition cooldown/source policy, zero-gain usefulness checks, and ping-pong suppression. Efficient utility, high-cost escape, and planner validation remain enabled.

No MPPI, RRT, new exploration node, or planner-validation bypass was added.

## 2. Root Cause Of Ping-Pong

The ping-pong came from the fallback path:

```text
Efficient utility skipped: no safe candidates
Efficient utility fallback: no safe candidates, trying baseline selector
fallback_mode=global_reposition
selected_source=global_reposition
information_gain=0.000
```

Root causes:

- `global_reposition` did not have a dedicated recent-goal cooldown.
- Recent-region diversity was mostly utility-ranking oriented and did not strongly gate global fallback.
- Zero-gain global reposition candidates could still be selected.
- Consecutive global reposition goals were not limited.
- There was no A/B ping-pong detector for repeated fallback regions.

## 3. Files Changed

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/goal_selector.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/models.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/node_params.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/README.md`

## 4. `active_slam_node.py` Line Count

Before Phase 5.4:

```text
798 lines
```

After Phase 5.4:

```text
798 lines
```

Increase:

```text
0 lines
```

All anti-ping-pong logic is in `goal_selector.py` and config/model files. `active_slam_node.py` remains orchestration-only.

## 5. New Config Params

Added:

```yaml
global_reposition_max_consecutive_goals: 2
global_reposition_cooldown_sec: 20.0
global_reposition_recent_goal_radius_m: 1.2
global_reposition_recent_region_penalty: 0.8
global_reposition_min_information_gain: 0.02
global_reposition_allow_zero_gain_only_if_no_alternative: true
global_reposition_blacklist_after_success_sec: 45.0
global_reposition_pingpong_window: 6
global_reposition_pingpong_radius_m: 1.0
recent_goal_region_timeout_sec: 120.0
recent_goal_region_radius_m: 1.0
enable_goal_usefulness_gate: true
min_goal_information_gain: 0.01
min_goal_frontier_distance_from_recent_m: 1.0
min_expected_frontier_reduction: 0
allow_low_gain_recovery_goal: true
post_global_reposition_prefer_frontier_cycles: 2
post_global_reposition_wait_for_map_update_sec: 1.0
```

## 6. Global Reposition Cooldown Logic

`goal_selector.py` records successful goal source/result and tracks consecutive `global_reposition` successes.

Global reposition is suppressed when:

- consecutive global reposition count reaches `global_reposition_max_consecutive_goals`
- post-global reposition cycles should prefer frontier/local candidates
- a candidate is inside `global_reposition_recent_goal_radius_m`
- a candidate matches ping-pong regions
- a zero-gain candidate is near recently visited regions

Expected reject logs:

```text
global_reposition_rejected: reason=recent_region ...
global_reposition_rejected: reason=zero_gain ...
global_reposition_rejected: reason=max_consecutive ...
global_reposition_rejected: reason=prefer_frontier_after_success ...
```

## 7. Recent Region Memory Logic

Recent goal memory stores:

- point
- source
- result
- timestamp

It applies to global reposition and low-gain recovery. `high_cost_escape` is ignored by recent-region rejection so safety escape remains available.

The memory expires using:

```yaml
recent_goal_region_timeout_sec: 120.0
global_reposition_blacklist_after_success_sec: 45.0
```

## 8. Ping-Pong Detector Logic

The selector inspects recent successful global reposition goals within:

```yaml
global_reposition_pingpong_window: 6
global_reposition_pingpong_radius_m: 1.0
```

If it detects repeated A/B regions, it logs:

```text
pingpong_detected: suppressing global_reposition regions for cooldown
```

Candidates near those recent global reposition regions are rejected with:

```text
global_reposition_rejected: reason=pingpong
```

## 9. Goal Usefulness Gate

Global reposition is no longer treated as useful just because Nav2 can plan to it.

Useful goals should have at least one of:

- information gain above `min_goal_information_gain`
- useful frontier/unknown context
- a region not recently visited
- a recovery justification

Zero-gain global reposition is allowed only as recovery and is suppressed near recent regions.

## 10. High-Cost Escape Priority

`high_cost_escape` is not weakened:

- It still overrides normal utility selection when robot cost is above threshold.
- Recent-region global reposition penalties do not block `high_cost_escape`.
- Post-escape resume from Phase 5.3 remains in place.

This phase only prevents global fallback from oscillating between old regions after the robot is not in immediate high-cost recovery.

## 11. Test Commands

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bumperbot_active_slam
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Bringup:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py \
  use_slam:=true \
  world_name:=small_house \
  slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml
```

Entropy test:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py \
  enable_navigation:=true \
  enable_efficient_utility:=true \
  scoring_mode:=efficient_entropy_utility
```

## 12. Expected Logs

Efficient utility still active:

```text
Phase5 utility: enable_efficient_utility=True scoring_mode=efficient_entropy_utility
Efficient utility: ...
```

Global reposition summary:

```text
Global reposition summary: recent_goal_regions=... consecutive_global_reposition_count=...
global_reposition_cooldown_active=... pingpong_detected=...
candidate_rejected_recent_region=... candidate_rejected_zero_gain=...
selected_source=... selected_information_gain=...
```

Ping-pong suppression:

```text
pingpong_detected: suppressing global_reposition regions for cooldown
global_reposition_rejected: reason=pingpong ...
```

The robot should not repeatedly alternate between the same two `global_reposition` regions. If no useful goal exists, the selector should reject repeated fallback regions and wait or select a different region rather than ping-pong.

## 13. Known Limitations

- This does not create a full roadmap/TSP scheduler.
- It does not add MPPI.
- It does not add RRT.
- It does not solve missing obstacles in the costmap.
- Global reposition is still a simplified fallback, not the main exploration planner.
