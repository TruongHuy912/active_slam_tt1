# Phase 5.3 Post-Escape Progress Gate Fix Report

## 1. Summary

Phase 5.3 fixes the stall after a successful `high_cost_escape` goal. The robot could escape a high-cost region, receive `NavigateToPose result: SUCCEEDED`, then repeatedly skip selection with:

```text
rejected_by_progress=1
sampled_frontier_cells=0
sampled_viewpoints=0
skip_reason=robot has not progressed enough since previous goal
```

The fix changes the progress gate from a hard pre-sampling return into a bounded gate with timeout, skip-cycle limit, and post-escape relaxation.

No MPPI, RRT, new exploration node, or planner validation bypass was added.

## 2. Root Cause Of Post-Escape Stuck

The lock came from the first branch in `GoalSelector.select()`:

```text
nav_state == SUCCEEDED
last_goal_robot_xy is not None
robot moved less than min_goal_progress_distance_m
```

That branch returned before candidate sampling. After `high_cost_escape`, the goal is intentionally short, so the robot may not exceed the normal progress threshold. The selector then skipped before frontier sampling, producing:

```text
sampled_frontier_cells=0
sampled_viewpoints=0
local_candidates=0
```

This was not an entropy scoring failure. It was a progress gate blocking exploration before candidate generation.

## 3. Files Changed

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/goal_selector.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/navigation_dispatcher.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/models.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/node_params.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/README.md`

## 4. `active_slam_node.py` Line Count

Before Phase 5.3:

```text
795 lines
```

After Phase 5.3:

```text
798 lines
```

Increase:

```text
3 lines
```

The node only passes `last_goal_source`, `last_goal_result`, and result age into `goal_selector`. The progress policy stays in `goal_selector.py`.

`active_slam_node.py` remains orchestration-only.

## 5. New Config Params

Added to `active_slam.yaml`:

```yaml
progress_gate_enabled: true
progress_gate_min_distance_m: 0.25
progress_gate_timeout_sec: 8.0
progress_gate_max_skip_cycles: 2
progress_gate_disable_after_escape: true
post_escape_resume_delay_sec: 1.0
post_escape_force_selection_cycles: 2
```

These are declared/read in `node_params.py` and stored in `GoalSelectorConfig`.

## 6. Progress Gate Timeout Logic

The progress gate now tracks:

- age of the gate
- skip count
- robot movement since previous goal
- previous goal source/result

It allows new selection if:

- robot moved at least `progress_gate_min_distance_m`
- gate age exceeds `progress_gate_timeout_sec`
- skip count exceeds `progress_gate_max_skip_cycles`

Expected logs:

```text
progress_gate_timeout: allowing new goal selection age=... moved=... threshold=...
progress_gate_max_skip_cycles reached; resuming exploration skip_count=...
```

When it does skip, the reason now includes:

```text
progress_gate_age_sec
progress_gate_skip_count
previous_goal_source
previous_goal_result
robot_moved_since_previous_goal
threshold
```

## 7. Post-Escape Resume Logic

`navigation_dispatcher.py` now records:

- `last_goal_source`
- `last_goal_result`
- `last_result_time`

Successful results log the source:

```text
NavigateToPose result: SUCCEEDED source=high_cost_escape
```

If the previous goal was `high_cost_escape` and succeeded, `goal_selector.py` relaxes the progress gate after `post_escape_resume_delay_sec` and forces selection for `post_escape_force_selection_cycles`.

Expected logs:

```text
post_escape_resume: high_cost_escape succeeded, progress gate relaxed age=...
post_escape_resume: forcing selection despite progress gate remaining_cycles=...
```

After that, normal exploration resumes:

```text
Goal selection: ... sampled_frontier_cells > 0 sampled_viewpoints > 0
Efficient utility: ...
```

## 8. High-Cost Escape Marker Explanation

The circles/candidate points seen around the robot during high-cost recovery are debug markers for nearby escape candidates, planner rejects, or blacklist/reject cache regions.

`high_cost_escape` is prioritized when the robot is in high/inflated cost. It samples short nearby goals in known free space. Those goals are still planner-validated before `NavigateToPose` dispatch.

The README now documents this behavior and clarifies that these markers are not direct velocity commands.

## 9. Chair Collision Note

If the robot hits a chair because the LiDAR does not observe that obstacle, this is primarily a sensing/costmap/world-model issue, not an entropy utility issue.

MPPI also cannot reliably avoid an obstacle that is missing from the costmap. A later phase should audit:

- robot footprint and inflation radius
- obstacle layer sources
- LiDAR height and field of view
- chair/world collision geometry
- whether another sensor is needed

MPPI should only be considered after valid goals/paths and costmap obstacle visibility are confirmed.

## 10. Test Commands

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

## 11. Expected Logs

Efficient utility still active:

```text
Phase5 utility: enable_efficient_utility=True scoring_mode=efficient_entropy_utility
Efficient utility: ...
```

High-cost escape:

```text
high_cost_escape: robot_cost=... selected_escape=(...)
Sending NavigateToPose goal: ... source=high_cost_escape
NavigateToPose result: SUCCEEDED source=high_cost_escape
```

Resume after escape:

```text
post_escape_resume: high_cost_escape succeeded, progress gate relaxed
Goal selection: ... sampled_frontier_cells=... sampled_viewpoints=...
```

The old infinite loop should not continue:

```text
sampled_frontier_cells=0 sampled_viewpoints=0 rejected_by_progress=1
skip_reason=robot has not progressed enough since previous goal
```

for more than the configured timeout/skip cycles.

## 12. Verification

Build:

```text
Summary: 1 package finished
```

Syntax/import:

```text
phase53 imports ok
```

Runtime Gazebo test was not executed in this turn.

## 13. Known Limitations

- This does not solve obstacles missing from LiDAR/costmap.
- No MPPI was added.
- No RRT/viewpoint generator was added.
- No multi-robot communication was added.
- Planner validation remains mandatory.
