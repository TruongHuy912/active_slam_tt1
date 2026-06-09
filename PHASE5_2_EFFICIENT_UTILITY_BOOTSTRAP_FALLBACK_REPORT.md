# Phase 5.2 Efficient Utility Bootstrap Fallback Report

## 1. Summary

Phase 5.2 fixes the early-map stall when running:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py \
  enable_navigation:=true \
  enable_efficient_utility:=true \
  scoring_mode:=efficient_entropy_utility
```

The runtime showed Phase 5 was enabled, but no candidates reached utility ranking:

```text
Efficient utility skipped: no safe candidates
local_candidates=0
sampled_frontier_cells=105
sampled_viewpoints=105
rejected_by_clearance=87
```

The fix makes efficient utility a ranking layer only. It no longer changes safe viewpoint generation before candidates exist. If there are too few/no safe candidates during bootstrap, the selector falls back to baseline `safe_viewpoint` behavior or creates a short known-free-space bootstrap candidate that still goes through planner validation.

No RRT, MPPI, multi-robot communication, new exploration node, or planner-validation bypass was added.

## 2. Root Cause

Efficient mode stood still because candidate generation changed before utility ranking:

- Baseline `safe_viewpoint` mode sampled viewpoints around frontier cells.
- Efficient mode used `scoring_mode=efficient_entropy_utility`.
- The viewpoint sampler was called with:

```python
self.config.scoring_mode == "safe_viewpoint"
```

That was false in efficient mode, so it tried the frontier cell itself instead of safe viewpoints around it. Frontier cells are often adjacent to unknown space and failed map/costmap clearance checks, leaving `local_candidates=0`.

There was a second risk: `utility.filter_clusters()` ran before candidate generation, which could narrow frontier clusters too early. Phase 5.2 stops using utility filtering as a pre-candidate hard filter.

## 3. Baseline vs Efficient Code Path

Before:

```text
efficient mode -> utility cluster filter -> frontier cell direct candidate -> clearance reject -> no utility ranking
```

After:

```text
frontier clusters
-> Phase 4 safe viewpoint candidate generation
-> if efficient utility enabled and enough safe candidates: rank candidates
-> else baseline fallback/bootstrap
-> planner validation
-> NavigateToPose
```

Baseline mode remains:

```text
frontier clusters -> safe viewpoint candidates -> baseline score -> planner validation -> NavigateToPose
```

## 4. Files Changed

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/goal_selector.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/models.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/node_params.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/README.md`

## 5. `active_slam_node.py` Line Count

Before Phase 5.2:

```text
792 lines
```

After Phase 5.2:

```text
795 lines
```

Increase:

```text
3 lines
```

The node only logs the new `utility_candidates` count. Candidate generation, fallback, bootstrap, and utility policy are all in `goal_selector.py`, `models.py`, and `node_params.py`.

`active_slam_node.py` remains orchestration-only.

## 6. Utility-As-Ranking-Only Confirmation

Efficient utility now ranks only after safe candidates exist.

In `goal_selector.py`:

- local candidate generation uses safe viewpoint sampling for both:
  - `safe_viewpoint`
  - `efficient_entropy_utility`
- utility pre-filtering is not used before safe candidate generation
- utility ranking runs only when:

```text
safe_candidates >= utility_min_safe_candidates_before_ranking
```

If fewer candidates exist, baseline scoring is used for that cycle.

## 7. Baseline Fallback Logic

New config:

```yaml
enable_utility_fallback_to_baseline: true
utility_fallback_when_no_candidates: true
utility_min_safe_candidates_before_ranking: 5
```

Expected logs:

```text
Efficient utility fallback: no safe candidates, trying baseline selector
Efficient utility fallback: using baseline safe_viewpoint safe_candidates=... min_required=5
Goal selection: mode=efficient_entropy_utility efficient_utility=True selected_mode=baseline_fallback ...
```

If baseline fallback finds a candidate, it still goes through `ComputePathToPose` validation before `NavigateToPose`.

## 8. Bootstrap Exploration Logic

New config:

```yaml
enable_bootstrap_exploration: true
bootstrap_max_cycles_without_goal: 3
bootstrap_min_goal_distance_m: 0.4
bootstrap_max_goal_distance_m: 1.2
bootstrap_allow_relaxed_clearance: true
bootstrap_safety_radius_m: 0.15
bootstrap_use_known_free_space: true
utility_bootstrap_min_frontier_clusters: 5
utility_bootstrap_min_robot_travel_m: 0.5
```

Bootstrap is considered when efficient mode is enabled and the map is still early/sparse:

- robot has not moved at least `utility_bootstrap_min_robot_travel_m`
- or frontier cluster count is below `utility_bootstrap_min_frontier_clusters`
- or repeated no-candidate cycles happen

Bootstrap candidates are short known-free-space goals around the robot. They are not sent directly; planner validation remains mandatory.

Expected log:

```text
Bootstrap exploration: selected candidate=(x, y) distance=... reason=initial_no_safe_candidates
Goal selection: mode=efficient_entropy_utility efficient_utility=True selected_mode=bootstrap ...
```

## 9. New Config

Added to `active_slam.yaml`:

```yaml
enable_utility_fallback_to_baseline: true
utility_fallback_when_no_candidates: true
utility_min_safe_candidates_before_ranking: 5
utility_bootstrap_min_frontier_clusters: 5
utility_bootstrap_min_robot_travel_m: 0.5
enable_bootstrap_exploration: true
bootstrap_max_cycles_without_goal: 3
bootstrap_min_goal_distance_m: 0.4
bootstrap_max_goal_distance_m: 1.2
bootstrap_allow_relaxed_clearance: true
bootstrap_safety_radius_m: 0.15
bootstrap_use_known_free_space: true
```

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

Baseline:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py \
  enable_navigation:=true \
  enable_efficient_utility:=false
```

Efficient utility:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py \
  enable_navigation:=true \
  enable_efficient_utility:=true \
  scoring_mode:=efficient_entropy_utility
```

## 11. Expected Logs

Efficient mode startup:

```text
Phase5 utility: enable_efficient_utility=True scoring_mode=efficient_entropy_utility
Goal selector: mode=efficient_entropy_utility efficient_utility=True
```

Bootstrap/fallback early map:

```text
Efficient utility fallback: no safe candidates, trying baseline selector
Bootstrap exploration: selected candidate=(..., ...) distance=... reason=initial_no_safe_candidates
Planner validation accepted: ...
Sending NavigateToPose goal: ...
```

When enough safe candidates exist:

```text
Efficient utility: utility_candidates_before_limit=... utility_candidates_after_limit=...
best_utility=... path_entropy=... information_gain=...
Goal selection: mode=efficient_entropy_utility efficient_utility=True selected_mode=efficient_utility ...
```

The old stall should not repeat indefinitely:

```text
Efficient utility skipped: no safe candidates
selected=none
```

without fallback/bootstrap attempts.

## 12. Verification

Build:

```text
Summary: 1 package finished
```

Syntax:

```bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Result: passed.

Import smoke:

```text
phase52 imports ok
```

Runtime Gazebo A/B was not executed in this turn.

## 13. Known Limitations

- Efficient utility still depends on Phase 4 candidate quality.
- Bootstrap is a short known-free-space reposition helper, not RRT.
- No MPPI was added.
- No multi-robot communication or frontier sharing was added.
- No planner validation bypass was added.
