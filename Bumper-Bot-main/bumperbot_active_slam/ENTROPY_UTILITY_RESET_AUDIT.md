# Entropy Utility Reset Audit

## 1. Summary

This audit inspected the current workspace directly:

`/home/hlq017912/Downloads/bumper_bot_active_slam_new_ondinhnhat`

The current codebase should be kept as the stable checkpoint. The default configuration is still the safe frontier/viewpoint baseline:

- `enable_efficient_utility: false`
- `scoring_mode: safe_viewpoint`
- planner validation is enabled and required before dispatch
- high-cost escape, progress gate, reject cache, medium/global fallback, and anti-pingpong controls are present

Recommendation: use Direction 1, Planner-Validated Frontier-Based Active SLAM, as the main direction. Do not continue path entropy as the main direction without new A/B evidence. Direction 2, a simple information-gain utility without path entropy, is worth testing only as an opt-in experimental mode.

## 2. Current Workspace Inspected

Inspected files:

- `bumperbot_active_slam/active_slam_node.py`
- `bumperbot_active_slam/goal_selector.py`
- `bumperbot_active_slam/efficient_active_slam_utility.py`
- `bumperbot_active_slam/information_gain.py`
- `bumperbot_active_slam/path_entropy.py`
- `bumperbot_active_slam/region_diversity.py`
- `bumperbot_active_slam/planner_validator.py`
- `bumperbot_active_slam/path_safety.py`
- `bumperbot_active_slam/costmap_utils.py`
- `bumperbot_active_slam/node_params.py`
- `bumperbot_active_slam/models.py`
- `config/active_slam.yaml`
- `launch/active_slam.launch.py`

Missing file:

- `scripts/compare_active_slam_logs.py` does not exist in this workspace.

## 3. Current Baseline Behavior

Baseline mode is `safe_viewpoint`. It samples candidate frontier cells, then samples viewpoints around each frontier point. Current defaults:

- `goal_candidate_mode: frontier_cell`
- `max_cells_sampled_per_cluster: 100`
- `viewpoint_num_samples: 16`
- `viewpoint_sample_radius_m: 0.45`
- `min_cluster_size_for_navigation: 5`
- `min_candidate_distance_m: 0.6`
- `min_viewpoint_distance_m: 0.8`
- `max_candidate_distance_m: 6.0`
- `max_viewpoint_distance_m: 6.0`

Baseline does use local information gain. It computes the unknown-cell ratio around each viewpoint within `information_radius_m: 0.6`.

Baseline candidate score:

```text
score =
  w_cluster_size * normalized_cluster_size
+ w_distance * distance_score
+ w_information_gain * information_gain
- w_cost_penalty * cost_penalty
- w_goal_switching * 0.0
```

Default weights:

```text
w_cluster_size: 1.0
w_distance: 0.5
w_information_gain: 1.0
w_cost_penalty: 1.0
w_goal_switching: 0.5
```

The configured `w_goal_switching` currently has no effect in the baseline formula because the local variable is always `0.0`.

Baseline uses planner validation before dispatch when `use_planner_validation: true`. The default also has `planner_validation_required_for_navigation: true`, so a selected candidate is not sent to Nav2 unless `ComputePathToPose` returns a path that passes path safety checks.

Baseline has high-cost escape/recovery:

- `high_cost_escape_enabled: true`
- robot high-cost threshold: `70`
- escape samples: `24`, capped by `high_cost_escape_max_attempts_per_cycle: 8`
- escape requires lower cost by default
- escape candidates still go through planner validation

Baseline has anti-pingpong/progress/reject protections:

- navigation goal blacklist
- planner reject cache by point and cluster
- no-path reject cache with separate radius/timeout
- progress gate after a succeeded goal
- stuck detection and cancel/blacklist behavior
- global reposition recent-region penalty and pingpong suppression

Why baseline is likely more stable:

- Its scoring is explainable and low-dimensional.
- It filters candidates before planner validation using map/costmap safety.
- Planner validation can still try next-best candidates, but the initial ranking is not dominated by noisy path entropy.
- Recovery policies are independent of entropy and remain available.

## 4. Current Entropy/Path Entropy Behavior

Entropy utility exists in this codebase:

- `efficient_active_slam_utility.py`
- `path_entropy.py`
- `information_gain.py`
- `region_diversity.py`
- `uncertainty_utils.py`

It is disabled by default. It only activates when both conditions are true:

```text
enable_efficient_utility == true
scoring_mode == "efficient_entropy_utility"
```

The entropy mode does not replace the candidate generation stage. It receives candidates already accepted by the baseline safe viewpoint filters, optionally applies region limiting, caps candidates, then re-ranks them by utility.

Important: the helper `EfficientActiveSlamUtility.filter_clusters()` exists, but the inspected `GoalSelector` does not call it in the local selection loop. Therefore `max_utility_frontier_clusters` is not currently reducing local candidate generation cost.

## 5. Exact Current Utility Formula If Present

Current efficient entropy utility:

```text
utility =
  w_information_gain * information_norm
+ w_path_entropy * path_entropy_norm
+ w_region_diversity * diversity
- w_path_length_penalty * path_length_norm
- w_cost_penalty * cost_norm
- w_uncertainty_penalty * uncertainty.value
- w_goal_switching * 0.0
```

Where:

```text
information_norm = clamp01(unknown_ratio + 0.5 * local_entropy_mean)
path_entropy_norm = clamp01(straight_line_mean_path_entropy)
path_length_norm = min(1.0, candidate.distance / global_reposition_max_distance_m)
cost_norm = clamp01(candidate.cost_penalty)
diversity = region_diversity_score(...)
uncertainty.value = compute_uncertainty_proxy(...)
```

Default weights:

```text
w_information_gain: 1.0
w_path_entropy: 1.0
w_region_diversity: 0.5
w_path_length_penalty: 0.4
w_cost_penalty: 1.0
w_uncertainty_penalty: 0.3
w_goal_switching: 0.5, but multiplied by 0.0
```

Path entropy before planner validation is straight-line entropy from robot to candidate, not Nav2 path entropy:

```text
estimate_straight_line_path_entropy(grid, robot_xy, candidate.point_world)
```

After planner validation accepts a path, the validator optionally computes Nav2-path entropy for logging/enrichment:

```text
compute_path_entropy_for_nav_path(grid, accepted_nav_path, path_entropy_sample_step_m)
```

That post-planner path entropy does not decide the accepted candidate; it is computed after the candidate has already passed validation.

## 6. Complexity Analysis of Entropy Utility

The current entropy formula is too complex for a default/main direction because it combines several partially overlapping signals:

- local information gain
- local entropy mean
- straight-line path entropy
- uncertainty proxy
- region diversity
- rejected-region penalty
- recent-goal penalty
- path length penalty
- cost penalty
- planner validation fallback effects
- high-cost escape priority

Terms with highest complexity/regression risk:

- `path_entropy_norm`: based on straight-line grid cells, not the actual Nav2 path before validation.
- `uncertainty.value`: can penalize goals using another derived proxy, making information gain less interpretable.
- `region_diversity_score`: depends on recent goal and rejected points, so it can be dominated by transient rejection patterns.
- planner validation retry-next-best: can dispatch a candidate different from the entropy top candidate.

Terms that can bias toward nearby/quieter behavior:

- `path_length_norm` penalty.
- cost penalty combined with strict clearance.
- rejected-region penalty if many candidates around useful frontiers were rejected.
- progress/recent-region gates after fallback or recovery behavior.

Terms that can increase planner validation work:

- entropy re-ranking can put candidates with high straight-line uncertainty ahead of candidates with easy Nav2 paths.
- candidate generation is not capped by `max_utility_frontier_clusters`.
- planner validation may test many next-best candidates after the entropy top candidate fails.

## 7. Runtime Risk Analysis

### Candidate Sampling

Each local cycle can sample up to:

```text
valid_clusters * max_cells_sampled_per_cluster * viewpoint_num_samples
```

With defaults:

```text
valid_clusters * 100 * 16
```

There is no global cap on `sampled_viewpoints` during local candidate generation. `max_utility_candidates: 60` applies only after safe candidates have already been generated and filtered. This means entropy mode can still pay the full baseline sampling cost before ranking.

### Planner Validation

Planner validation sorts candidates by current score and tests them with `ComputePathToPose`.

Current defaults:

```text
planner_validation_timeout_sec: 3.0
planner_validation_retry_next_best: true
planner_validation_batch_size: 20
planner_validation_max_batches: 4
max_planner_validation_candidates: 20
```

Because retry-next-best is enabled, effective validation capacity is:

```text
planner_validation_batch_size * planner_validation_max_batches = 80
```

`max_planner_validation_candidates` is only used when retry-next-best is disabled. This is a major reason validation can become slow when top-ranked candidates repeatedly fail.

### Path Clearance

Path safety checks include:

- min/max path length
- max path cost
- unknown rejection
- path interpolation by `path_check_step_m: 0.05`
- clearance radius `path_clearance_radius_m: 0.22`
- fallback relaxed clearance `0.15`
- start ignore radius `0.25`

Strict path clearance is good for safety, but if entropy ranks hard-to-plan candidates first, the validator can spend time rejecting them before finding a safe candidate.

### High-Cost Recovery

High-cost escape has priority over the selected candidate. If robot cost near the base exceeds threshold, entropy is skipped and an escape goal is generated. This is appropriate for safety, but it means entropy selection can be overruled by recovery.

## 8. Why Newer Entropy/Path Entropy Directions Likely Regressed

The code structure supports the observed regression pattern:

- Entropy can select a top candidate before planner validation, but planner validation may send a different candidate after rejecting the top one.
- Straight-line path entropy may not correlate with Nav2 path validity or actual exploration progress.
- Local candidate generation is potentially large and not reduced by `max_utility_frontier_clusters`.
- Retry-next-best can validate up to 80 candidates per selection attempt.
- Path clearance and unknown-path rejection can reject many entropy-preferred candidates.
- Utility can be negative; there is no `reject_negative_utility` gate in current entropy mode.
- Recovery and global reposition can override or bypass the entropy-ranked local candidate.

This explains high selected-to-sent mismatch, slow goal computation, planner timeout accumulation, and final frontier metrics worse than baseline.

## 9. Whether Path Entropy Should Continue As Main Direction

Path entropy should not continue as the main direction based on this audit.

The current path entropy term is not inherently wrong, but it is not strong enough evidence to justify mainline complexity. Before planner validation it is straight-line entropy, not planned-path entropy. After planner validation it is computed too late to choose the goal. This makes it easy for path entropy to add compute and ranking noise without reliably improving dispatched goals.

Keep current entropy/path entropy only as experimental or ablation code. Do not port newer, more complex entropy variants into this stable workspace.

## 10. Recommendation

Direction 1 should be the main direction:

```text
Planner-Validated Frontier-Based Active SLAM for Indoor Mobile Robots
```

or:

```text
Costmap-Aware Frontier-Based Active SLAM with Nav2 Planner Validation
```

Focus should remain on:

- frontier detection
- safe viewpoint sampling
- planner validation
- path clearance
- high-cost recovery
- anti-pingpong
- goal hysteresis/recent-region memory
- metrics/reporting

Direction 2 is worth testing only as an experimental mode:

```text
scoring_mode:=simple_information_gain
```

It should not use path entropy. It should not change default baseline behavior.

## 11. Proposed Simple Utility Formula

Proposal only; not implemented in this audit.

```text
simple_utility =
  w_simple_gain * information_gain
+ w_simple_cluster * cluster_size_score
- w_simple_distance * normalized_path_length
- w_simple_cost * cost_penalty
- w_simple_clearance * path_clearance_risk
- w_simple_recent * recent_region_penalty
```

No `path_entropy` term.

Suggested initial config:

```yaml
enable_simple_information_gain_utility: false
simple_utility_mode: simple_information_gain
w_simple_gain: 1.2
w_simple_cluster: 0.3
w_simple_distance: 0.5
w_simple_cost: 1.0
w_simple_clearance: 0.8
w_simple_recent: 0.4
max_simple_utility_candidates: 30
max_simple_planner_candidates: 8
reject_negative_simple_utility: true
min_simple_utility: 0.0
```

The minimum implementation should live outside `active_slam_node.py`, for example in `information_gain.py`, `scoring_utils.py`, or a small dedicated utility module. `active_slam_node.py` should remain orchestration only.

## 12. Minimal A/B Test Plan

Bringup bookstore:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new_ondinhnhat
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bumperbot_bringup simulated_robot.launch.py \
  use_slam:=true \
  world_name:=bookstore \
  slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml
```

Baseline test:

```bash
rm -f /tmp/active_slam_baseline_ondinhnhat_bookstore.log
ros2 launch bumperbot_active_slam active_slam.launch.py \
  enable_navigation:=true \
  enable_efficient_utility:=false \
  2>&1 | tee /tmp/active_slam_baseline_ondinhnhat_bookstore.log
```

Current entropy test:

```bash
rm -f /tmp/active_slam_entropy_ondinhnhat_bookstore.log
ros2 launch bumperbot_active_slam active_slam.launch.py \
  enable_navigation:=true \
  enable_efficient_utility:=true \
  scoring_mode:=efficient_entropy_utility \
  2>&1 | tee /tmp/active_slam_entropy_ondinhnhat_bookstore.log
```

Compare command from the request cannot run in this workspace until `scripts/compare_active_slam_logs.py` exists:

```bash
python3 Bumper-Bot-main/bumperbot_active_slam/scripts/compare_active_slam_logs.py \
  --baseline-log /tmp/active_slam_baseline_ondinhnhat_bookstore.log \
  --entropy-log /tmp/active_slam_entropy_ondinhnhat_bookstore.log \
  --output-md /tmp/active_slam_ondinhnhat_bookstore_comparison.md
```

For a future simple utility A/B, keep all runtime parameters identical except:

```bash
enable_simple_information_gain_utility:=true
scoring_mode:=simple_information_gain
```

Decision metrics:

- final frontier cells/clusters
- goal sent/succeeded/failed/canceled
- planner timeout rejects per goal
- selected-to-sent mismatch rate
- goal computation time
- sampled viewpoints per cycle
- rejected_by_clearance
- high_cost_escape count
- final travel estimate

## 13. Files That Should Not Be Touched

Do not change these for a simple utility experiment:

- Nav2 planner/controller configuration
- SLAM Toolbox configuration
- `active_slam_node.py` orchestration behavior except minimal parameter wiring if absolutely needed
- existing baseline `safe_viewpoint` behavior
- existing planner validation safety gates
- existing high-cost escape safety policy
- existing path clearance thresholds solely to improve metrics

Do not copy code from other workspaces, roadmap-explorer, m-explore-ros2, ROS1, `rospy`, `actionlib`, or `move_base`.

## 14. active_slam_node.py Line Count

`active_slam_node.py` currently has 798 lines.

It is mostly orchestration: parameters, subscriptions, TF, frontier detection call, goal selector call, planner validator call, high-cost escape call, navigation dispatch, markers, and high-level logging.

There is no large entropy formula in `active_slam_node.py`. The entropy formula is in `efficient_active_slam_utility.py`. Planner validation is in `planner_validator.py`. Path safety is in `path_safety.py`.

Code cleanliness notes:

- `goal_selector.py` is large at 1116 lines and carries selection, fallback, bootstrap, global reposition, scoring, and progress gate logic.
- `node_params.py` is large at 686 lines due to parameter declarations and config construction.
- `active_slam_node.py` has a duplicate `return` in `_update_navigation()` after `enable_navigation=false`.
- `_log_planner_validation_stats()` appears to pass `rejected_by_path_unknown` twice, shifting the displayed `rejected_by_path_clearance` value in the log format. This audit did not change it.

Forbidden ROS1 imports check:

- No `rospy`
- No `actionlib`
- No `move_base`
- No ROS1 `tf` import
- ROS2 `tf2_ros` is used, which is expected

## 15. What Was Not Changed

No runtime behavior was changed.

This audit did not:

- implement `simple_information_gain`
- change baseline defaults
- change entropy weights
- change planner validation
- change Nav2
- change SLAM Toolbox
- copy code from another workspace
- create missing comparison tooling
- relax path safety

Only this report file was added.

Verification run:

```text
colcon build --symlink-install --packages-select bumperbot_active_slam
Result: PASS
```

```text
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
Result: PASS
```

Requested full py_compile command:

```text
python3 -m py_compile \
  Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py \
  Bumper-Bot-main/bumperbot_active_slam/scripts/compare_active_slam_logs.py
Result: FAIL, because scripts/compare_active_slam_logs.py does not exist in this workspace.
```

## 16. Final Recommendation

Keep this codebase as the stable checkpoint.

Baseline should be the main direction. It is already costmap-aware, planner-validated, recovery-capable, and easier to reason about than the entropy utility.

Path entropy should not continue as the main direction. It can remain as experimental/ablation code, but the current implementation has too many ways to add ranking noise, candidate churn, validation retries, and selected-to-sent mismatch.

Simple information gain is worth trying as a small A/B experiment if implemented as an opt-in mode with strict caps:

- keep baseline default unchanged
- no path entropy
- cap utility candidates around 30
- cap planner candidates around 8 for this mode
- reject negative utility
- keep planner validation required
- keep code outside `active_slam_node.py`

If simple information gain does not beat or closely match baseline on final frontier coverage, planner timeout rate, and selected-to-sent consistency, keep Direction 1 as the project direction and treat all utility variants as experimental only.
