# Phase 5 Efficient Multi-Robot Active SLAM Utility Report

## 1. Summary

Phase 5 adds an optional Efficient Multi-robot Active SLAM inspired utility layer on top of the existing Phase 4 pipeline.

Default behavior remains Phase 4 compatible:

```yaml
enable_efficient_utility: false
scoring_mode: safe_viewpoint
```

To enable Phase 5:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true enable_efficient_utility:=true scoring_mode:=efficient_entropy_utility
```

No RRT explorer, multi-robot communication, MPPI, ROS1 dependency, or direct-goal shortcut was added.

## 2. Why Phase 5 Follows Efficient Multi-Robot Active SLAM

The local paper was read from:

```text
/home/hlq017912/Downloads/research_refs/Efficient Multi-robot Active SLAM.pdf
```

The paper emphasizes:

- filtering and prioritizing fewer frontier candidates for computational efficiency
- utility based on frontier path entropy and pose-graph uncertainty
- reward reduction/diversity so robots do not choose the same region
- maintaining exploration performance while reducing utility computation over too many frontiers

For Bumper-Bot, this is adapted as a single-robot utility layer, not a new exploration architecture.

## 3. Why Not The Old RRT/Viewpoint Direction

The current package already has a working ROS2 Humble pipeline:

```text
frontier_detector
-> safe viewpoint / reposition candidates
-> planner validation
-> Nav2 NavigateToPose
```

Returning to the old RRT/viewpoint approach would duplicate candidate generation and risk bypassing Phase 4 safety. Phase 5 therefore only ranks candidates already produced by the current safe candidate pipeline.

## 4. Adapted Ideas For Single Robot

Adapted:

- limit frontier clusters before utility scoring
- limit candidates per region
- compute pre-planner straight-line path entropy
- compute post-planner path entropy from the returned Nav2 path
- compute local information gain around candidate viewpoints
- use a lightweight SLAM uncertainty proxy
- use region diversity instead of multi-robot frontier sharing

Not ported:

- multi-robot frontier sharing
- robot assignment server
- inter-robot communication
- real pose graph covariance or D-optimality over Karto/g2o
- ROS1 code from `aslam_rosbot`
- RRT frontier/viewpoint generator

## 5. Files Changed Or Added

Added:

- `bumperbot_active_slam/path_entropy.py`
- `bumperbot_active_slam/information_gain.py`
- `bumperbot_active_slam/uncertainty_utils.py`
- `bumperbot_active_slam/region_diversity.py`
- `bumperbot_active_slam/efficient_active_slam_utility.py`

Updated:

- `bumperbot_active_slam/models.py`
- `bumperbot_active_slam/node_params.py`
- `bumperbot_active_slam/goal_selector.py`
- `bumperbot_active_slam/planner_validator.py`
- `bumperbot_active_slam/marker_utils.py`
- `bumperbot_active_slam/active_slam_node.py`
- `bumperbot_active_slam/config/active_slam.yaml`
- `bumperbot_active_slam/launch/active_slam.launch.py`

## 6. Module Boundaries

New logic is not inside `active_slam_node.py`:

- `path_entropy.py`: entropy along straight-line estimates and Nav2 paths
- `information_gain.py`: local unknown count and local entropy
- `uncertainty_utils.py`: lightweight uncertainty proxy
- `region_diversity.py`: region limiting and repeated-region penalty
- `efficient_active_slam_utility.py`: final utility scoring and candidate limiting
- `goal_selector.py`: calls the utility layer after Phase 4 candidate generation
- `planner_validator.py`: computes post-planner path entropy on accepted paths
- `node_params.py`: owns config declaration/loading and config object construction

`active_slam_node.py` remains orchestration: params, subscriptions, TF, frontier detector, goal selector, planner validator, navigation dispatcher, markers, and high-level logs.

## 7. active_slam_node.py Line Count

Before Phase 5:

```text
854 lines
```

After Phase 5:

```text
775 lines
```

Net change:

```text
-79 lines
```

The node got smaller because config-object construction was moved to `node_params.py`.

## 8. Path Entropy Formula

For each sampled occupancy cell:

```text
H(p) = -p log2(p) - (1-p) log2(1-p)
```

Probability mapping:

- unknown cell: `p = 0.5`
- free/occupied cell: `p = occupancy_value / 100`

Returned metrics:

- `sum_path_entropy`
- `mean_path_entropy`
- `unknown_ratio_along_path`
- `entropy_sample_count`

Two levels are implemented:

- pre-planner estimate: straight-line grid entropy from robot to candidate
- post-planner exact score: entropy along the Nav2 `ComputePathToPose` path

Pre-planner entropy only ranks candidates before planner validation. It never sends a goal directly.

## 9. Information Gain Formula

`information_gain.py` samples a radius around the candidate:

```yaml
information_radius_m: 0.6
```

It returns:

- `unknown_count`
- `unknown_ratio`
- `local_entropy_sum`
- `local_entropy_mean`

The utility layer uses unknown ratio plus local entropy mean as a lightweight information score.

## 10. Uncertainty Proxy

True pose graph covariance is not available from the current single-robot SLAM Toolbox pipeline. Phase 5 therefore uses a proxy:

- normalized candidate distance
- near-unknown ratio
- optional map staleness component

This is not real pose graph covariance and not D-optimality over the SLAM graph. True SLAM uncertainty should be a later phase if Slam Toolbox/g2o covariance or graph metrics are exposed cleanly.

## 11. Region Diversity Adaptation

The paper uses multi-robot reward spreading so robots do not duplicate work. For single robot Bumper-Bot, this becomes region diversity:

- limit candidates per `region_grid_size_m`
- penalize candidates in the same region as recent goals
- penalize candidates near rejected regions

Config:

```yaml
enable_region_diversity: true
recent_goal_region_penalty: 0.4
rejected_region_penalty: 0.7
region_memory_timeout_sec: 120.0
```

## 12. Final Utility Formula

Implemented utility:

```text
utility =
  w_information_gain * normalized_information_gain
  + w_path_entropy * normalized_path_entropy
  + w_region_diversity * normalized_region_diversity
  - w_path_length_penalty * normalized_path_length
  - w_cost_penalty * normalized_cost
  - w_uncertainty_penalty * normalized_uncertainty_proxy
  - w_goal_switching * goal_switch_penalty
```

Config:

```yaml
enable_efficient_utility: false
scoring_mode: safe_viewpoint
w_information_gain: 1.0
w_path_entropy: 1.0
w_region_diversity: 0.5
w_path_length_penalty: 0.4
w_cost_penalty: 1.0
w_uncertainty_penalty: 0.3
w_goal_switching: 0.5
max_utility_frontier_clusters: 20
max_utility_candidates: 60
max_candidates_per_region: 5
region_grid_size_m: 1.0
path_entropy_sample_step_m: 0.1
normalize_utility_scores: true
```

## 13. Integration With Planner Validation

Pipeline after Phase 5:

```text
frontier_detector
-> goal_selector/viewpoint_sampler safe candidates
-> efficient_active_slam_utility rank candidates
-> planner_validator ComputePathToPose validation
-> navigation_dispatcher NavigateToPose goal
```

Safety remains Phase 4 controlled:

- costmap endpoint checks still apply
- planner validation still gates navigation
- no_path/reject cache still applies
- high_cost_escape still overrides exploration utility
- progress monitor remains active

If the top utility candidate fails planner validation, the existing planner validation batching tries the next candidates.

## 14. Logging

When enabled, expected logs include:

```text
Goal selector: mode=efficient_entropy_utility efficient_utility=True ...
Efficient utility: utility_candidates_before_limit=...
utility_candidates_after_limit=...
top_cluster_count=...
path_entropy=...
information_gain=...
uncertainty_proxy=...
region_diversity=...
final_utility=...
selected_before_planner_validation=(...)
Planner validation accepted: ... post_planner_path_entropy=... unknown_ratio_along_path=...
```

When disabled:

```text
Goal selector: mode=safe_viewpoint efficient_utility=False ...
```

## 15. Test A/B Baseline vs Phase 5

Runtime A/B was not executed in this turn. The build and import tests passed, but a meaningful A/B needs Gazebo/Nav2 running for several minutes in `small_house`.

Do not record invented numbers. Use the same run duration for both tests and collect:

- number of `NavigateToPose result: SUCCEEDED`
- number of `NavigateToPose result: FAILED`
- number of `no planner-valid candidate`
- approximate map coverage
- number of `high_cost_escape`
- elapsed runtime

Baseline:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true enable_efficient_utility:=false
```

Phase 5:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true enable_efficient_utility:=true scoring_mode:=efficient_entropy_utility
```

## 16. Test Commands

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Bringup:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_turning_stable.yaml
```

Baseline:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true enable_efficient_utility:=false
```

Efficient utility:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true enable_efficient_utility:=true scoring_mode:=efficient_entropy_utility
```

## 17. Expected Behavior

Expected:

- Phase 4 baseline still runs unchanged.
- Efficient utility mode logs utility/path entropy.
- Candidate goals remain planner-validated.
- No RRT node or RRT tree appears.
- No multi-robot communication appears.
- high-cost escape remains priority when robot is in high cost.
- `active_slam_node.py` does not grow with utility logic.

## 18. Known Limitations

- No real multi-robot frontier sharing.
- No real pose graph covariance or D-optimality over the Slam Toolbox graph.
- Uncertainty is a proxy.
- No RRT.
- No MPPI.
- Runtime A/B still needs to be executed in Gazebo.

## 19. Verification

Build:

```text
Summary: 1 package finished
```

Syntax/import smoke test:

```text
phase5 utility imports ok
```
