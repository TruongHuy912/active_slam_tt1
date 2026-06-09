# Phase 4.6 Planner-Validated Goals And Collision Diagnostics Report

## 1. Summary

Phase 4.6 adds Nav2 planner validation before `active_slam_explorer` sends `NavigateToPose` goals.

The node still keeps the Phase 4.5 module split. New logic is outside `active_slam_node.py`:

- `planner_validator.py` calls Nav2 `ComputePathToPose`.
- `path_safety.py` checks returned paths against the global costmap.
- `collision_diagnostics.py` and `scripts/collision_diagnostics_check.sh` provide diagnostics for scan/costmap/door issues.

When `use_planner_validation: true` and `planner_validation_required_for_navigation: true`, the node will not send a goal unless Nav2 returns a valid path and that path passes costmap safety checks.

## 2. Why Phase 5 Path Entropy Was Not Added

The current failure mode is path feasibility and collision behavior, not information scoring. Path entropy would rank goals, but it would not prove that a candidate has a feasible Nav2 path through known safe space. Planner validation is the correct next gate before adding entropy or SLAM uncertainty.

## 3. Why MPPI Was Not Ported Now

MPPI can improve local control, smoothing, and obstacle avoidance when the global path is already valid. It does not fix:

- a goal selected inside an unsafe region
- a global path that crosses a wall
- doors/walls missing from `/scan`
- costmaps that do not contain the obstacle
- SLAM maps missing obstacle geometry

Therefore Phase 4.6 first validates global path feasibility. If paths are clean but the robot still clips walls while tracking them, then a later controller phase can evaluate RotationShim + MPPI.

## 4. Root Cause Hypotheses For Door/Wall Contact

The observed collision can come from several layers:

1. Goal selector only checked the endpoint.
   - Fixed in this phase by validating the full Nav2 path before dispatch.
2. Planner/costmap permits unknown traversal.
   - `planner_server.yaml` uses Smac 2D with `allow_unknown: true`.
   - Phase 4.6 path safety rejects unknown path cells when `reject_path_unknown: true`.
3. Door/wall is missing from costmap.
   - If `/scan` does not hit the door or Gazebo collision is missing, Nav2 cannot mark it as an obstacle.
4. Door/wall exists in costmap but path still crosses it.
   - Then planner/costmap/path validation config must be inspected.
5. Path is valid but robot hits while following it.
   - Then controller tuning or MPPI becomes relevant.

No Nav2 planner/controller config was changed in this phase.

## 5. Files Changed

New files:

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/planner_validator.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/path_safety.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/collision_diagnostics.py`
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/node_params.py`
- `Bumper-Bot-main/bumperbot_active_slam/scripts/collision_diagnostics_check.sh`

Updated files:

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
  - only orchestration changes: create validator, start validation, dispatch only after validation result.
  - parameter declaration/loading was moved to `node_params.py` so the node stays focused on orchestration.
- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/marker_utils.py`
  - adds `path_validated` `LINE_STRIP` marker.
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
  - adds planner validation parameters.
- `Bumper-Bot-main/bumperbot_active_slam/package.xml`
  - adds `sensor_msgs` for diagnostics helpers.
- `Bumper-Bot-main/bumperbot_active_slam/README.md`
  - documents planner validation and diagnostics.

## 6. Planner Validation Flow

1. `goal_selector.py` generates and scores safe viewpoint/global reposition candidates.
2. `active_slam_node.py` passes top candidates to `PlannerValidator`.
3. `planner_validator.py` calls Nav2:

```text
/compute_path_to_pose
nav2_msgs/action/ComputePathToPose
planner_id: GridBased
```

4. The validator checks at most:

```yaml
max_planner_validation_candidates: 20
```

5. The first candidate with a returned path that passes `path_safety.py` is selected.
6. Only that planner-validated candidate is sent to `NavigateToPose`.
7. If validation is required and no valid path exists, no goal is sent.

The local interface audit confirmed `nav2_msgs/action/ComputePathToPose` supports `goal`, `start`, `planner_id`, and `use_start` on this Humble install.

## 7. Path Safety Checks

New config:

```yaml
use_planner_validation: true
planner_action_name: /compute_path_to_pose
planner_id: GridBased
planner_validation_timeout_sec: 3.0
min_valid_path_length_m: 0.2
max_valid_path_length_m: 15.0
max_path_cost: 70
reject_path_unknown: true
path_check_step_m: 0.05
path_clearance_radius_m: 0.22
max_planner_validation_candidates: 20
planner_validation_required_for_navigation: true
```

Validation rejects:

- planner timeout
- rejected/unavailable planner action
- empty path
- path shorter than `min_valid_path_length_m`
- path longer than `max_valid_path_length_m`
- path cells with cost above `max_path_cost`
- unknown path cells when `reject_path_unknown: true`
- path samples without `path_clearance_radius_m` clearance

Returned path poses and interpolated samples are checked at `path_check_step_m`.

## 8. Door/Wall/Costmap Diagnostics

Use:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/collision_diagnostics_check.sh
```

It prints:

- action list and `/compute_path_to_pose` info
- `/navigate_to_pose` info
- one `/scan` sample and `/scan` hz
- `/global_costmap/costmap` info and header/info sample
- `/local_costmap/costmap` info and header/info sample
- Nav2 lifecycle states

Relevant audited Nav2 config:

- `planner_server.yaml`
  - Smac 2D planner under `GridBased`
  - `allow_unknown: true`
  - global costmap has static, obstacle, and inflation layers
  - global costmap listens to `/scan`
  - `robot_radius: 0.1`
  - `inflation_radius: 0.55`
- `controller_server.yaml`
  - local costmap has obstacle and inflation layers
  - local costmap listens to `/scan`
  - RPP collision detection is enabled

If doors/walls are absent from `/scan` or costmaps, active_slam cannot reliably reject paths through them. That is a world/sensor/costmap issue, not a frontier selector issue.

## 9. Ideas From m-explore-ros2

Used as design ideas only:

- blacklist failed goals
- avoid repeated goal dispatch
- avoid retrying unreachable goals forever
- treat planner/costmap failures as goal rejection signals

No code was copied.

## 10. Ideas From roadmap-explorer

Used as design ideas only:

- validate candidate usefulness before committing to it
- avoid greedy local choices
- preserve the global reposition concept from Phase 4.5
- keep scheduling/candidate validation separate from ROS node orchestration

Not ported:

- roadmap graph
- TSP scheduler
- raytraced gain engine
- lifecycle/plugin framework
- package source code

## 11. SLAM Toolbox Profile Status

No SLAM Toolbox tuning was changed in Phase 4.6.

The Phase 4.5 fast simulation profile remains available:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house slam_config:=$(ros2 pkg prefix bumperbot_mapping)/share/bumperbot_mapping/config/slam_toolbox_sim_fast.yaml
```

`slam_config` is still exposed by `simulated_robot.launch.py`.

Do not use the fast profile by default on Raspberry Pi until CPU/RAM load has been measured. No scan matcher or loop closure parameters were changed.

## 12. Commands Test

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam bumperbot_mapping bumperbot_bringup bumperbot_navigation
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Terminal 1:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

Terminal 2:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh
```

Terminal 3:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=false
```

Terminal 4:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

Collision diagnostics:

```bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/collision_diagnostics_check.sh
```

Runtime action audit:

```bash
ros2 action list | grep compute
ros2 action info /compute_path_to_pose
```

Expected:

```text
Action servers: 1
```

## 13. Expected Logs

Candidate generation:

```text
Goal selection: mode=safe_viewpoint ...
```

Planner validation start:

```text
Planner validation started: action=/compute_path_to_pose candidates_before_planner_validation=...
```

Planner accepted:

```text
Planner validation accepted: candidate=(x, y) path_length=... max_cost=... validated_count=...
```

Summary:

```text
Planner validation summary: candidates_before_planner_validation=... planner_validated_count=... rejected_by_planner_timeout=... rejected_by_no_path=... rejected_by_path_cost=... rejected_by_path_unknown=... rejected_by_path_clearance=... selected_path_length=... selected_candidate_after_planner_validation=True
```

Dispatch:

```text
Sending NavigateToPose goal: state=WAITING_FOR_SERVER ...
NavigateToPose goal accepted: state=NAVIGATING ...
```

No valid path:

```text
Navigation skip: state=IDLE enable_navigation=True reason=no planner-validated candidate
```

No goal spam:

```text
Navigation skip: state=NAVIGATING enable_navigation=True reason=currently navigating
```

RViz marker:

```text
namespace: path_validated
```

## 14. When To Integrate MPPI Later

Integrate RotationShim + MPPI only after:

1. `/scan` clearly sees the door/wall.
2. Global and local costmaps mark the obstacle.
3. Planner validation returns a path that avoids the obstacle.
4. The robot still collides while following that valid path.

At that point MPPI may help with:

- smoother local control
- better obstacle-aware local rollouts
- tight door/corridor handling
- reducing wall clipping while tracking a valid path

MPPI should not be used to mask bad goals, missing costmap obstacles, or paths that already cross walls.

## 15. Known Limitations

- No Phase 5 path entropy.
- No SLAM pose graph uncertainty.
- No MPPI or controller changes.
- Planner validation depends on Nav2 costmaps being correct.
- If Gazebo doors do not have collision or LiDAR returns do not hit them, costmap/path validation cannot infer the obstacle.
- Validation is asynchronous and limited to `max_planner_validation_candidates` to avoid runtime lag.

## 16. Verification Performed

Build result:

```text
Summary: 4 packages finished
```

Syntax/import checks:

```bash
source install/setup.bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
python3 -c "from bumperbot_active_slam.planner_validator import PlannerValidator, PlannerValidationConfig; from bumperbot_active_slam.path_safety import validate_path_safety; from nav2_msgs.action import ComputePathToPose; print('planner validation imports ok')"
```

Result:

```text
planner validation imports ok
```

`simulated_robot.launch.py --show-args` with `ROS_LOG_DIR=/tmp/ros_logs` confirmed `slam_config` remains exposed.
