# Nav2 Lifecycle And Goal Selection Fix Report

## 1. Root Cause For `/navigate_to_pose` Action Servers: 0

`/navigate_to_pose` has no action server when `bt_navigator` is not active.

The Bumper-Bot navigation launch was audited:

- `bumperbot_bringup/launch/simulated_robot.launch.py` includes `bumperbot_navigation/launch/navigation.launch.py`.
- `bumperbot_navigation/launch/navigation.launch.py` starts `controller_server`, `planner_server`, `smoother_server`, `bt_navigator`, `behavior_server`, and `lifecycle_manager_navigation`.
- `lifecycle_manager_navigation` already has:
  - `autostart: True`
  - `node_names: ['controller_server', 'planner_server', 'smoother_server', 'bt_navigator', 'behavior_server']`

The live ROS graph showed duplicate Nav2 node names:

```text
/behavior_server
/behavior_server
/bt_navigator
/bt_navigator
/controller_server
/controller_server
/lifecycle_manager_navigation
/lifecycle_manager_navigation
```

That means Nav2 was very likely launched more than once, for example by running
`simulated_robot.launch.py` and also launching `navigation.launch.py` separately.
Duplicate lifecycle node names make lifecycle queries ambiguous and can leave
the `bt_navigator` instance you query in `unconfigured`, so it does not create
the `/navigate_to_pose` action server.

No lifecycle-manager file defect was found in the audited launch/config.

## 2. Files Changed For Nav2 Lifecycle

No Nav2 lifecycle launch/config file was changed.

The existing lifecycle config is already correct for a single clean Nav2 launch:

```text
autostart: True
node_names: controller_server, planner_server, smoother_server, bt_navigator, behavior_server
```

Documentation was updated in:

- `Bumper-Bot-main/bumperbot_active_slam/README.md`

The README now warns that `simulated_robot.launch.py` already includes
`navigation.launch.py`, so Nav2 should not be launched a second time unless the
simulation is being assembled from lower-level component launches.

## 3. Correct Launch Commands

Use a clean ROS graph first. Stop old launch terminals before retesting.

Terminal 1:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

This single launch starts Gazebo, robot control, SLAM Toolbox, Nav2, and RViz.
Do not also run `bumperbot_navigation navigation.launch.py` in another terminal
when using this bringup launch.

Terminal 2, marker-only:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

Terminal 2, with goal dispatch enabled:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

If you intentionally start simulation from lower-level pieces instead of
`simulated_robot.launch.py`, then launch Nav2 exactly once:

```bash
ros2 launch bumperbot_navigation navigation.launch.py use_sim_time:=true
```

## 4. Expected Output

Action server:

```bash
ros2 action info /navigate_to_pose
```

Expected:

```text
Action servers: 1
```

Lifecycle:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
```

Expected:

```text
active [3]
active [3]
active [3]
active [3]
```

Also check for duplicate node names:

```bash
ros2 node list | sort | uniq -d
```

Expected: no duplicate Nav2 names.

## 5. Active SLAM Goal Selection Changes

Files changed:

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
- `Bumper-Bot-main/bumperbot_active_slam/README.md`

New parameter:

```yaml
min_candidate_distance_m: 0.6
```

Behavior changes:

- Frontier clusters closer than `min_candidate_distance_m` are ignored as
  navigation candidates.
- The node no longer sets `SUCCEEDED` just because the current best frontier is
  within `goal_reached_distance_m` before any goal has been sent.
- `goal_reached_distance_m` is now only used while a sent goal is active, to log
  that the robot is near the active goal while waiting for the Nav2 result.
- If every candidate is too close, the node stays `IDLE` and logs:

```text
Navigation skip: state=IDLE enable_navigation=True reason=all frontier candidates are closer than min_candidate_distance_m
```

Goal selection logs include:

- total frontier clusters
- rejected-too-close count
- rejected-blacklisted count
- selected candidate id and distance
- skip reason

Example:

```text
Goal selection: total_frontier_clusters=2 rejected_too_close=1 rejected_blacklisted=0 selected_id=... selected_distance=...
```

## 6. Test With `enable_navigation=false`

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

Expected:

```text
Navigation dispatch: enable_navigation=False, action_name=/navigate_to_pose, state=IDLE
Navigation skip: state=IDLE enable_navigation=False reason=enable_navigation=false
```

No NavigateToPose goal should be sent.

## 7. Test With `enable_navigation=true`

First verify Nav2:

```bash
ros2 action info /navigate_to_pose
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
```

Then:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

Expected when a valid frontier farther than `min_candidate_distance_m` exists:

```text
Goal selection: total_frontier_clusters=... rejected_too_close=... rejected_blacklisted=... selected_id=... selected_distance=...
Sending NavigateToPose goal: state=WAITING_FOR_SERVER frame=map x=... y=...
NavigateToPose goal accepted: state=NAVIGATING goal=(..., ...)
```

If all frontiers are near the robot:

```text
Navigation skip: state=IDLE enable_navigation=True reason=all frontier candidates are closer than min_candidate_distance_m
```

Move or rotate the robot to expand the map and create farther frontiers.

## 8. Verification

Full workspace build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install
```

Result:

```text
Summary: 14 packages finished
```

Subset build after README/config/code update:

```bash
colcon build --symlink-install --packages-select bumperbot_active_slam bumperbot_navigation bumperbot_bringup
```

Result:

```text
Summary: 3 packages finished
```

Python syntax:

```bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Result: passed.

## 9. Known Limitations

- No costmap rejection yet.
- No safe viewpoint sampling yet.
- No path entropy scoring yet.
- Goal pose is still the selected frontier centroid, not a costmap-validated
  safe viewpoint.
