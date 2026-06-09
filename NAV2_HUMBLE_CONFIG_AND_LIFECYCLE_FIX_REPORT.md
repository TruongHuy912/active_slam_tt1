# Nav2 Humble Config And Lifecycle Fix Report

## 1. Root Cause

The root cause was ROS 2 Humble plugin ID mismatch in Nav2 config.

Two configs used plugin class IDs that do not match the plugin descriptions
installed on this machine:

```text
nav2_behaviors::Spin
nav2_behaviors::BackUp
nav2_behaviors::Wait
nav2_smac_planner::SmacPlanner2D
```

The local Humble plugin XML declares these as:

```text
nav2_behaviors/Spin
nav2_behaviors/BackUp
nav2_behaviors/Wait
nav2_smac_planner/SmacPlanner2D
```

Because of this:

- `behavior_server` failed to configure while creating `spin`.
- `planner_server` failed to configure while creating the Smac planner.
- `bt_navigator` could not load the default BT XML because planner actions such
  as `compute_path_through_poses` were not available.
- `/navigate_to_pose` was missing or only appeared temporarily during manual
  lifecycle experiments.

The `lifecycle_manager_navigation` service node is not itself a lifecycle node,
so this is not a root cause:

```bash
ros2 lifecycle get /lifecycle_manager_navigation
```

`Node not found` for that command is expected. The relevant checks are its
services:

```text
/lifecycle_manager_navigation/is_active
/lifecycle_manager_navigation/manage_nodes
```

## 2. Files Changed

- `Bumper-Bot-main/bumperbot_navigation/config/behavior_server.yaml`
- `Bumper-Bot-main/bumperbot_navigation/config/planner_server.yaml`
- `Bumper-Bot-main/bumperbot_navigation/launch/navigation.launch.py`
- `Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh`

No `bumperbot_active_slam` node logic was changed.

## 3. Plugin Type Changes

Behavior plugins:

```yaml
nav2_behaviors::Spin -> nav2_behaviors/Spin
nav2_behaviors::BackUp -> nav2_behaviors/BackUp
nav2_behaviors::Wait -> nav2_behaviors/Wait
```

Planner plugin:

```yaml
nav2_smac_planner::SmacPlanner2D -> nav2_smac_planner/SmacPlanner2D
```

Other Nav2 plugin IDs were audited against `/opt/ros/humble/share` plugin XML.
The existing controller, smoother, and costmap plugin IDs match the local Humble
plugin descriptions and were left unchanged.

## 4. Why `behavior_server` Failed

The log showed:

```text
Failed to create behavior spin of type nav2_behaviors::Spin
Declared types are nav2_behaviors/AssistedTeleop nav2_behaviors/BackUp nav2_behaviors/DriveOnHeading nav2_behaviors/Spin nav2_behaviors/Wait
```

That is a direct pluginlib class ID mismatch. The config now uses the declared
slash-form IDs.

## 5. Why `bt_navigator` Did Not Expose `/navigate_to_pose`

`bt_navigator` depends on Nav2 action servers created by the planner,
controller, smoother, and behavior servers. The planner log showed:

```text
Failed to create global planner ... nav2_smac_planner::SmacPlanner2D ...
Declared types are ... nav2_smac_planner/SmacPlanner2D ...
```

Since `planner_server` died or stayed unconfigured, the BT navigator could not
find:

```text
compute_path_through_poses
```

and failed loading:

```text
navigate_through_poses_w_replanning_and_recovery.xml
```

When `bt_navigator` fails configure/activation, `/navigate_to_pose` is not
reliably exposed as an active action server.

## 6. Lifecycle Launch Audit

`bumperbot_navigation/launch/navigation.launch.py` already had:

```text
autostart: True
```

and correct managed node names:

```text
controller_server
planner_server
smoother_server
bt_navigator
behavior_server
```

The lifecycle order was adjusted so `behavior_server` is managed before
`bt_navigator`:

```text
controller_server
planner_server
smoother_server
behavior_server
bt_navigator
```

This lets required servers configure before BT navigator loads its behavior
tree.

## 7. Planner/Costmap Audit

`planner_server.yaml` contains one `global_costmap` subtree under the planner
server config, which is the normal Nav2 pattern for the planner server's costmap
ROS node:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      ...
```

No duplicate `global_costmap` launch action was found in
`bumperbot_navigation/launch/navigation.launch.py`.

The previous message:

```text
Node '/global_costmap/global_costmap' has already been added to an executor
```

is consistent with dirty/manual lifecycle retries or duplicate/stale Nav2 graph
state. Retest from a clean graph is required after this plugin fix.

## 8. Debug Script

Added:

```text
Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh
```

It reports OK/WARN/FAIL for:

- duplicate node names
- lifecycle manager services
- lifecycle manager `is_active`
- lifecycle states of `bt_navigator`, `planner_server`, `controller_server`,
  `behavior_server`, and `smoother_server`
- `/navigate_to_pose` action server count
- recent Nav2 error lines from the newest ROS log directory or recent top-level
  node logs

Run:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
bash Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh
```

## 9. Correct Launch Command

Stop old ROS/Gazebo launch terminals first. Then run a clean bringup:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

`simulated_robot.launch.py` already includes:

```text
bumperbot_navigation/launch/navigation.launch.py
```

Do not launch `bumperbot_navigation navigation.launch.py` a second time when
using the bringup launch above.

## 10. Expected Output

Duplicate node check:

```bash
ros2 node list | sort | uniq -d
```

Expected: no duplicate Nav2 node names.

Action:

```bash
ros2 action info /navigate_to_pose
```

Expected:

```text
Action servers: 1
    /bt_navigator
```

Lifecycle:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /smoother_server
```

Expected:

```text
active [3]
active [3]
active [3]
active [3]
active [3]
```

Debug script expected summary:

```text
OK: no duplicate node names detected
OK: /lifecycle_manager_navigation/is_active exists
OK: /lifecycle_manager_navigation/manage_nodes exists
OK: /bt_navigator active [3]
OK: /planner_server active [3]
OK: /controller_server active [3]
OK: /behavior_server active [3]
OK: /smoother_server active [3]
OK: /navigate_to_pose has 1 action server(s)
```

## 11. Build Verification

Command:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_navigation bumperbot_bringup bumperbot_active_slam
```

Result:

```text
Summary: 3 packages finished
```

Script syntax and Python syntax:

```bash
bash -n Bumper-Bot-main/bumperbot_active_slam/scripts/nav2_lifecycle_check.sh
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Result: passed.

## 12. Scope Not Changed

- No Active SLAM navigation logic changed.
- No costmap rejection added.
- No path entropy added.
- No roadmap logic added.
- No reference repo copied or modified.
