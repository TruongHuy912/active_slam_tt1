# Phase Nav2 Goal Dispatch Report

## Files Changed

- `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
  - Added optional `nav2_msgs/action/NavigateToPose` dispatch through `rclpy.action.ActionClient`.
  - Added a minimal navigation state machine.
  - Added goal timeout, cancel, retry, and blacklist handling.
  - Added `selected_goal` and `blacklist` RViz marker namespaces.
- `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
  - Added navigation safety parameters.
- `Bumper-Bot-main/bumperbot_active_slam/launch/active_slam.launch.py`
  - Added `enable_navigation` launch argument.
- `Bumper-Bot-main/bumperbot_active_slam/package.xml`
  - Added `nav2_msgs` dependency.
- `Bumper-Bot-main/bumperbot_active_slam/README.md`
  - Added safe navigation launch instructions and updated limitations.

No Bumper-Bot Nav2 planner/controller config, SLAM config, costmap config, or reference repo files were changed.

## New Parameters

```yaml
enable_navigation: false
navigate_action_name: /navigate_to_pose
goal_update_period_sec: 5.0
goal_timeout_sec: 60.0
min_goal_separation_m: 0.5
goal_reached_distance_m: 0.35
blacklist_radius_m: 0.6
blacklist_timeout_sec: 90.0
max_retries_per_frontier: 2
send_goal_on_startup: false
```

`enable_navigation` defaults to `false`, so the node remains marker-only unless explicitly enabled.

## Action Server Audit

The expected Nav2 action name is:

```text
/navigate_to_pose
```

The local audit command returned the action name, but no active server in the current shell snapshot:

```text
Action: /navigate_to_pose
Action clients: 0
Action servers: 0
```

The node handles this safely: it logs a throttled warning and does not crash or block.

## State Machine

States:

- `IDLE`: ready to consider a frontier goal.
- `WAITING_FOR_SERVER`: action server missing or a goal request has been sent and is waiting for acceptance.
- `NAVIGATING`: Nav2 accepted the goal and result is pending.
- `SUCCEEDED`: last goal completed successfully.
- `FAILED`: last goal was rejected, aborted, or returned a non-success status.
- `TIMED_OUT`: active goal exceeded `goal_timeout_sec`; cancel is requested and the frontier is blacklisted.

Goal dispatch is skipped unless all of these are true:

- `enable_navigation=true`
- map is valid and non-empty
- robot TF is available
- at least one frontier cluster exists
- no goal is pending or navigating
- the best frontier is not blacklisted
- goal separation/retry rules allow the goal
- `/navigate_to_pose` action server is ready

## How To Test With `enable_navigation=false`

Terminal 1:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

Terminal 2:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

Expected log:

```text
Navigation dispatch: enable_navigation=False, action_name=/navigate_to_pose, state=IDLE
Navigation skip: state=IDLE enable_navigation=False reason=enable_navigation=false
```

No Nav2 goal should be sent.

## How To Test With `enable_navigation=true`

Start simulation, SLAM, and Nav2 as usual. Confirm the action server exists:

```bash
ros2 action list
ros2 action info /navigate_to_pose
```

Then launch:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
source install/setup.bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

Expected good output once a valid map/frontier and Nav2 server are available:

```text
Navigation dispatch: enable_navigation=True, action_name=/navigate_to_pose, state=IDLE
Sending NavigateToPose goal: state=WAITING_FOR_SERVER frame=map x=... y=...
NavigateToPose goal accepted: state=NAVIGATING goal=(..., ...)
```

A successful result logs:

```text
NavigateToPose result: SUCCEEDED
```

Failures/timeouts add the failed centroid to retry/blacklist handling.

## How To Verify No Goal Spam

Watch the logs while `enable_navigation=true`:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py enable_navigation:=true
```

The node should not send another goal while it logs `state=NAVIGATING` or while a goal request is pending. New sends are limited by:

- `goal_update_period_sec`
- `min_goal_separation_m`
- blacklist radius/timeout
- `max_retries_per_frontier`

You can also inspect actions:

```bash
ros2 action info /navigate_to_pose
```

## How To Disable Navigation Immediately

Stop the active_slam node with `Ctrl+C`, then restart without the override:

```bash
ros2 launch bumperbot_active_slam active_slam.launch.py
```

This returns to marker-only mode because `enable_navigation` defaults to `false`.

## Verification Run

Build:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
colcon build --symlink-install --packages-select bumperbot_active_slam
```

Result:

```text
Summary: 1 package finished
```

Syntax:

```bash
python3 -m py_compile Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/*.py
```

Result: passed.

Default launch smoke test:

```bash
source install/setup.bash
timeout --signal=INT 6 ros2 launch bumperbot_active_slam active_slam.launch.py
```

Observed:

```text
Navigation dispatch: enable_navigation=False, action_name=/navigate_to_pose, state=IDLE
process has finished cleanly
```

## Known Limitations

- No costmap rejection yet.
- No safe viewpoint sampling yet.
- No path entropy scoring yet.
- Goal pose is the current best frontier centroid, not a shifted safe viewpoint.
- If Nav2 is not launched or `/navigate_to_pose` has no server, the node only logs and waits.
