# Active SLAM New Baseline Audit

Scope: audit only. No code, launch, config, or dependency changes were made.

## Current Bumper-Bot baseline

Project root audited: `/home/hlq017912/Downloads/bumper_bot_active_slam_new`.

Existing ROS 2 packages under `Bumper-Bot-main/`:

- `bumperbot_bringup`
- `bumperbot_controller`
- `bumperbot_cpp_examples`
- `bumperbot_description`
- `bumperbot_firmware`
- `bumperbot_localization`
- `bumperbot_mapping`
- `bumperbot_motion`
- `bumperbot_msgs`
- `bumperbot_navigation`
- `bumperbot_planning`
- `bumperbot_py_examples`
- `bumperbot_utils`

Launch files related to simulation, Gazebo, SLAM, Nav2, and RViz:

- `Bumper-Bot-main/bumperbot_bringup/launch/simulated_robot.launch.py`
  - Main simulated robot launch.
  - Includes Gazebo, controller, joystick teleop, either localization or SLAM, Nav2, and RViz.
  - Has `use_slam` launch argument, default `false`.
- `Bumper-Bot-main/bumperbot_description/launch/gazebo.launch.py`
  - Starts `ros_gz_sim` using `gz_sim.launch.py`.
  - Spawns robot from `robot_description`.
  - Bridges `/clock`, `/imu`, `/scan` from Gazebo to ROS 2.
  - Supports `world_name`, default `empty`.
- `Bumper-Bot-main/bumperbot_mapping/launch/slam.launch.py`
  - Starts `slam_toolbox/sync_slam_toolbox_node`.
  - Starts `nav2_map_server/map_saver_server`.
  - Starts lifecycle manager for SLAM/map saver.
- `Bumper-Bot-main/bumperbot_navigation/launch/navigation.launch.py`
  - Starts Nav2 `controller_server`, `planner_server`, `smoother_server`, `bt_navigator`, `behavior_server`.
  - Starts lifecycle manager for those Nav2 nodes.
- `Bumper-Bot-main/bumperbot_localization/launch/global_localization.launch.py`
  - Starts `map_server` and `amcl` when `use_slam:=false`.
- RViz configs:
  - `Bumper-Bot-main/bumperbot_mapping/rviz/slam.rviz`
  - `Bumper-Bot-main/bumperbot_description/rviz/display.rviz`
  - `Bumper-Bot-main/bumperbot_localization/rviz/global_localization.rviz`
  - Main simulated launch uses `nav2_bringup/rviz/nav2_default_view.rviz`.

Current simulation run path:

- Mapping/SLAM baseline:
  - `ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true`
- Localization-on-saved-map baseline:
  - `ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=false`
- Optional Gazebo world argument is handled by `bumperbot_description/launch/gazebo.launch.py` as `world_name`.
- Available worlds:
  - `empty.world`
  - `small_house.world`
  - `small_warehouse.world`

Important topics and frames:

- `/scan`
  - Gazebo bridge publishes ROS 2 `sensor_msgs/msg/LaserScan`.
  - SLAM Toolbox uses `scan_topic: /scan`.
  - Nav2 local/global obstacle layers use `/scan`.
- `/map`
  - Published by SLAM Toolbox during mapping.
  - Used by Nav2 static layers as `map_topic: /map`.
  - Used by AMCL/map server when not in SLAM mode.
- `/tf`, `/tf_static`
  - Robot state publisher provides static/dynamic robot frames.
  - Controller/SLAM/AMCL provide odom/map transforms depending on mode.
- `/odom`
  - Frame name used by SLAM Toolbox and Nav2 local costmap.
  - Nav2 config uses odometry topic `/bumperbot_controller/odom`.
- Frame names:
  - `map`: global frame for SLAM, AMCL, global costmap, BT navigator.
  - `odom`: local continuous frame for controller/local costmap.
  - `base_link`: Nav2 `robot_base_frame` in costmaps and BT navigator.
  - `base_footprint`: SLAM Toolbox `base_frame`, AMCL `base_frame_id`, and diff drive controller `base_frame_id`.

Nav2 action/server/config baseline:

- Action expected by exploration: `/navigate_to_pose` using `nav2_msgs/action/NavigateToPose`.
- `bt_navigator`
  - Config: `Bumper-Bot-main/bumperbot_navigation/config/bt_navigator.yaml`
  - `global_frame: map`
  - `robot_base_frame: base_link`
  - `odom_topic: /bumperbot_controller/odom`
- `planner_server`
  - Config: `Bumper-Bot-main/bumperbot_navigation/config/planner_server.yaml`
  - Plugin: `nav2_smac_planner::SmacPlanner2D`
  - `allow_unknown: true`
  - `global_costmap` nested in same file.
- `controller_server`
  - Config: `Bumper-Bot-main/bumperbot_navigation/config/controller_server.yaml`
  - Plugin: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
  - `odom_topic: /bumperbot_controller/odom`
  - `local_costmap` nested in same file.
- `global_costmap`
  - `global_frame: map`
  - `robot_base_frame: base_link`
  - `track_unknown_space: true`
  - Static layer subscribes `/map`.
  - Obstacle layer subscribes `/scan`.
- `local_costmap`
  - `global_frame: odom`
  - `robot_base_frame: base_link`
  - Rolling window `3 x 3 m`.
  - Obstacle layer subscribes `/scan`.

Packages likely to remain unchanged for the new package:

- `bumperbot_description`: keep robot model, Gazebo worlds, bridge setup.
- `bumperbot_controller`: keep controller and odometry baseline.
- `bumperbot_mapping`: keep existing SLAM Toolbox launch/config.
- `bumperbot_navigation`: keep existing Nav2 config and action servers.
- `bumperbot_bringup`: keep existing baseline launch.
- `bumperbot_localization`: keep AMCL/map-server path for non-SLAM mode.

Package to create later:

- `bumperbot_active_slam` at `/home/hlq017912/Downloads/bumper_bot_active_slam_new/bumperbot_active_slam`.

Packages not relevant to first Active SLAM baseline unless later needed:

- `bumperbot_cpp_examples`
- `bumperbot_py_examples`
- `bumperbot_firmware`
- `bumperbot_msgs`
- `bumperbot_utils`
- Custom planner/controller packages `bumperbot_planning`, `bumperbot_motion` should not be modified for the first exploration package because Nav2 already uses standard Smac 2D and Regulated Pure Pursuit.

## m-explore-ros2 useful parts

Reference root audited: `/home/hlq017912/Downloads/research_refs/m-explore-ros2`.

Main frontier exploration package:

- Package directory: `explore/`
- Package name: `explore_lite`
- Main node: `explore/src/explore.cpp`
- Frontier search: `explore/src/frontier_search.cpp`
- Costmap/map wrapper: `explore/src/costmap_client.cpp`

How it subscribes map:

- `Costmap2DClient` declares:
  - `costmap_topic`, default `costmap`
  - `costmap_updates_topic`, default `costmap_updates`
  - `robot_base_frame`, default `base_link`
  - `transform_tolerance`, default `0.3`
- `explore/config/params.yaml` uses:
  - `costmap_topic: map`
  - `costmap_updates_topic: map_updates`
- `explore/config/params_costmap.yaml` can use Nav2 costmap topics:
  - `/global_costmap/costmap`
  - `/global_costmap/costmap_updates`
- Full map messages are `nav_msgs/msg/OccupancyGrid`.
- Partial map updates are `map_msgs/msg/OccupancyGridUpdate`.
- The node converts occupancy values into `nav2_costmap_2d::Costmap2D` cost values:
  - `0 -> FREE`
  - `100 -> LETHAL`
  - `-1 -> NO_INFORMATION`

How it detects frontiers:

- Starts BFS from robot pose transformed into the map/costmap global frame.
- Finds nearest `FREE_SPACE` cell if the robot cell is not directly free.
- Expands through free/known traversable cells with 4-connected neighbors.
- A frontier cell is:
  - `NO_INFORMATION`
  - not already flagged
  - adjacent to at least one `FREE_SPACE` cell in 4-connected neighborhood.
- Builds each frontier cluster using 8-connected BFS over frontier cells.
- Stores:
  - all frontier points
  - centroid
  - initial contact point
  - middle/closest point
  - size
  - minimum distance from robot/reference.
- Filters by `min_frontier_size`.

How it chooses frontier goal:

- Computes cost:
  - `potential_scale * min_distance * resolution - gain_scale * size * resolution`
- Sorts frontiers by ascending cost.
- Picks first non-blacklisted frontier.
- Sends the frontier centroid as goal position.
- Orientation is simple identity yaw (`orientation.w = 1.0`).

How it sends goal to Nav2:

- Uses `rclcpp_action::Client<nav2_msgs::action::NavigateToPose>`.
- Action name resolves to `navigate_to_pose` for newer ROS 2 versions.
- Sends `NavigateToPose::Goal` with:
  - `pose.header.frame_id = costmap global frame`
  - `pose.pose.position = selected frontier centroid`
  - `pose.pose.orientation.w = 1.0`
- Waits for action server before starting exploration.

How it avoids repeated goals / handles failures:

- Planner timer runs at `planner_frequency`, default around `0.15 Hz`.
- Tracks `prev_goal_` and returns early if selected goal is effectively the same.
- Tracks `prev_distance_` and `last_progress_`.
- If no progress for `progress_timeout`, it blacklists the current target.
- If Nav2 result is `ABORTED`, it blacklists that frontier.
- Blacklist match tolerance is `5 * map_resolution` in x/y.
- Supports pause/resume via `explore/resume` (`std_msgs/msg/Bool`).
- Publishes exploration status via `explore/status`.

Marker/debug topics:

- Frontier markers: `explore/frontiers` when `visualize: true`.
- Status: `explore/status`, transient local QoS.
- Resume/stop command: `explore/resume`.
- Marker semantics:
  - blue frontier points for usable frontiers
  - red frontier points for blacklisted frontiers
  - green sphere at frontier centroid

Useful parts for Bumper-Bot:

- Minimal ROS 2 Humble-compatible pattern: `OccupancyGrid` + TF + `NavigateToPose`.
- BFS frontier detection over a 2D occupancy/cost map.
- Goal de-duplication.
- Progress timeout and blacklist.
- RViz marker publication.

Parts to adapt carefully:

- It depends on `map_msgs` for partial updates; the new package should avoid adding that dependency in the first step unless already available and needed.
- It uses `nav2_costmap_2d::Costmap2D`; a cleaner first Bumper-Bot package can operate directly on `nav_msgs/msg/OccupancyGrid` to avoid extra dependency surface.
- Its default `robot_base_frame: base_link` differs from SLAM Toolbox `base_footprint`; Bumper-Bot Nav2 uses `base_link`, so the new package should expose this as a parameter.

## roadmap-explorer useful ideas

Reference root audited: `/home/hlq017912/Downloads/research_refs/roadmap-explorer`.

Main package:

- `roadmap_explorer`
- Executable: `roadmap_exploration_server`
- Main launch: `roadmap_explorer/launch/exploration_server.launch.py`
- TurtleBot example launch: `roadmap_explorer/launch/tb3_exploration.launch.py`

Useful ideas, not direct code to port:

- Frontier candidate model:
  - Represents each frontier as a stateful object with UID, size, goal point, orientation, information gain, path length, achievability, blacklist flag, and weighted cost.
  - For Bumper-Bot, use a simpler struct/dataclass with `id`, `centroid`, `middle`, `size`, `distance`, `information_gain`, `score`, `blacklisted_until`.
- Viewpoint scheduling:
  - `CountBasedGain` raytraces from candidate frontier poses and chooses the orientation with the most expected unknown cells in field of view.
  - For 2D LiDAR Bumper-Bot, adapt the idea as 2D unknown-cell counting around a candidate, not camera FOV modeling.
  - A simple first version can use yaw toward frontier normal or toward unknown-space centroid.
- Frontier ranking:
  - Combines reachability/path length and information gain.
  - Marks frontiers unachievable when outside boundary, blacklisted, near lethal cells, or path planning fails.
  - For Bumper-Bot, rank by `info_gain - distance_weight * Nav2/Euclidean distance - obstacle_penalty`.
- Global planning / avoiding greedy behavior:
  - Maintains a frontier roadmap graph and uses local/global separation.
  - `FullPathOptimizer` considers a small set of local frontiers plus a closest global frontier instead of blindly taking the immediate best centroid.
  - Has goal hysteresis: keep current committed goal unless the new goal is sufficiently better.
  - For Bumper-Bot, start with simple hysteresis and goal commitment before considering a roadmap.
- State/session management:
  - Uses blackboard state in BT.
  - Tracks current committed goal.
  - Tracks blacklisted frontiers.
  - Has JSON save/load for roadmap spatial hash map.
  - For Bumper-Bot, keep in-memory state only for the first baseline.
- Debug visualization:
  - Publishes frontiers, all-frontiers, local search area, full path, global repositioning path, roadmap markers.
  - For Bumper-Bot, use RViz markers for frontier clusters, chosen goal, blacklist, and optional candidate scores.

Parts too heavy or not appropriate for 2D LiDAR Bumper-Bot baseline:

- BehaviorTree.CPP/pluginlib architecture for exploration.
- Custom lifecycle exploration server.
- Separate exploration costmap.
- Sensor simulator and `/explored_map` pipeline.
- Camera-specific parameters like `max_camera_depth`, `camera_fov`, and camera ray scheduling.
- Full frontier roadmap graph with spatial hash persistence as a first implementation.
- Theta*/NavFn internal planning plugins duplicated inside the exploration package.
- JSON session persistence for first baseline.
- PCL dependency.
- Custom messages in `roadmap_explorer_msgs`.

## What NOT to port

- Do not copy `m-explore-ros2/explore` into the project.
- Do not copy `m-explore-ros2/explore_lite_msgs` into the project.
- Do not copy `roadmap-explorer/roadmap_explorer*` packages into the project.
- Do not port ROS1-era concepts such as `rospy`, `move_base`, or `actionlib`.
- Do not add a new Nav2 stack, costmap server, planner plugin, or controller plugin for the first baseline.
- Do not modify the existing Bumper-Bot SLAM Toolbox config for the first baseline.
- Do not modify the existing Bumper-Bot Nav2 configs for the first baseline.
- Do not add new dependencies until the new package is created and dependency availability is checked.
- Do not depend on camera/viewpoint logic as if Bumper-Bot has a depth camera; the baseline target is 2D LiDAR.

## Proposed new package structure

Create a new ROS 2 Python package for fast iteration and minimal dependency surface:

```text
bumperbot_active_slam/
  package.xml
  setup.py
  setup.cfg
  resource/
    bumperbot_active_slam
  bumperbot_active_slam/
    __init__.py
    active_slam_node.py
    frontier_detector.py
    frontier_ranker.py
    nav2_client.py
    map_utils.py
    markers.py
  launch/
    active_slam.launch.py
  config/
    active_slam.yaml
  rviz/
    active_slam.rviz
  test/
    test_frontier_detector.py
    test_frontier_ranker.py
```

First baseline dependencies should stay within common ROS 2/Nav2 interfaces:

- `rclpy`
- `nav_msgs`
- `geometry_msgs`
- `sensor_msgs` only if scan-aware validation is added later
- `visualization_msgs`
- `nav2_msgs`
- `tf2_ros`
- `tf_transformations` only if already available/needed; otherwise avoid
- `std_msgs`

## Step-by-step implementation plan

1. Create `bumperbot_active_slam` package skeleton only.
2. Add launch file that starts only the active exploration node and reads config; it should assume Bumper-Bot simulation, SLAM Toolbox, and Nav2 are already launched.
3. Add config defaults:
   - `map_topic: /map`
   - `scan_topic: /scan`
   - `robot_base_frame: base_link`
   - `global_frame: map`
   - `nav_action: navigate_to_pose`
   - frontier min cluster size, goal distance thresholds, timeout, blacklist radius.
4. Implement `OccupancyGrid` map cache.
5. Implement TF lookup from `map` to `base_link`.
6. Implement BFS frontier detector directly on occupancy values:
   - free: `0`
   - occupied: `>= occupied_threshold`
   - unknown: `-1`
7. Cluster frontier cells with 8-connected search.
8. Generate candidate goal as centroid or nearest/middle frontier point adjusted to adjacent free cell.
9. Rank frontiers by size/information gain and distance.
10. Send `NavigateToPose` goal to Nav2.
11. Add goal commitment:
    - do not resend same goal repeatedly
    - keep current goal unless new score is significantly better
12. Add progress timeout and blacklist.
13. Publish RViz markers:
    - all frontier clusters
    - selected goal
    - blacklisted goals
14. Add unit tests for frontier detection/ranking using synthetic occupancy grids.
15. Validate manually with:
    - `ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true`
    - then `ros2 launch bumperbot_active_slam active_slam.launch.py`

## Risks and assumptions

- Frame mismatch risk:
  - SLAM Toolbox uses `base_footprint`.
  - Nav2 uses `base_link`.
  - The active node should default to `base_link` for Nav2 consistency but allow parameter override.
- `/map` QoS risk:
  - SLAM/map topics often use transient local QoS. The active node should subscribe with compatible QoS.
- Goal validity risk:
  - Frontier centroid may lie in unknown space. Candidate goal should be moved to a nearby free cell before sending to Nav2.
- Nav2 reachability risk:
  - Smac Planner has `allow_unknown: true`; this helps exploration but can also plan through unknown space. The active node should still prefer free-space-adjacent goals.
- Replanning churn risk:
  - Without hysteresis, the robot may switch goals every map update. Add committed-goal logic early.
- Blacklist persistence risk:
  - In-memory blacklist is enough for baseline but resets on node restart.
- Performance risk:
  - Full-map BFS every timer tick is fine for small maps but should be throttled for larger maps.
- Assumption:
  - Existing simulation publishes `/scan`, `/map`, `/tf`, `/tf_static`, and `/bumperbot_controller/odom` when launched with `use_slam:=true`.
- Assumption:
  - Existing Nav2 lifecycle nodes become active through `bumperbot_navigation/launch/navigation.launch.py`.

## Exact files that should be created later

Under `/home/hlq017912/Downloads/bumper_bot_active_slam_new/bumperbot_active_slam/`:

- `package.xml`
- `setup.py`
- `setup.cfg`
- `resource/bumperbot_active_slam`
- `bumperbot_active_slam/__init__.py`
- `bumperbot_active_slam/active_slam_node.py`
- `bumperbot_active_slam/frontier_detector.py`
- `bumperbot_active_slam/frontier_ranker.py`
- `bumperbot_active_slam/nav2_client.py`
- `bumperbot_active_slam/map_utils.py`
- `bumperbot_active_slam/markers.py`
- `launch/active_slam.launch.py`
- `config/active_slam.yaml`
- `rviz/active_slam.rviz`
- `test/test_frontier_detector.py`
- `test/test_frontier_ranker.py`

Optional later, after baseline works:

- `bumperbot_active_slam/viewpoint_selector.py`
- `bumperbot_active_slam/session_state.py`
- `test/test_viewpoint_selector.py`

## Exact files that should not be touched

Do not modify these existing Bumper-Bot baseline files for the first Active SLAM package:

- `Bumper-Bot-main/bumperbot_bringup/launch/simulated_robot.launch.py`
- `Bumper-Bot-main/bumperbot_bringup/launch/real_robot.launch.py`
- `Bumper-Bot-main/bumperbot_description/launch/gazebo.launch.py`
- `Bumper-Bot-main/bumperbot_description/urdf/bumperbot.urdf.xacro`
- `Bumper-Bot-main/bumperbot_description/urdf/bumperbot_gazebo.xacro`
- `Bumper-Bot-main/bumperbot_mapping/launch/slam.launch.py`
- `Bumper-Bot-main/bumperbot_mapping/config/slam_toolbox.yaml`
- `Bumper-Bot-main/bumperbot_navigation/launch/navigation.launch.py`
- `Bumper-Bot-main/bumperbot_navigation/config/bt_navigator.yaml`
- `Bumper-Bot-main/bumperbot_navigation/config/controller_server.yaml`
- `Bumper-Bot-main/bumperbot_navigation/config/planner_server.yaml`
- `Bumper-Bot-main/bumperbot_navigation/config/costmap.yaml`
- `Bumper-Bot-main/bumperbot_navigation/config/behavior_server.yaml`
- `Bumper-Bot-main/bumperbot_navigation/config/smoother_server.yaml`
- `Bumper-Bot-main/bumperbot_localization/launch/global_localization.launch.py`
- `Bumper-Bot-main/bumperbot_localization/config/amcl.yaml`
- Any file under `/home/hlq017912/Downloads/research_refs/m-explore-ros2`
- Any file under `/home/hlq017912/Downloads/research_refs/roadmap-explorer`

