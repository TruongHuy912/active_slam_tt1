# Humble Nav2 Planner API Fix Report

## Scope

Fixed ROS 2 Humble/Nav2 Humble `nav2_core::GlobalPlanner` API compatibility in `bumperbot_planning` only.

No files in `bumperbot_active_slam` were modified.
No launch/config files were modified.
No dependencies were added.

## Files Changed

- `Bumper-Bot-main/bumperbot_planning/include/bumperbot_planning/a_star_planner.hpp`
- `Bumper-Bot-main/bumperbot_planning/include/bumperbot_planning/dijkstra_planner.hpp`
- `Bumper-Bot-main/bumperbot_planning/src/a_star_planner.cpp`
- `Bumper-Bot-main/bumperbot_planning/src/dijkstra_planner.cpp`

## Change

Updated both custom planner plugins from the newer API:

```cpp
createPlan(start, goal, cancel_checker)
```

to the Nav2 Humble API:

```cpp
nav_msgs::msg::Path createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
```

The removed `cancel_checker` parameter was not used in either planner body, so the A* and Dijkstra planning logic was left unchanged.

## Pluginlib

Plugin exports were checked and left unchanged:

- `PLUGINLIB_EXPORT_CLASS(bumperbot_planning::AStarPlanner, nav2_core::GlobalPlanner)`
- `PLUGINLIB_EXPORT_CLASS(bumperbot_planning::DijkstraPlanner, nav2_core::GlobalPlanner)`
- `global_planner_plugins.xml` still exports both planners as `nav2_core::GlobalPlanner`.

## Verification

Command run from workspace root:

```bash
cd /home/hlq017912/Downloads/bumper_bot_active_slam_new
rm -rf build install log
colcon build --symlink-install
```

Result:

```text
Summary: 14 packages finished
```

No remaining build errors were found.

