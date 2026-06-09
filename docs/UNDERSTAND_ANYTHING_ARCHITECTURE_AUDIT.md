# Understand-Anything Architecture Audit: bumperbot_active_slam

Generated from the current Understand-Anything graph at:

`Bumper-Bot-main/bumperbot_active_slam/.understand-anything/knowledge-graph.json`

Scope: `Bumper-Bot-main/bumperbot_active_slam`

This report is documentation-only. It does not propose or apply runtime changes.

## 1. Tổng quan kiến trúc hiện tại

`bumperbot_active_slam` là ROS 2 Humble package cho Active SLAM theo baseline frontier-based safe-viewpoint. Kiến trúc hiện tại xoay quanh một node orchestration chính, `ActiveSlamExplorer`, nhận map/costmap/TF, phát hiện frontier, chọn viewpoint an toàn, xác thực candidate bằng Nav2 planner nếu được bật, rồi dispatch goal qua Nav2 `NavigateToPose` khi `enable_navigation=true`.

Các trách nhiệm runtime chính đang được tách thành module helper:

- Frontier detection: tìm frontier cell và gom cluster.
- Candidate selection: sample viewpoint, filter theo map/costmap/blacklist/recent goal, score candidate.
- Planner validation: gọi Nav2 `ComputePathToPose` và kiểm tra path safety.
- Navigation dispatch: gửi `NavigateToPose`, theo dõi state/result/timeout/blacklist.
- Recovery/diagnostics/visualization: high-cost escape, progress monitor, marker output, planner reject cache.

Graph hiện tại ghi nhận 234 nodes, 303 edges, 5 architecture layers và 5 tour steps. Các edge import cho thấy `active_slam_node.py` là hub điều phối chính, import trực tiếp các module selection, validation, navigation, marker, parameter và recovery.

## 2. Runtime flow

Luồng runtime baseline:

```text
/map + /global_costmap/costmap + TF
  -> frontier detection
  -> safe viewpoint candidate selection
  -> planner validation
  -> navigation dispatch
```

Chi tiết:

1. `ActiveSlamExplorer` subscribe `OccupancyGrid` từ `/map` và costmap từ `/global_costmap/costmap`, đồng thời lookup robot pose qua TF.
2. Timer loop kiểm tra map hợp lệ, blacklist/timeout/progress state, rồi gọi frontier detection trên latest map.
3. `frontier_detector.py` xác định frontier cell: free cell kề unknown cell, sau đó gom thành `FrontierCluster` theo connectivity.
4. `GoalSelector` nhận clusters, robot pose, map, costmap và navigation history để chọn `NavigationCandidate`.
5. Với `scoring_mode=safe_viewpoint`, selector sample viewpoint quanh frontier, loại candidate quá gần/quá xa, blacklisted, không free trên map, không an toàn theo costmap, hoặc thiếu information gain.
6. Nếu planner validation được bật, node tạo danh sách candidate đa dạng từ selected candidate và `last_valid_candidates`.
7. `PlannerValidator` gọi Nav2 `ComputePathToPose` tới `planner_action_name`, rồi dùng `path_safety.py` để kiểm tra path length, max cost, unknown cells, clearance radius và near-path cost.
8. Candidate đầu tiên có path hợp lệ được trả về node; validated path được dùng cho marker/debug.
9. `NavigationDispatcher` gửi Nav2 `NavigateToPose` tới `navigate_action_name`, theo dõi accepted/result/timeout và blacklist goal thất bại sau retry limit.

Nếu `enable_navigation=false`, node vẫn có thể detect frontier, select/debug candidates và publish markers, nhưng không dispatch Nav2 goal.

## 3. Vai trò từng module

### `active_slam_node.py`

ROS orchestration chính. Module này tạo subscription cho map/costmap, TF listener, marker publisher, `GoalSelector`, `PlannerValidator`, `NavigationDispatcher`, `HighCostEscapePolicy`, `PlannerRejectCache` và `ProgressMonitor`. Nó điều phối timer loop, skip conditions, planner-validation lifecycle, navigation dispatch, marker output và runtime logging.

### `frontier_detector.py`

Phát hiện frontier trên `OccupancyGrid`. Frontier cell được định nghĩa là free cell cạnh unknown cell. Module gom các frontier cell thành `FrontierCluster` với `cells`, `size`, `centroid_map` và `centroid_world`.

### `goal_selector.py`

Chọn safe viewpoint candidate. Module này sample frontier cells/viewpoints, lọc theo distance, blacklist, planner reject cache, goal separation, map safety, costmap clearance và information gain. Nó cũng quản lý fallback/recovery selection như global reposition, frontier bridge, bootstrap và medium reposition.

### `planner_validator.py`

Xác thực candidate trước dispatch bằng Nav2 `ComputePathToPose`. Module sort candidate theo score, thử lần lượt theo batch, timeout request nếu quá lâu, reject path không hợp lệ, optionally retry với relaxed clearance, và trả `PlannerValidationResult` khi có path an toàn.

### `navigation_dispatcher.py`

Wrapper cho Nav2 `NavigateToPose`. Module này tạo `PoseStamped` goal, set orientation hướng về target, gửi action goal, xử lý accepted/result, timeout, cancel, retry count và blacklist.

### `costmap_utils.py`

Helper cho costmap/world conversion, cost lookup, pose safety, clearance radius, costmap summary và nearest safe pose sampling. Đây là nền cho filter candidate và path safety.

### `path_safety.py`

Kiểm tra path từ Nav2 planner. Module tính path length, sample dọc path, kiểm tra costmap bounds, unknown cells, max path cost, clearance radius, near-path cost, ignore-start radius và trả `PathSafetyResult`.

### `marker_utils.py`

Tạo visualization markers cho frontier clusters, selected candidate, valid/rejected candidates, active goal, blacklist, planner rejected candidates, planner reject cache và validated path.

### `high_cost_escape.py`

Recovery policy khi robot đang ở vùng high-cost. Module chọn short escape candidate tới pose an toàn hơn gần robot, dùng cùng safety/cost/path-clearance logic nhưng với profile recovery.

### `progress_monitor.py`

Theo dõi robot progress khi đang navigate. Nếu robot không di chuyển đủ trong khoảng thời gian cấu hình, node có thể cancel active goal và blacklist tùy parameter.

### `efficient_active_slam_utility.py`

Utility scoring opt-in/experimental cho entropy-aware candidate ranking. Graph và YAML cho thấy baseline hiện tại vẫn là `safe_viewpoint`; efficient utility không phải default.

### `path_entropy.py`

Tính entropy statistics dọc Nav2 path. Module này phục vụ efficient utility/path entropy analysis khi feature đó được bật, không phải baseline dispatch bắt buộc.

## 4. Runtime-used modules vs opt-in/experimental vs offline scripts

Runtime-used baseline modules:

- `active_slam_node.py`
- `frontier_detector.py`
- `goal_selector.py`
- `navigation_dispatcher.py`
- `planner_validator.py` khi `use_planner_validation=true`
- `costmap_utils.py`
- `path_safety.py`
- `marker_utils.py`
- `models.py`
- `node_params.py`
- `planner_reject_cache.py`
- `progress_monitor.py`
- `viewpoint_sampler.py`
- `entropy_utils.py`

Runtime recovery/fallback modules:

- `high_cost_escape.py`
- global/medium/frontier-bridge selection paths inside `goal_selector.py`

Opt-in/experimental modules:

- `efficient_active_slam_utility.py`
- `path_entropy.py`
- entropy/path-entropy behavior guarded by `enable_efficient_utility` and related scoring config

Offline/diagnostic scripts:

- `scripts/active_slam_ab_summary.sh`
- `scripts/analyze_goal_pingpong.py`
- `scripts/collision_diagnostics_check.sh`
- `scripts/nav2_lifecycle_check.sh`
- `scripts/phase1_runtime_check.sh`
- `scripts/slam_map_runtime_check.sh`
- `scripts/slam_turning_diagnostics.sh`

These scripts are tooling/diagnostic helpers, not core runtime modules in the node import graph.

## 5. Các tham số YAML quan trọng

Values from `config/active_slam.yaml`:

- `enable_navigation: false`
  - Default không gửi Nav2 `NavigateToPose` goal. Navigation dispatch cần được bật rõ ràng.
- `scoring_mode: safe_viewpoint`
  - Baseline selection dùng safe-viewpoint scoring.
- `enable_efficient_utility: false`
  - Efficient entropy utility là opt-in/experimental, không phải default.
- `use_planner_validation: true`
  - Candidate được xác thực bằng Nav2 planner trước dispatch.
- `planner_action_name: /compute_path_to_pose`
  - Nav2 action dùng cho `ComputePathToPose`.
- `navigate_action_name: /navigate_to_pose`
  - Nav2 action dùng cho `NavigateToPose`.
- `safety_radius_m: 0.25`
  - Radius safety chính cho candidate/costmap checks.
- `path_clearance_radius_m: 0.22`
  - Clearance radius chính cho path safety validation.
- `high_cost_escape_enabled: true`
  - Bật recovery policy khi robot ở vùng cost cao.

Các tham số này được declare/read trong `node_params.py` và map vào config objects cho selector, planner validator và high-cost escape policy.

## 6. Rủi ro kiến trúc hiện tại

- `active_slam_node.py` đang lớn.
  - Node này vừa làm ROS wiring, orchestration, navigation state, planner validation transitions, fallback routing, marker/debug và logging. Điều này làm behavior khó review nếu tiếp tục thêm runtime logic trực tiếp vào node.
- Planner validation có thể làm delay dispatch.
  - `PlannerValidator` thử candidate theo thứ tự score, có timeout và batch/retry-next-best. Khi nhiều candidate bị reject hoặc planner chậm, dispatch sang `NavigateToPose` bị trì hoãn.
- Costmap/frame mismatch.
  - Runtime phụ thuộc map frame, costmap frame, TF robot pose và marker frame. Mismatch giữa `/map`, costmap và configured global frame có thể làm candidate/path reject hoặc marker misleading.
- Strict clearance reject nhiều goal.
  - `path_clearance_radius_m`, `max_path_cost`, `reject_path_unknown` và near-path cost checks có thể reject nhiều candidate trong môi trường hẹp. Fallback relaxed clearance giúp giảm reject nhưng cũng làm policy khó đọc hơn.
- Recovery/fallback có thể làm behavior khó đọc.
  - Local safe-viewpoint, medium reposition, global reposition, frontier bridge, high-cost escape, stale frontier suppression và planner reject cache đều có thể ảnh hưởng candidate cuối cùng. Nếu không có log baseline, rất dễ hiểu sai nguyên nhân goal được chọn hoặc bị reject.

## 7. Những điều không được giả định sai

- Không có evidence runtime RRT trong graph hiện tại.
  - Runtime flow được thể hiện là frontier detection + safe-viewpoint candidate selection + Nav2 planner validation.
- Không có BFS exploration riêng biệt.
  - Frontier clustering dùng queue/BFS-like clustering nội bộ cho frontier cells, nhưng đó không phải một exploration algorithm riêng thay thế frontier-based Active SLAM.
- Entropy utility là opt-in.
  - `enable_efficient_utility=false` trong default YAML; `scoring_mode=safe_viewpoint` là baseline.
- Default `enable_navigation=false`.
  - Mặc định node không dispatch Nav2 goals nếu không bật parameter/launch override.

## 8. Đề xuất next step

- Audit `active_slam_node.py` để tách module nếu cần.
  - Ưu tiên tách orchestration state/fallback transition/logging nếu có nhu cầu thay đổi runtime lớn.
- Test `enable_navigation=true` trong world nhỏ.
  - Chỉ làm sau khi có baseline log rõ ràng và môi trường Nav2/SLAM ổn định.
- So log candidate selection/planner validation.
  - So sánh candidate được chọn, candidate bị reject, planner validation stats, reject reason và validated path trước khi chỉnh thuật toán.
- Không sửa thuật toán khi chưa có baseline log.
  - Các thay đổi scoring, fallback, clearance hoặc recovery nên dựa trên log/marker evidence thay vì suy đoán từ graph.

