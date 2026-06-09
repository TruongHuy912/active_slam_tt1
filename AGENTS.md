# Active SLAM Tooling Guardrails

This workspace is a ROS 2 Humble Active SLAM project. The current runtime baseline is frontier-based safe-viewpoint Active SLAM.

## Runtime Safety

- Do not change Active SLAM runtime logic unless the user explicitly asks for an algorithm/runtime change.
- Do not edit `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py` for tooling-only tasks.
- Do not change runtime YAML parameters such as `enable_navigation`, `scoring_mode`, `enable_efficient_utility`, planner, controller, or costmap parameters for tooling-only tasks.
- Do not install third-party tool source inside `Bumper-Bot-main/` or any ROS 2 package.
- Put external tool source under `~/Downloads/ai_code_tools/` or another directory outside this workspace.

## Default Project Context

- Main package: `bumperbot_active_slam`
- Main node: `Bumper-Bot-main/bumperbot_active_slam/bumperbot_active_slam/active_slam_node.py`
- Main launch: `Bumper-Bot-main/bumperbot_active_slam/launch/active_slam.launch.py`
- Main configs:
  - `Bumper-Bot-main/bumperbot_active_slam/config/active_slam.yaml`
  - `Bumper-Bot-main/bumperbot_active_slam/config/active_slam_small_warehouse.yaml`
- Default navigation: disabled.
- Default scoring: `safe_viewpoint`.
- Efficient entropy utility: opt-in/experimental.

## Tooling Workflow

- Use `tools/active_slam_repo_snapshot.sh` to collect a read-only project snapshot.
- Use `tools/run_repomix_snapshot.sh` to pack an AI-friendly context snapshot when Repomix is installed.
- Use `tools/run_code_review_check.sh` to preview/review diffs with Open Code Review when `ocr` is installed and an LLM is configured.
- Use `tools/run_codegraph_index.sh` to initialize/index CodeGraph deliberately.
- Use Understand-Anything to build an interactive knowledge graph for `Bumper-Bot-main/bumperbot_active_slam` before major edits.
- Understand-Anything is installed for Codex at user level under `~/.understand-anything/`, `~/.understand-anything-plugin`, and `~/.agents/skills`.
- After a Codex/CLI restart, use `/understand Bumper-Bot-main/bumperbot_active_slam` before major edits; do not scan the full workspace root by default.
- Do not use graph output to justify runtime changes without code review and tests.

Before applying any generated suggestions, review `docs/CODE_REVIEW_CHECKLIST.md`.
