# Active SLAM Tooling Scripts

These scripts support audit, context packaging, and review. They do not modify ROS 2 runtime logic.

## Scripts

- `active_slam_repo_snapshot.sh`: read-only workspace snapshot.
- `run_repomix_snapshot.sh`: package an AI-friendly context snapshot if `repomix` is installed.
- `run_code_review_check.sh`: preview or run Open Code Review if `ocr` is installed and configured.
- `run_codegraph_index.sh`: explicitly initialize/index CodeGraph for this workspace.
- `run_understand_anything_scan.sh`: optionally runs/scopes Understand-Anything for the Active SLAM package if installed.

## Safety

- Do not store API keys in scripts.
- Do not edit runtime YAML from these scripts.
- Do not run generated review suggestions without checking `docs/CODE_REVIEW_CHECKLIST.md`.
