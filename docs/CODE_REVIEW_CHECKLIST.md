# Code Review Checklist

Use this checklist before accepting changes suggested by Codex, Open Code Review, CodeGraph, Repomix, Graphify, or agent teams.

## Runtime Guardrails

- `active_slam_node.py` was not changed unless the task explicitly required runtime work.
- Active SLAM runtime configs were not changed for tooling-only tasks.
- `enable_navigation` remains at the intended value.
- `scoring_mode` remains at the intended value.
- `enable_efficient_utility` remains at the intended value.
- Planner/controller/costmap YAML remains unchanged unless explicitly requested.
- No external tool source was cloned into a ROS 2 package.

## Active SLAM Behavior Risks

- Frontier detection is still the main candidate source.
- `safe_viewpoint` baseline behavior is preserved.
- Planner validation remains required when navigation dispatch is enabled.
- High-cost escape, progress gate, blacklist, and planner reject cache behavior are not bypassed accidentally.
- Entropy/utility remains opt-in/experimental unless the user explicitly requested an experiment.

## Secret And Environment Safety

- No API key, token, endpoint secret, or auth header was committed.
- Open Code Review config is stored locally or via environment variables only.
- Graphify LLM backend keys are not written into repo files.

## Snapshot Hygiene

- Repomix output excludes `build/`, `install/`, `log/`, binary assets, bag files, and mesh/media files by default.
- Snapshot output is placed under `/tmp` or `tools/snapshots/`, not mixed into runtime packages.
- Generated index directories are understood before committing.

## Understand-Anything Safety

- Understand-Anything generated files/caches were reviewed before committing.
- Scan scope excludes `build/`, `install/`, `log/`, maps/media/mesh/bag/database files.
- Graph/dashboard output is used for understanding only, not as proof that runtime behavior is correct.
- Any suggestions from graph/chat are still checked against runtime code and ROS 2 tests.

## Validation

- Shell scripts pass `bash -n`.
- Tool help/version checks work where applicable.
- Git status is checked if the workspace has valid Git metadata.
- If Git metadata is invalid, report: `Git status could not be verified because this workspace is not a valid Git repository.`
