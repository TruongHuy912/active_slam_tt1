# Agent Roles For Active SLAM Tooling

These roles are for review and context work only. They do not grant permission to change runtime behavior.

## Repo Cartographer

Purpose: map package structure, entry points, launch files, YAML configs, topics, actions, and scripts.

Inputs:
- `tools/active_slam_repo_snapshot.sh`
- CodeGraph status/query output
- Repomix snapshot

Output:
- Architecture summary with file paths and confidence notes.

## Runtime Guardian

Purpose: protect default runtime behavior.

Checks:
- `enable_navigation`
- `scoring_mode`
- `enable_efficient_utility`
- planner/controller/costmap YAML
- `active_slam_node.py`

Output:
- Safety report listing any runtime-risk changes.

## Diff Reviewer

Purpose: review patch risk after code edits.

Tools:
- Open Code Review CLI (`ocr`)
- `docs/CODE_REVIEW_CHECKLIST.md`

Output:
- Findings ordered by severity.
- Explicit note if no runtime files changed.

## Context Packer

Purpose: prepare compact AI context.

Tools:
- Repomix
- `.repomixignore`

Output:
- AI-friendly snapshot with build/log/binary artifacts excluded.

## Optional Knowledge Graph Analyst

Purpose: use Graphify or CodeGraph for deeper code relationships.

Rules:
- Do not require API keys for code-only analysis.
- Prefer isolated installs.
- Do not create project-scoped hooks or agent configs without explicit approval.
