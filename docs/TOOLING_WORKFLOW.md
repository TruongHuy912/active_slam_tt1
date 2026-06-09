# Tooling Workflow

This document describes non-runtime tooling for understanding, reviewing, and packaging the Active SLAM workspace.

## Installed Or Checked Tools

### CodeGraph

Purpose: symbol graph, call graph, and semantic code navigation for AI coding agents.

Official README checked from:
`https://github.com/colbymchenry/codegraph`

Install method used:

```bash
npm install -g @colbymchenry/codegraph
```

Useful commands:

```bash
codegraph --help
codegraph status .
codegraph init -i .
codegraph query ActiveSlamExplorer
```

This workflow does not auto-run `codegraph install` because that modifies agent configuration. Run it only when you intentionally want to wire CodeGraph into an agent.

### Repomix

Purpose: pack the workspace into an AI-friendly context file.

Official install docs checked from:
`https://repomix.com/guide/installation`

Install method used:

```bash
npm install -g repomix
```

Use:

```bash
tools/run_repomix_snapshot.sh
```

The workspace `.repomixignore` excludes build artifacts, logs, binary media, mesh assets, and large map/runtime outputs.

### Open Code Review

Purpose: review Git diffs after code changes.

Official README checked from:
`https://github.com/alibaba/open-code-review`

Install method used:

```bash
npm install -g @alibaba-group/open-code-review
```

The installed command is `ocr`, not `open-code-review`.

Open Code Review requires an LLM endpoint/token for real reviews. Do not hard-code secrets. Use environment variables or `ocr config set` locally.

Safe preview:

```bash
tools/run_code_review_check.sh
```

### Harness

Purpose: Claude Code agent team architecture and role generation.

Official README checked from:
`https://github.com/revfactory/harness`

Status: docs only. Harness is a Claude Code plugin/skill and is not required for ROS runtime integration. Do not install it into this ROS workspace unless project-scoped agent assets are explicitly requested.

### Graphify

Purpose: optional knowledge graph over code, docs, logs, and reports.

Official README checked from:
`https://github.com/safishamsi/graphify`

Status: optional/docs only for this workspace. Graphify has a wider Python tooling surface and optional LLM/API dependencies. Use an isolated tool install such as `uv tool install graphifyy` or `pipx install graphifyy`; avoid plain pip installs into the ROS environment.

### Understand-Anything

Purpose:
- Builds an interactive knowledge graph for the Active SLAM codebase.
- Useful for understanding file/function/class/dependency relationships before editing runtime code.
- Recommended scope for this workspace: `Bumper-Bot-main/bumperbot_active_slam`.

Status: installed for Codex at user level.

Official README checked from:
`https://github.com/Egonex-AI/Understand-Anything`

Repository note:
- The requested repository currently resolves in README/package metadata to `Lum1104/Understand-Anything`.
- The cloned checkout was placed outside this ROS workspace at `~/Downloads/ai_code_tools/Understand-Anything`.

Install method used:

```bash
mkdir -p ~/Downloads/ai_code_tools/understand_anything_install_check
cd ~/Downloads/ai_code_tools/understand_anything_install_check
curl -fsSL https://raw.githubusercontent.com/Lum1104/Understand-Anything/main/install.sh -o install_understand_anything.sh
chmod +x install_understand_anything.sh
sed -n '1,240p' install_understand_anything.sh
bash install_understand_anything.sh codex
```

Install locations:
- `~/.understand-anything/repo`
- `~/.understand-anything-plugin`
- `~/.agents/skills/understand`
- `~/.agents/skills/understand-chat`
- `~/.agents/skills/understand-dashboard`
- `~/.agents/skills/understand-diff`
- `~/.agents/skills/understand-domain`
- `~/.agents/skills/understand-explain`
- `~/.agents/skills/understand-knowledge`
- `~/.agents/skills/understand-onboard`

Restart Codex/CLI before using the slash commands in a new session.

Install notes from README:
- Claude Code marketplace:

```bash
/plugin marketplace add Lum1104/Understand-Anything
/plugin install understand-anything
```

- Codex and other CLI/IDE platforms:

```bash
curl -fsSL https://raw.githubusercontent.com/Lum1104/Understand-Anything/main/install.sh | bash -s codex
```

The installer clones to `~/.understand-anything/repo`, links a universal plugin root at `~/.understand-anything-plugin`, and links skills under `~/.agents/skills` for Codex. Restart the CLI/IDE afterward.

Safety:
- Do not scan `build/`, `install/`, `log/`, large map/media/mesh assets by default.
- Do not create project hooks or auto-update configs without explicit approval.
- Do not commit generated graph/cache files until reviewed.
- Do not enable `/understand --auto-update` unless project hooks are explicitly approved.

Suggested usage:

```bash
# Prefer scanning only the Active SLAM package:
/understand Bumper-Bot-main/bumperbot_active_slam

# Dashboard and chat after a graph exists:
/understand-dashboard
/understand-chat Explain the runtime flow from frontier detection to Nav2 planner validation and navigation dispatch.
```

Generated files:
- Main graph/config output is under `.understand-anything/`.
- README says team-shared graph files can be committed after review.
- README says local scratch should be ignored:

```gitignore
.understand-anything/intermediate/
.understand-anything/diff-overlay.json
```

## Recommended Flow

1. Run a read-only snapshot:

```bash
tools/active_slam_repo_snapshot.sh
```

2. Pack context for ChatGPT/Codex:

```bash
tools/run_repomix_snapshot.sh
```

3. Before changing runtime code, inspect structure:

```bash
tools/run_codegraph_index.sh
```

4. After any code/config change, review diff:

```bash
tools/run_code_review_check.sh
```

5. Apply the checklist in `docs/CODE_REVIEW_CHECKLIST.md`.

## Do Not

- Do not change `enable_navigation`, `scoring_mode`, or `enable_efficient_utility` as part of tooling setup.
- Do not add API keys to scripts, docs, YAML, shell history snippets, or repo files.
- Do not pack `build/`, `install/`, `log/`, Gazebo meshes, image assets, bags, or database files unless explicitly needed.
