#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TARGET="Bumper-Bot-main/bumperbot_active_slam"

echo "=== Understand-Anything scan helper ==="
echo "Workspace: $ROOT"
echo "Target scope: $TARGET"

if [ ! -d "$TARGET" ]; then
  echo "Target directory not found: $TARGET"
  exit 1
fi

echo "=== Existing Understand-Anything commands ==="
command -v understand-anything || true
command -v understand || true
command -v ua || true

echo
echo "Understand-Anything is documented as an agent plugin/slash-command workflow."
echo "Official README install examples:"
echo "  /plugin marketplace add Lum1104/Understand-Anything"
echo "  /plugin install understand-anything"
echo "  curl -fsSL https://raw.githubusercontent.com/Lum1104/Understand-Anything/main/install.sh | bash -s codex"
echo
echo "If Understand-Anything is installed as a shell CLI, run the documented command against:"
echo "  $TARGET"
echo
echo "If it is installed as an agent slash command, use:"
echo "  /understand $TARGET"
echo "  /understand-dashboard"
echo "  /understand-chat Explain the runtime flow from frontier detection to Nav2 planner validation and navigation dispatch."
echo
echo "This helper does not run unknown commands automatically because Understand-Anything may be installed as a plugin/skill rather than a normal shell binary."
