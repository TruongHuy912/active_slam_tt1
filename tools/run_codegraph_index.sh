#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if ! command -v codegraph >/dev/null 2>&1; then
  echo "codegraph not found. Install with: npm install -g @colbymchenry/codegraph"
  exit 1
fi

echo "=== CodeGraph status before indexing ==="
codegraph status . || true

echo "=== CodeGraph initialize and index ==="
codegraph init -i .

echo "=== CodeGraph status after indexing ==="
codegraph status .

echo "=== Example query ==="
codegraph query ActiveSlamExplorer || true
