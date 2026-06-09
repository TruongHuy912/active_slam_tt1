#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if ! command -v ocr >/dev/null 2>&1; then
  echo "Open Code Review CLI not found. Install with: npm install -g @alibaba-group/open-code-review"
  exit 1
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Git status could not be verified because this workspace is not a valid Git repository."
  exit 0
fi

echo "=== Open Code Review preview ==="
ocr review --preview

if [[ "${RUN_OCR_REVIEW:-0}" == "1" ]]; then
  echo "=== Open Code Review ==="
  ocr review --audience agent
else
  echo "Preview only. Set RUN_OCR_REVIEW=1 to run the LLM-backed review."
fi
