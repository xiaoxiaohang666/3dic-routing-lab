#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Uniform preset; callers may override settings with later CLI arguments.
exec bash "$PROJECT_ROOT/scripts/run_tsv_resource_v6.sh" \
    --tsv-index-reuse 0 --checkpoint-reuse 0 --checkpoint-amortization 0 "$@"
