#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# October 29 PIN floor plus A150 geometric/track rules. The unpublished
# strengthened congestion formula remains an explicit existing-score proxy.
exec bash "$PROJECT_ROOT/scripts/run_tsv_resource_v7.sh" \
    --rules-profile a150-20261029 --tsv-obstacle-scope all-layers \
    --tsv-track-demand 1 --v9-subtree-recovery 0 --low-layer-mask 0 \
    --rrr-reserved-seconds 0 "$@"
