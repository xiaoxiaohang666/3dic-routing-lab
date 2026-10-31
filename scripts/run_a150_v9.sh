#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# A150 conservative legal baseline. Unresolved layer scope is recorded explicitly.
# New subtree and low-layer experiments start only after baseline qualification.
exec bash "$PROJECT_ROOT/scripts/run_tsv_resource_v7.sh" \
    --rules-profile a150 --tsv-obstacle-scope all-layers \
    --tsv-track-demand 1 --v9-subtree-recovery 0 --low-layer-mask 0 \
    --rrr-reserved-seconds 0 "$@"
