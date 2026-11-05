#!/usr/bin/env bash
# Router settings of the 2026-11-05 candidate; run_current.sh defaults stay unchanged.
# Candidate entry, not the official submission entry: it needs an explicit --thermal-gain G.
# Later options override these values. See docs/rules/fixed-resource-model-20261105.md.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT/scripts/run_current.sh" \
    --fixed-resource-model observed-20261104 \
    --rrr-reserved-seconds 2000 --rrr-rounds 64 \
    --v7-rrr-net-limit 32768 --rrr-net-limit 8192 \
    --congestion-target-q 30 --future-tsv-reservation 1 \
    --time-limit 3000 "$@"
