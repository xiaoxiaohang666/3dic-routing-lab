#!/usr/bin/env bash
# Platform submission entry. The platform passes only --def/--lef/--thermal-report/
# --output, never the per-case G, so the router estimates G (1.5 / hottest-K
# Pearson bound). Everything else matches run_fixed_resource_candidate.sh; the
# time budget comes from the package's run.sh.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
    cat <<'HELP'
Usage: bash scripts/run_package.sh --def FILE --lef FILE --thermal-report FILE --output NEW.guide [router options]
Settings of run_fixed_resource_candidate.sh with --thermal-gain-estimate hottest-k
instead of a per-case G. Later options override these values, for example an explicit
--thermal-gain G or --climb-escape-radius 0. Pass --time-limit; the package uses 3600, of which the
overflow cleanup takes the last 800 s (--overflow-cleanup-seconds 0 turns it off).
HELP
    exit 0
fi
exec bash "$ROOT/scripts/run_a150_20261029.sh" \
    --via-stack-radius 0 --recovery-global-wirelength 0 \
    --thermal-gain-estimate hottest-k --fixed-resource-model observed-20261104 \
    --rrr-reserved-seconds 2000 --rrr-rounds 64 \
    --v7-rrr-net-limit 32768 --rrr-net-limit 8192 \
    --congestion-target-q 30 --future-tsv-reservation 1 --climb-escape-radius 3 \
    --overflow-cleanup-seconds 800 --overflow-cleanup-blockers 1 "$@" \
    --model official --score-profile 20261020 \
    --resource-profile documented --rules-profile a150-20261029 \
    --objective-profile site-20261029 --tsv-obstacle-scope interface \
    --tsv-resource-mode global-grid --tsv-track-demand 1
