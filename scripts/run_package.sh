#!/usr/bin/env bash
# Platform submission entry. The platform passes only --def/--lef/--thermal-report/
# --output, never the per-case G, so the router estimates G (1.5 / hottest-K
# Pearson bound) and decides multi-net same-cell TSV hubs before routing.
# Everything else is the run_current.sh production contract.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
    cat <<'HELP'
Usage: bash scripts/run_package.sh --def FILE --lef FILE --thermal-report FILE --output NEW.guide [router options]
Production contract of run_current.sh with --thermal-gain-estimate hottest-k,
--tsv-shared-auto 1 and --climb-escape-radius 3. A later explicit --thermal-gain G
overrides the estimate; a later --climb-escape-radius 0 turns the sideways step off.
HELP
    exit 0
fi
exec bash "$ROOT/scripts/run_a150_20261029.sh" \
    --via-stack-radius 0 --recovery-global-wirelength 0 \
    --thermal-gain-estimate hottest-k --tsv-shared-auto 1 --climb-escape-radius 3 "$@" \
    --model official --score-profile 20261020 \
    --resource-profile documented --rules-profile a150-20261029 \
    --objective-profile site-20261029 --tsv-obstacle-scope interface \
    --tsv-resource-mode global-grid --tsv-track-demand 1
