#!/usr/bin/env bash
# Canonical current production contract; experimental operators remain explicit.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# == 0 || "${1:-}" == --help ]]; then
    cat <<'HELP'
Usage: bash scripts/run_current.sh --thermal-gain G --def FILE --lef FILE \
  --thermal-json FILE --output NEW.guide --stats NEW.json [router options]
Run on benchmark-host. G is required; no input-name lookup is performed.
Public G: 001=11.684, 003=4.224, 005=8.323. Hidden-case G is unconfirmed.
Defaults: mature A150 preset; site-20261029 objective; experimental v13 flags off;
sideways step before a via climb at radius 3 (--climb-escape-radius 0 turns it off).
Later router options can change budgets/operators; the production contract is fixed.
TSVs are banned only on cells with PIN/OBS on M10/M11 (official Q&A and returned scores,
docs/rules/qa-notes-20261105.md). EDA_TSV_OBSTACLE_SCOPE=all-layers restores the older rule.
This is an offline research entry, not an official submission package.
HELP
    exit 0
fi
gain_found=0
for argument in "$@"; do
    if [[ "$argument" == --thermal-gain ]]; then gain_found=1; fi
done
if [[ "$gain_found" != 1 ]]; then
    echo 'An explicit --thermal-gain G is required; see docs/SCORING.md.' >&2
    exit 2
fi
scope="${EDA_TSV_OBSTACLE_SCOPE:-interface}"
if [[ "$scope" != all-layers && "$scope" != interface ]]; then
    echo "EDA_TSV_OBSTACLE_SCOPE must be all-layers or interface" >&2
    exit 2
fi
exec bash "$ROOT/scripts/run_a150_20261029.sh" \
    --via-stack-radius 0 --recovery-global-wirelength 0 "$@" \
    --model official --score-profile 20261020 \
    --resource-profile documented --rules-profile a150-20261029 \
    --objective-profile site-20261029 --tsv-obstacle-scope "$scope" \
    --tsv-resource-mode global-grid --tsv-track-demand 1
