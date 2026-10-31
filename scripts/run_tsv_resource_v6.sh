#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Reuse the existing packaged FLUTE libraries in this child process only.
# Resolve symlinks so a project-local runtime link finds its sibling lib folder.
OFFICIAL_RUNTIME="${EDA_OFFICIAL_RUNTIME:-$PROJECT_ROOT/output/runtime/official-flute}"
# The CLI permits repeated runtime flags; its last value wins. Keep $@ intact
# and let the executable diagnose missing or invalid option values.
waiting_official_runtime=0
for argument in "$@"; do
    if [[ "$waiting_official_runtime" == 1 ]]; then
        OFFICIAL_RUNTIME="$argument"
        waiting_official_runtime=0
    elif [[ "$argument" == --official-runtime ]]; then
        waiting_official_runtime=1
    fi
done
if [[ -d "$OFFICIAL_RUNTIME" ]]; then
    OFFICIAL_RUNTIME="$(cd -- "$OFFICIAL_RUNTIME" && pwd -P)"
    RUNTIME_LIB="$(dirname -- "$(dirname -- "$OFFICIAL_RUNTIME")")/lib"
    if [[ -f "$RUNTIME_LIB/libpython3.10.so.1.0" ]]; then
        export LD_LIBRARY_PATH="$RUNTIME_LIB:$OFFICIAL_RUNTIME${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
fi
export EDA_OFFICIAL_RUNTIME="$OFFICIAL_RUNTIME"
# Resource-v6 production candidate: correct residual-ledger landing quotes.
# Experimental operators stay explicit and can be enabled by later flags.
# Documented production model, full via charge, fixed uniform G=20.
exec bash "$PROJECT_ROOT/scripts/run_tsv_resource.sh" \
    --thermal-gain 20 --future-tsv-reservation 0 \
    --tsv-landing-pricing 1 --resource-candidate-pool 0 \
    --resource-local-rrr 0 --joint-layer-recovery 0 --thermal-coverage 0 \
    "$@"