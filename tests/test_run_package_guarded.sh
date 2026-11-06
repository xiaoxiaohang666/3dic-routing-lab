#!/usr/bin/env bash
# Tests for scripts/run_package_guarded.sh with a fake entry (no routing).
set -u
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
FAKE="$WORK/fake_entry.sh"
cat > "$FAKE" <<'EOF'
#!/usr/bin/env bash
# Behaviour by FAKE_MODE; the call number comes from the log line count.
echo "$*" >> "$FAKE_LOG"
call=$(wc -l < "$FAKE_LOG")
out=""; prev=""
for a in "$@"; do [[ "$prev" == --output ]] && out="$a"; prev="$a"; done
case "$FAKE_MODE" in
    ok) echo "guide$call" > "$out"; exit 0 ;;
    fail-once) if (( call == 1 )); then exit 1; fi; echo "guide$call" > "$out"; exit 0 ;;
    write-and-fail) echo "guide$call" > "$out"; exit 1 ;;
    never) exit 1 ;;
esac
EOF
chmod +x "$FAKE"
failures=0
check() { if eval "$2"; then echo "PASS $1"; else echo "FAIL $1"; failures=$((failures+1)); fi; }
run() {  # run MODE -> sets status, log, out
    export FAKE_MODE="$1" FAKE_LOG="$WORK/$1.log" EDA_GUARDED_ENTRY="$FAKE"
    out="$WORK/$1/case.guide"; mkdir -p "$WORK/$1"; : > "$FAKE_LOG"
    bash "$ROOT/scripts/run_package_guarded.sh" --time-limit 6300 --def d --lef l --thermal-report t --output "$out" \
        > "$WORK/$1.stdout" 2> "$WORK/$1.stderr"
    status=$?; log="$FAKE_LOG"
}
run ok
check "ok: exit 0" '[[ $status -eq 0 ]]'
check "ok: one call" '[[ $(wc -l < "$log") -eq 1 ]]'
check "ok: first Guide" '[[ "$(cat "$out")" == guide1 ]]'
check "ok: original time limit last" '[[ "$(head -1 "$log")" == *"--time-limit 6300" ]]'
check "ok: no extra files" '[[ $(ls "$WORK/ok" | wc -l) -eq 1 ]]'

run fail-once
check "fail-once: exit 0" '[[ $status -eq 0 ]]'
check "fail-once: two calls" '[[ $(wc -l < "$log") -eq 2 ]]'
check "fail-once: retry Guide" '[[ "$(cat "$out")" == guide2 ]]'
check "fail-once: retry turns maze and polish off" '[[ "$(sed -n 2p "$log")" == *"--initial-maze-expansions 0 --wirelength-polish-seconds 0 --time-limit 3600" ]]'

run write-and-fail
check "write-and-fail: exit 0 with warning" '[[ $status -eq 0 ]] && grep -q WARNING "$WORK/write-and-fail.stderr"'
check "write-and-fail: three calls" '[[ $(wc -l < "$log") -eq 3 ]]'
check "write-and-fail: keeps first written Guide" '[[ "$(cat "$out")" == guide1 ]]'
check "write-and-fail: no extra files" '[[ $(ls "$WORK/write-and-fail" | wc -l) -eq 1 ]]'
check "write-and-fail: third call turns cleanup, radius, reservation off" '[[ "$(sed -n 3p "$log")" == *"--overflow-cleanup-seconds 0 --climb-escape-radius 0 --future-tsv-reservation 0 --time-limit 3000" ]]'

run never
check "never: exit 1" '[[ $status -eq 1 ]]'
check "never: three calls" '[[ $(wc -l < "$log") -eq 3 ]]'
check "never: no Guide" '[[ ! -e "$out" ]]'

echo "failures=$failures"
exit $(( failures > 0 ))
