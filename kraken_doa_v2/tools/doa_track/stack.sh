#!/bin/bash
# kraken_doa_test + one fake-heimdall scene, for browser_check.py (run that
# from Windows while this runs). Uses the CURRENT kraken_doa.html of the
# checkout (the page is read per request; no rebuild needed for page edits).
#   wsl -e bash kraken_doa_v2/tools/doa_track/stack.sh [SCENE] [WARMUP_S]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SCENE="${1:-bursty30}"; WARM="${2:-25}"
R="$HERE/results"; RUN=~/kdt/run
mkdir -p "$R"
[ -x "$RUN/kraken_doa_test" ] || { echo "no test build - run wsl_setup.sh first"; exit 1; }
cp "$HERE/../../kraken_doa.html" "$RUN/" && sed -i 's/\r$//' "$RUN/kraken_doa.html"
rm -f "$RUN/doa_settings.json" "$R/truth_browser.json"
cd "$RUN"
KRAKEN_AI_MCP_PORT=28079 KRAKEN_DOA_VERBOSE_LOG=1 ./kraken_doa_test > "$R/kraken_doa_browser.log" 2>&1 < /dev/null &
KD=$!
trap 'kill $KD 2>/dev/null' EXIT
sleep 3
python3 "$HERE/fake_heimdall.py" --scene "$SCENE" --warmup "$WARM" --truth "$R/truth_browser.json" > "$R/fh_browser.log" 2>&1
sleep 2
echo "stack done"
