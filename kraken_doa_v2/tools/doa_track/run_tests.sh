#!/bin/bash
# The bearing-tracker matrix against a fake heimdall (docs/dev/doa-tracking.md).
# Needs wsl_setup.sh first. One kraken_doa_test (ports 2808x / 2809x, run dir
# ~/kdt/run, fresh settings), one fake-heimdall run per case.
#
#   wsl -e bash kraken_doa_v2/tools/doa_track/run_tests.sh scene:profile[:amp] ...
#   e.g. run_tests.sh pass100:normal pass30:fast bursty30:normal:6
# Scenes: fake_heimdall.py SCENES. Results: tools/doa_track/results/
# (run_<scene>_<profile>_a<amp>.json; python3 summarize.py results).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
R="$HERE/results"
RUN=~/kdt/run
mkdir -p "$R"
[ -x "$RUN/kraken_doa_test" ] || { echo "no test build - run wsl_setup.sh first"; exit 1; }
cp ~/kdt/src/kraken_doa_v2/kraken_doa.html "$RUN/"
rm -f "$RUN/doa_settings.json"
cd "$RUN"
export PATH=$HOME/.local/bin:$PATH
KRAKEN_AI_MCP_PORT=28079 KRAKEN_DOA_VERBOSE_LOG=1 ./kraken_doa_test > "$R/kraken_doa.log" 2>&1 < /dev/null &
KD=$!
trap 'kill $KD 2>/dev/null; pkill -f "[f]ake_heimdall.py" 2>/dev/null' EXIT
sleep 3
for c in "$@"; do
  IFS=: read -r scene prof amp <<< "$c"; amp=${amp:-20}
  tag="${scene}_${prof}_a$amp"
  rm -f "$R/truth_$scene.json"
  python3 "$HERE/fake_heimdall.py" --scene "$scene" --amp "$amp" --truth "$R/truth_$scene.json" > "$R/fh_$tag.log" 2>&1 &
  FH=$!
  sleep 1
  echo "=== $tag"
  python3 "$HERE/track_client.py" --truth "$R/truth_$scene.json" --profile "$prof" --out "$R/run_$tag.json" \
      > "$R/client_$tag.log" 2>&1 || tail -5 "$R/client_$tag.log"
  wait $FH
  grep -E "behind real time" "$R/fh_$tag.log" | tail -1
  sleep 2
done
python3 "$HERE/summarize.py" "$R"
