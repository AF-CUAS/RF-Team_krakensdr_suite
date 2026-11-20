#!/bin/bash
# One-shot: a Linux TEST build of kraken_doa for the bearing-tracker tests
# (docs/dev/doa-tracking.md "Status and how to resume"). For a Windows dev
# machine with WSL Ubuntu and NO sudo: the build dependencies come from
# `apt-get download` unpacked into ~/kdt/prefix, cmake from Kitware's
# tarball, numpy / websockets with pip --user, node for the page's parse
# check. Then a copy of kraken_doa_v2 in ~/kdt/src (CRLF stripped - the
# Windows checkout has autocrlf), its ports moved to 2808x / 2809x, built,
# and a self-signed certificate in the run dir ~/kdt/run.
#
#   wsl -e bash kraken_doa_v2/tools/doa_track/wsl_setup.sh [--deps-only|--build-only]
#
# Re-run it (--build-only) after changing kraken_doa sources on Windows.
# Never run git from WSL in the Windows checkout: WSL's git has no autocrlf,
# sees every file as changed, and a stash / checkout rewrites them as LF.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
K=~/kdt
P=$K/prefix/usr
CMAKE=$K/cmake-3.28.6-linux-x86_64/bin
NODE=$K/node-v20.18.0-linux-x64/bin
mode="${1:-all}"

if [ "$mode" != "--build-only" ]; then
  mkdir -p $K/debs $K/prefix
  (cd $K/debs && apt-get download libopus-dev libopus0 libssl-dev zlib1g-dev libsqlite3-dev libeigen3-dev)
  for d in $K/debs/*.deb; do dpkg -x "$d" $K/prefix; done
  [ -x $CMAKE/cmake ] || (cd $K && curl -sSL https://github.com/Kitware/CMake/releases/download/v3.28.6/cmake-3.28.6-linux-x86_64.tar.gz | tar xz)
  [ -x $NODE/node ] || (cd $K && curl -sSL https://nodejs.org/dist/v20.18.0/node-v20.18.0-linux-x64.tar.xz | tar xJ)
  python3 -m pip install --user -q numpy websockets
fi
[ "$mode" = "--deps-only" ] && exit 0

# the source copy (keeps uWebSockets / liquid-dsp / objects between runs)
mkdir -p $K/src
(cd "$REPO" && tar cf - --exclude=build --exclude=obj --exclude=uWebSockets --exclude='*.o' \
    --exclude=liquid-dsp --exclude='plugins/*/build' --exclude=tools/doa_track/results \
    kraken_doa_v2 scripts) | tar xf - -C $K/src
cd $K/src/kraken_doa_v2
find . -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.c' -o -name Makefile \
    -o -name '*.mk' -o -name '*.sh' -o -name '*.html' -o -name '*.py' \) -exec sed -i 's/\r$//' {} +
sed -i 's/WEB_PORT = 8080;/WEB_PORT = 28080;/; s/DOA_HTTP_PORT = 8081;/DOA_HTTP_PORT = 28081;/;
        s/AI_MCP_PORT = 8079;/AI_MCP_PORT = 28079;/; s/TCP_DATA_PORT = 8091;/TCP_DATA_PORT = 28091;/;
        s/TCP_CONTROL_PORT = 8092;/TCP_CONTROL_PORT = 28092;/' include/config.hpp
grep -q 'WEB_PORT = 28080' include/config.hpp || { echo 'port move failed'; exit 1; }

export PATH=$CMAKE:$PATH CPATH=$P/include:$P/include/eigen3:$P/include/x86_64-linux-gnu \
       LIBRARY_PATH=$P/lib/x86_64-linux-gnu PKG_CONFIG_PATH=$P/lib/x86_64-linux-gnu/pkgconfig:$P/share/pkgconfig
make -j10 check-uws > $K/build.log 2>&1
make -j10 kraken_doa >> $K/build.log 2>&1 || { tail -30 $K/build.log; exit 1; }

mkdir -p $K/run
cp kraken_doa $K/run/kraken_doa_test       # not "kraken_doa": run.sh's stale-process sweep would stop it
cp kraken_doa.html $K/run/
[ -f $K/run/server.crt ] || openssl req -x509 -newkey rsa:2048 -nodes -keyout $K/run/server.key \
    -out $K/run/server.crt -days 365 -subj /CN=localhost 2>/dev/null
echo "test build ready: $K/run/kraken_doa_test (web https://localhost:28080, heimdall ports 28091 / 28092)"
