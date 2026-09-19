#!/usr/bin/env bash
set -euo pipefail
output="$(cd "$(dirname "$0")" && pwd)"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT
mkdir "$build/source"
tar -xzf "$output/source.tar.gz" -C "$build/source"
emcmake cmake -S "$build/source" -B "$build/objects" \
  -DWITH_3RD_PARTY_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_SKIP_INSTALL_RULES=ON
cmake --build "$build/objects" --target coacd --parallel
em++ -O3 -sMODULARIZE=1 -sEXPORT_ES6=1 -sALLOW_MEMORY_GROWTH=1 \
  -sINITIAL_MEMORY=67108864 -sENVIRONMENT=web,worker,node --bind -std=c++20 \
  -I"$build/source/public" "$output/bind.cpp" "$build/objects/libcoacd.a" \
  -o "$output/coacd.js"
