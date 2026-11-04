#!/usr/bin/env bash
# Re-run the C++ gtests that write image logs and copy their outputs here.
# usage: regenerate.sh <test_hydra binary> <test_khronos binary>  (workspace sourced)
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
tmp=$(mktemp -d)
cat > "$tmp/keep.cpp" <<'CPP'
#include <cstdint>
#include <filesystem>
#include <system_error>
namespace std::filesystem {
std::uintmax_t remove_all(const path&) { return 0; }
std::uintmax_t remove_all(const path&, std::error_code& ec) { ec.clear(); return 0; }
}
CPP
g++ -std=c++17 -shared -fPIC -o "$tmp/libkeep.so" "$tmp/keep.cpp"
mkdir -p "$tmp/out"
TMPDIR="$tmp/out" LD_PRELOAD="$tmp/libkeep.so" "$1" \
  --gtest_filter='KeyframeSelector.SavesAcceptedKeyframes:KeyframeSelector.SensorSuffixForSecondarySensors:KeyframeSelector.SavesDescriptorsOnlyWhenRequested'
TMPDIR="$tmp/out" LD_PRELOAD="$tmp/libkeep.so" "$2" \
  --gtest_filter='TrackLogger.WritesTrackAndKeptImages:TrackLogger.LogsFailedExtraction:TrackLogger.NonFiniteValuesAreNull'
rm -rf "$here/keyframes" "$here/keyframes_multi" "$here/keyframes_features" "$here/tracks"
mkdir -p "$here/keyframes" "$here/keyframes_multi" "$here/keyframes_features" "$here/tracks"
cp "$tmp"/out/hydra-keyframes-*/keyframes/*.zip "$here/keyframes/"
cp "$tmp"/out/hydra-keyframes-*/keyframes_multi/*.zip "$here/keyframes_multi/"
cp "$tmp"/out/hydra-keyframes-*/on/*.zip "$here/keyframes_features/"
for d in "$tmp"/out/khronos-tracks-*/tracks/* "$tmp"/out/khronos-tracks-*/[0-9]*; do
  [ -f "$d/track.json" ] && cp -r "$d" "$here/tracks/"
done
rm -rf "$tmp"
