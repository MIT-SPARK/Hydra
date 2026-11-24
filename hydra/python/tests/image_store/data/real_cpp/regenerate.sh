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
// Keep nonempty fixture directories, but let writers clean empty staging directories.
std::uintmax_t remove_all(const path& p) { return is_empty(p) ? remove(p) : 0; }
std::uintmax_t remove_all(const path& p, std::error_code& ec) {
  return is_empty(p, ec) ? remove(p, ec) : 0;
}
}
CPP
g++ -std=c++17 -shared -fPIC -o "$tmp/libkeep.so" "$tmp/keep.cpp"
# Keep the historical non-finite sample; its dependency-only gtest was removed.
cp -r "$here/tracks/8" "$tmp/8"
mkdir -p "$tmp/out"
TMPDIR="$tmp/out" LD_PRELOAD="$tmp/libkeep.so" "$1" \
  --gtest_filter='KeyframeSelector.SavesAcceptedKeyframes:KeyframeSelector.SensorSuffixForSecondarySensors:KeyframeSelector.SavesDescriptorsOnlyWhenRequested'
TMPDIR="$tmp/out" LD_PRELOAD="$tmp/libkeep.so" "$2" \
  --gtest_filter='TrackLogger.WritesTrackAndKeptImages:TrackLogger.LogsFailedExtraction'
rm -rf "$here/keyframes" "$here/keyframes_multi" "$here/keyframes_features" "$here/tracks"
mkdir -p "$here/keyframes" "$here/keyframes_multi" "$here/keyframes_features" "$here/tracks"
cp -r "$tmp/8" "$here/tracks/"
cp "$tmp"/out/hydra-keyframes-*/keyframes/*.zip "$here/keyframes/"
cp "$tmp"/out/hydra-keyframes-*/keyframes_multi/*.zip "$here/keyframes_multi/"
cp "$tmp"/out/hydra-keyframes-*/on/*.zip "$here/keyframes_features/"
for d in "$tmp"/out/khronos-tracks-*/tracks/* "$tmp"/out/khronos-tracks-*/[0-9]*; do
  [ -f "$d/track.json" ] && cp -r "$d" "$here/tracks/"
done
# every test logs the same sensor; keep one sensors.yaml
for f in "$tmp"/out/khronos-tracks-*/tracks/sensors.yaml "$tmp"/out/khronos-tracks-*/sensors.yaml; do
  [ -f "$f" ] && cp "$f" "$here/tracks/sensors.yaml" && break
done
rm -rf "$tmp"
