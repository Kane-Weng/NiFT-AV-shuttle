#!/usr/bin/env bash
# File: build_world.sh
#
# Description:
#   Rebuilds the playground's 3D world (web/public/world/) from the public digital twin
#   mcity/mcity-digital-twin (MIT), tag 1.9.1, folder Omniverse/. The result is committed, so
#   this only runs when the world should change. Nothing private is read.
#
#   tools/world3d/build_world.sh [--twin <clone>] [--work <dir>]
#
#     --twin  an existing clone of the twin. Its git objects are used as they are (git archive),
#             so a clone whose work tree is empty works too. Without it, the script makes a
#             partial clone of the public repo at the tag (about 1.9 GB, plain git, no LFS).
#     --work  scratch folder (default: a new temp dir). It needs about 4 GB.
#
#   Needs: git, Blender 5.2 (headless, `blender` on PATH or BLENDER=...), Node 22 and npm,
#   python3 (standard library only). Takes about a minute after the download.
#
#   Steps:
#     1. Source. Omniverse/ at the tag, checked against its git tree id below.
#     2. export_world.py in Blender: USD import without foliage, the exclusion list, flat
#        colours, brand names stripped, map-frame transform, raw .glb, height grid, trees,
#        alignment check against maps/lanelet2_odr2024.osm.
#     3. optimize.mjs (glTF-Transform, pinned in package-lock.json): instancing, join, weld,
#        meshoptimizer simplification, Draco, then the checks (no textures, no brand names,
#        < 5 MB).
#     4. world.json (what the page reads: files, height grid layout, provenance, versions) and
#        NOTICE next to the files.

set -euo pipefail

TWIN_URL=https://github.com/mcity/mcity-digital-twin.git
TWIN_REF=1.9.1
TWIN_DIR=Omniverse/Collected_McityMap_NSR_v4_1_6
# git tree id of Omniverse/ at 1.9.1 (unchanged since commit ca9567a, 2025-05-06)
TWIN_TREE=9640054eca8e4506be46ada1f570611063f00198
BLENDER=${BLENDER:-blender}
BLENDER_WANT=5.2

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../../.." && pwd)
dest="$repo/nift_playground/web/public/world"
osm="$repo/maps/lanelet2_odr2024.osm"

twin=""
work=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --twin) twin=$2; shift 2 ;;
    --work) work=$2; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
work=${work:-$(mktemp -d -t nift-world3d-XXXXXX)}
mkdir -p "$work/src" "$work/out" "$dest"
echo "work dir: $work"

# ---- versions
bver=$("$BLENDER" --version 2>/dev/null | head -1 | awk '{print $2}')
[[ "$bver" == "$BLENDER_WANT"* ]] || echo "warning: Blender $bver, the committed world was built with $BLENDER_WANT" >&2
node --version >/dev/null

# ---- 1. source
if [[ -z "$twin" ]]; then
  twin="$work/twin"
  if [[ ! -d "$twin/.git" ]]; then
    git clone --filter=blob:none --no-checkout --branch "$TWIN_REF" --depth 1 "$TWIN_URL" "$twin"
    git -C "$twin" sparse-checkout set --no-cone /Omniverse/
    git -C "$twin" checkout --quiet "$TWIN_REF"   # fetches the Omniverse/ blobs in one go
  fi
fi
tree=$(git -C "$twin" rev-parse "$TWIN_REF:Omniverse")
commit=$(git -C "$twin" rev-parse "$TWIN_REF^{commit}")
if [[ "$tree" != "$TWIN_TREE" ]]; then
  echo "Omniverse/ at $TWIN_REF is tree $tree, expected $TWIN_TREE" >&2
  exit 1
fi
if [[ ! -f "$work/src/$TWIN_DIR/McityMap_Main.usdc" ]]; then
  git -C "$twin" archive "$TWIN_REF" Omniverse | tar -x -C "$work/src"
fi
usd="$work/src/$TWIN_DIR/McityMap_Main.usdc"
usd_sha=$(sha256sum "$usd" | cut -d' ' -f1)

# ---- 2. Blender
"$BLENDER" -b --python-exit-code 1 --python "$here/export_world.py" -- "$usd" "$osm" "$work/out" \
  > "$work/out/blender.log" 2>&1 || { tail -40 "$work/out/blender.log"; exit 1; }
grep '^\[world3d' "$work/out/blender.log"

# ---- 3. glTF-Transform
(cd "$here" && npm ci --no-audit --no-fund --silent)
node "$here/optimize.mjs" "$work/out/world_raw.glb" "$dest/world.glb" --stats "$work/out/stats.json"
cp "$work/out/height.bin.gz" "$work/out/trees.json" "$dest/"
cp "$here/NOTICE" "$dest/NOTICE"

# ---- 4. world.json
python3 - "$work/out" "$dest" "$here" "$TWIN_URL" "$TWIN_REF" "$commit" "$tree" "$usd_sha" "$bver" <<'EOF'
import json, os, sys
out, dest, here, url, ref, commit, tree, usd_sha, bver = sys.argv[1:]
rep = json.load(open(os.path.join(out, 'export_report.json')))
stats = json.load(open(os.path.join(out, 'stats.json')))
height = json.load(open(os.path.join(out, 'height.json')))
lock = json.load(open(os.path.join(here, 'package-lock.json')))['packages']
ver = lambda p: lock.get('node_modules/' + p, {}).get('version')
files = ['world.glb', 'height.bin.gz', 'trees.json', 'NOTICE']
world = {
    'format': 1,
    'frame': 'map (x east, y north, z up, metres); heights relative to z0',
    'model': 'world.glb',
    'model_up': 'glTF y up: rotate +90 deg about x for the map frame',
    'height': height,
    'trees': 'trees.json',
    'notice': 'NOTICE',
    'bytes': {f: os.path.getsize(os.path.join(dest, f)) for f in files},
    'stats': stats,
    'alignment': rep['alignment'],
    'excluded': rep['excluded'] + [
        {'what': 'all 2009 plants of the twin (source not credited); drawn as our own low-poly trees from trees.json'},
        {'what': 'every texture (flat colours only): no logos, text or photos ship'},
    ],
    'warnings': rep['warnings'],
    'source': {'repo': url, 'ref': ref, 'commit': commit, 'omniverse_tree': tree,
               'usd': 'Omniverse/Collected_McityMap_NSR_v4_1_6/McityMap_Main.usdc', 'usd_sha256': usd_sha},
    'built_with': {'blender': bver, '@gltf-transform/functions': ver('@gltf-transform/functions'),
                   'draco3dgltf': ver('draco3dgltf'), 'meshoptimizer': ver('meshoptimizer'),
                   'map': 'maps/lanelet2_odr2024.osm'},
}
json.dump(world, open(os.path.join(dest, 'world.json'), 'w'), indent=1)
total = sum(world['bytes'].values())
for f, b in world['bytes'].items():
    print(f'{f:16s} {b / 1e6:6.2f} MB')
    if b > 5 * 1024 * 1024:
        sys.exit(f'{f} is over 5 MB')
print(f'{"total":16s} {total / 1e6:6.2f} MB')
EOF
echo "world written to $dest"
