# Lanelet2 stand-in tests

These tests check the pure-Python Lanelet2 stand-in (`nift_playground/py/fakelanelet2`) against the real Lanelet2 1.2.3 of ROS Humble. The native suite needs no Docker and no ROS. It compares the stand-in with saved results of the real Lanelet2 (`fixtures/*.json.gz`). Docker is only needed to refresh those fixtures or the node cache.

| File | What it does |
|---|---|
| `test_standin.py` | Unit tests on small inputs: API names, projection, hypot, C++ container order, centerline, routing, the NiFT map |
| `test_equivalence.py` | The stand-in against the saved real results, on the NiFT map and on Lanelet2's example map, with and without the node cache |
| `test_perf.py` | Load time budget of the NiFT map |
| `equivalence.py` | Query generation and the comparison. `compare(..., strict=True)` accepts no nearest-lanelet ties |
| `lanelet2_dump.py` | Asks one Lanelet2 (real or stand-in) everything the NiFT nodes ask, and writes JSON |
| `compare_real.py` | Runs the dump in Docker against the real Lanelet2, compares, and refreshes `fixtures/` |
| `make_node_cache.py` | Writes `py/fakelanelet2/lanelet2/node_cache/<sha>.bin` from the real Lanelet2 |
| `pyodide_check.mjs` | The same strict check inside Pyodide under Node. Exits with status 1 on any failure |
| `maps/` | Lanelet2's example map, copied for the equivalence test (see `maps/README.md`) |

## Run

From the repository root:

```bash
python3 -m pytest nift_playground/tests/lanelet2 -q
node nift_playground/tests/lanelet2/pyodide_check.mjs   # after npm ci in nift_playground/tests/pyodide
```

The comparison is strict. Projection and map nodes must be within 1 mm, centerline points within 1 cm. Every `findNearest` answer must name the same lanelets in the same order, also where a point is equally near to two overlapping lanelets. Routes must be the same, or cost exactly as much (Lanelet2's boost heap picks between equal-cost routes).

In Pyodide, `lanelet2.io.load` logs a warning when it can't use a node cache, because the browser's C library rounds a few sin and cos results differently from glibc. The map then still loads, but results may differ slightly from the real Lanelet2.

## Python 3.10 in the ROS image

ROS Humble runs Python 3.10, so the suite also runs inside `nift_pub_deps:humble`, with ROS not sourced (the stand-in must win on `sys.path`):

```bash
docker run --rm -v "$PWD":/ws:ro -w /ws -e PYTHONDONTWRITEBYTECODE=1 nift_pub_deps:humble \
  bash -c 'unset PYTHONPATH; python3 -m pytest -p no:cacheprovider nift_playground/tests/lanelet2 -q'
```

## The `nift_pub_deps:humble` image

The image is the official `ros:humble` image plus the system and Python dependencies of our ROS packages, installed with rosdep and pip. It has no NiFT code in it, only the `package.xml` and `requirements.txt` files. That keeps it valid while the code changes. Build it from the repository root:

```bash
CTX=$(mktemp -d)
mkdir -p "$CTX/src"
git ls-files '*package.xml' '*requirements.txt' | grep -v '^_local_no_commit/' \
  | rsync -a --files-from=- . "$CTX/src/"
cat > "$CTX/Dockerfile" <<'EOF'
FROM ros:humble
COPY src /deps/src
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip rsync \
 && rosdep update --rosdistro humble \
 && rosdep install --from-paths /deps/src --ignore-src --rosdistro humble -y \
 && find /deps/src -name requirements.txt -print0 | xargs -0 -r -n1 pip3 install -r \
 && rm -rf /var/lib/apt/lists/*
EOF
docker build -t nift_pub_deps:humble "$CTX"
```

rosdep installs Lanelet2 because `nift_waypoint`, `nift_sensor` and `nift_carla` depend on `lanelet2`. The fixtures in this folder were made with image `a94b4c319076`. It has Ubuntu 22.04.5, Python 3.10.12, glibc 2.35, libstdc++ 6.0.30, `ros-humble-lanelet2` 1.2.3, `libgeographic19` 1.52 and `libboost1.74-dev` 1.74.0. A later build may pull newer patch releases. If `compare_real.py` then reports differences, check the versions first.

## Refresh the fixtures and the node cache

```bash
python3 nift_playground/tests/lanelet2/compare_real.py            # both maps, saves fixtures/
python3 nift_playground/tests/lanelet2/make_node_cache.py         # NiFT map, site origin
```

Both run the real Lanelet2 in `nift_pub_deps:humble` and write files that the native suite then uses. `fixtures/cxx_ground_truth.json.gz` comes from the C++ program in `tools/cxx_ground_truth.cpp.txt`.

## Licences of the data

The stand-in's own licence notes are in `py/fakelanelet2/NOTICE` and `py/fakelanelet2/LICENSES/`. The data here comes from two maps:

- `fixtures/odr2024.json.gz` and the node cache hold results computed from `maps/lanelet2_odr2024.osm`. That map is derived from the Mcity Digital Twin, which is MIT licensed. Its notice is in `maps/README.md` at the repository root.
- `fixtures/mapping_example.json.gz` holds results computed from Lanelet2's example map in `maps/` here, which is BSD-3-Clause (see `maps/README.md` in this folder).
