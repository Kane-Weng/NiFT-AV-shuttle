# maps

Lanelet2 maps for the route planner (`nift_waypoint`). Its `setup.py` installs every `maps/lanelet2*.osm` into the package's `share/nift_waypoint/maps/` folder, and `waypoint_node` loads the one its `map_file` parameter names.

| File | What it is |
|---|---|
| `lanelet2_odr2024.osm` | 507 lanelets of road network, generated from the 2024 OpenDRIVE file of the public digital twin below. It is the default map, on the shuttle and in simulation, and it matches the world a stock CARLA server with the twin loads. Coordinates are WGS84 lat/lon, so the stack's map origin (`nift_common/nift_common/site.py`) turns them into map-frame metres with no offsets. |

## Source and license

`lanelet2_odr2024.osm` is a derived work. We converted the OpenDRIVE road description (`.xodr`) of the digital twin at [mcity/mcity-digital-twin](https://github.com/mcity/mcity-digital-twin) into Lanelet2: one lanelet per driving lane per lane section, successor links from the OpenDRIVE lane links, a speed limit tag on every lanelet, and small repairs where lane ends did not meet. The twin is published under the MIT License, and its notice is reproduced here as that license requires. The rest of this repository is Apache-2.0 (see the root `LICENSE`).

This project is not affiliated with or endorsed by the owners of the twin or of the test facility it models.

```
MIT License

Copyright (c) 2024 Quantum Signal AI LLC
Copyright (c) 2024 The Regents of the University of Michigan

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

## Using another map

Any Lanelet2 `.osm` with WGS84 coordinates works. Pass its path as `map_file`, and set `map_origin_lat` / `map_origin_lon` to a point near it. Lanelet2's own example map, which `ros-humble-lanelet2-maps` installs, is a good first test (see the `nift_waypoint` README).
