`mapping_example.osm` is the example map of Lanelet2 1.2.3 (`lanelet2_maps/res`), copied unchanged for the equivalence test. BSD-3-Clause, Copyright 2018 FZI Forschungszentrum Informatik, see `LICENSE.lanelet2`. `../fixtures/mapping_example.json.gz` holds results computed from it.

The NiFT map is not copied here. The tests read it from `maps/lanelet2_odr2024.osm` at the repository root. That map is derived from the Mcity Digital Twin and keeps its MIT License, whose notice is in `maps/README.md` at the repository root. The data computed from it carries the same notice: `../fixtures/odr2024.json.gz` here, and the node cache `py/fakelanelet2/lanelet2/node_cache/a148057a7545a8fd.bin`, which ships with the stand-in (its copy of the notice is `py/fakelanelet2/LICENSES/MIT-mcity-digital-twin.txt`).

This project is not affiliated with or endorsed by FZI, the Lanelet2 project, or the owners of the twin.
