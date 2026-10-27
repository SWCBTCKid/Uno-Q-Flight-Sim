# Full 787/SFO profile sources

The repository contains acquisition manifests and converters, not an assertion
of ownership over source aircraft, imagery, elevation, map or airport data.

- FlightGear Aircraft-2024 787-8 archive: GPL-2.0 aircraft source and
  textures, fetched from
  `https://fgaddon.b-cdn.net/Aircraft-2024/787-8.zip`. The alpha input archive
  is pinned by SHA-256
  `9061f74b83edeacbbc0b205c128494edb81a29b5b35530e21f1660ddd32018c6`;
  `tools/fetch_b787.py` also verifies every converter input plus `COPYING` and
  `Authors.txt`.
- JSBSim 787-8: distributed with the pinned JSBSim source under its upstream
  license and notices.
- USGS 3DEP elevation: U.S. Government public-domain data; credit USGS.
- USDA NAIP imagery: public-domain imagery; credit USDA/FSA.
- FAA airport/runway/taxiway/building data: U.S. Government data; retain the
  FAA attribution and "not for navigation" disclaimer.
- NOAA shoreline and lidar: retain NOAA and named-partner attribution and
  modification disclaimers.
- OpenStreetMap aeroways: © OpenStreetMap contributors, ODbL 1.0.

Exact endpoints, bounds, selection rules and acquisition constraints are in
`assets/spike-002/sfo-sources.v1.json` and
`assets/spike-002/ksfo-lidar-pdal.v1.json`. Generated release assets must ship
their acquisition/source manifest and these notices.
