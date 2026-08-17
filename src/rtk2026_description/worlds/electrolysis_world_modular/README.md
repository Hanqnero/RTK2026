# Modular electrolysis world

- `worlds/electrolysis.sdf`: only world settings and model includes.
- `models/platform`: 4000 x 4000 x 80 mm platform.
- `models/rear_wall`: rear wall.
- `models/electrode_cell`: one reusable 600 x 375 mm block with 12 parallel plates.
- `models/aruco_marker`: one standard OpenCV ArUco `DICT_4X4_50`,
  marker id 0, for RGB-D SLAM.
- `scripts/generate_aruco_marker.py`: reproducibly generates the marker texture
  from the predefined OpenCV dictionary.

Each side contains eight blocks arranged as two rows of four blocks. Every
block contains 12 parallel plates. Plate thickness is 25 mm, clear plate gap
is 25 mm, and plate length is 375 mm. There is no raised base under an
electrode block: the plates start on the main platform and their top is
25 mm above it. Both the clear gap between adjacent blocks in one row and
the clear gap between the two block rows are 15 mm. The central passage
between the left and right sides (the world X axis) is 1000 mm between
the nominal 600 mm block envelopes. Four blocks extend towards the rear
wall along the world Y axis and form a 1545 mm long row. The baths retain
their original orientation; only the inner row is rotated by 180 degrees
to reverse the starting plate colour. The clear distance from the nearest
bath edges to the inner surface of the rear wall is 20 mm.
The ArUco landmark has no collision geometry and therefore does not change
the traversable area or the lidar map.

Regenerate its texture with the OpenCV supplied by the simulation image:

```bash
python3 scripts/generate_aruco_marker.py
```

Run:
```bash
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim worlds/electrolysis.sdf
```
