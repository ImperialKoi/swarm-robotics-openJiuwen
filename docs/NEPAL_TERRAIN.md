# Nepal confluence terrain

The demo uses a **320 × 216 m** local river-confluence crop inspired by the owner's
first two Nepal reference photographs. It replaces the 360 × 240 m procedural map:
11.1% narrower, 10% shorter and **20% less area**. The 1 m grid, robot dimensions,
sensor ranges, speeds and 512-unit roster retain their physical scale.

The photographs have no scale bar, coordinates or elevation data. This is an authored
reconstruction with plausible local proportions, **not a surveyed digital twin**.
It represents only the junction and neighbouring terrace settlement; the distant
mountains and the dam gorge in the other photograph are outside this crop.

| Feature | Scale / layout |
|---|---|
| Water | Two headwaters join one downstream channel; widths 18, 14 and 22 m |
| Water depth | 0.90–1.10 m centre depth, tapering to shallow margins; simulation wading bands, not measured river depths |
| Roads | 4.8 m wide; connected terrace routes and two short dry crossings near the junction |
| Houses | 22 authored 6–8 m footprints, with damaged shells and roof fragments inside blocked cells |
| Relief | Approximately 48 m across the crop, with low banks and rising wooded shoulders |
| Disaster debris | Gravel/silt fans on the lower terraces, rubble and scattered boulders |
| Collection | Base at (30, 40) m; 11 collection points on the road network |
| Hazard | Lengths and drift/growth/recession speeds at about 0.9× the previous map, preserving its timing |

The river elevations decrease toward the outlet and meet at one elevation at the
confluence. The crossings are graded dry **causeways in the 2.5D simulation**; water
does not flow underneath a bridge deck. There is no flood-fluid simulation. Water
and slope still gate traversal, the Tier 1 safety floor remains active, and casualty
discovery still comes from robot-camera detections.

`assets/scenarios/demo.yaml` is the source of truth for every layout dimension.
`sim/landscape.py` builds the occupancy, height and water fields; it shares road
grading with the procedural fixture. Road-node elevations are made mutually feasible
before grading short links, so corners cannot introduce a step that strands wheels.
Seeds 42–45 keep the geography and change trees, debris and mission placements.

The Python renderer and Godot apply matching forest, sediment and road colours plus
building/forest prop selection. Godot uses the checked-in display copy at
`godot/assets/landscapes/demo.json`. It is generated from the YAML and an equality test
prevents drift. The frozen wire protocol is unchanged: the bridge still sends the
authoritative height, water and occupancy. These display colours never enter perception.

The M-89 water pass rounds the shoreline inward, preserves dry crossings and uses
channel coordinates for downstream ripples, narrow reflection streaks and broken
shallow-water foam. `viz/water.py` is the static reference for `water.gdshader`;
the Python and Godot meshes use the same channel coordinates and shore vertices.
Before/after river views are in `runs/3d/river_before/` and `runs/3d/river_after/`.
This is a display change; it does not alter water depth, traversability or perception.

Spawn placement now repairs dry/body-clearance and coarse-reachability violations
without resampling valid starts or consuming additional RNG draws. Ground units
must have a route to base on the same grid used for auction bids. M-89 records
construction checks and small-fixture behavior; a full Nepal rehearsal is still needed.

Regenerate the display copy and review without advancing a mission:

```bash
uv run python scripts/design_terrain.py --export-layout \
  --out runs/3d/nepal --views overview river mountain settlement plan
make check
```

The procedural `test.yaml` fixture is retained, including its deterministic scorecard.
All four demo seeds are checked for dry collection points reachable by every ground
chassis. Historical training, gate margins and rescue-rate measurements refer to the
previous terrain. They do not establish mission performance on this crop; a new demo
rehearsal is still needed. No training or full demo mission is part of this terrain change.
