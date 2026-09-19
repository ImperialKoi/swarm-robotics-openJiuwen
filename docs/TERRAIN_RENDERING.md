# Terrain and visibility

The dashboard renders a damaged river valley from the frozen height/occupancy/water
messages. Meadow and bare soil patches give way to exposed rock on steep faces, with
subtle erosion ribs and strata. Gravel and damp sediment mark the river banks. The
land surface removes the old mean obstacle-height decoration, smooths within wet and
dry regions, and shares vertices between cells. This is an approximate visual
reconstruction: the four demo layouts, navigation, water traversal, perception and
custom 2.5D kinematic simulation remain unchanged. The existing river courses and
mountain locations are retained; this is a rewrite of their display, not hydrology.

Original procedural models populate coherent forest, rock and settlement areas:
layered conifers, fallen trunks and stumps; angular, partly buried rocks; damaged plaster
walls, roof sections, rafters and masonry. Tall geometry stays inside actual dry blocked
footprints. Steep sites receive rock rather than houses. Passable rubble receives only
shallow fragments, and wet cells receive no structural scenery. Low rubble foundations
replace the former tall rectangular plinths. These are visual assets, not new obstacles
or simulator objects; they do not alter what the robot detector sees.

Water preserves the wet-cell classification at every detail level, including dry
crossings. Shared shoreline vertices chamfer into wet cells by less than a quarter
cell, softening grid stair steps without flooding dry ground. Its surface is filtered
within wet regions, coloured by depth, and carries subtle animated reflections in Godot.
Both land and water use the same height-attenuated atmospheric haze. Geometry stays
static; the animated shader cannot submerge a crossing.

## Loading and detail

| View | Terrain and ruins | Units |
|---|---|---|
| Orbit | Terrain strides 1/2/4/8 based on projected cell size; distant ruins simplify | Three mesh levels based on projected size |
| First-person | Every visible terrain chunk and ruin requests full detail | Followed shell hidden; other units use distance detail |
| Chase | Every visible terrain chunk and ruin requests full detail | Followed unit fully detailed; other units use distance detail |

Terrain uses 32-cell tiles and ruins use 32 m tiles. Setup retains height samples and
bounds, not an entire map of GPU meshes. Camera-frustum tests select resident geometry.
Terrain and prop construction each permit at most two tiles per frame, with a 3 ms
budget checked between builds. Near tiles load first. Existing geometry remains until
its replacement is ready, and skirts cover boundaries between different mesh strides.
Newly revealed areas fill progressively after a large camera move.

Offscreen terrain hides immediately and frees after 1.2 seconds. Ruins recheck visibility
every 80 ms, retain an 8 m preload margin, and free after 1.2 seconds outside it. Unit
visibility updates at 20 Hz or immediately on telemetry/terrain changes; inactive mesh
variants free after eight seconds. These reuse intervals avoid repeated allocations
when the camera moves across an edge. Fog of war and sector overlays remain active,
including on water; full-detail follow views do not reveal unexplored information.

## Ground contact

The land mesh and height sampler use the same triangle diagonal. Robot support checks
all terrain vertices in its footprint's containing cells, so a crest between wheels
cannot pierce the chassis plane. The chassis aligns with the local slope, with a tilt
limit; animated feet and rotor guards receive appropriate clearance. Neighbouring
terrain strides use one common support basis at tile boundaries. A tile change
invalidates fleet poses immediately rather than waiting for another state packet.

Camera anchors use the supported chassis height. First-person/chase transitions clear
both the current mesh and its incoming detailed surface; orbit also follows terrain
height and clamps its eye above hills.

## Files and verification

- `swarmmind/viz/terrain_surface.py` and `godot/scripts/terrain_surface.gd`: geometry,
  water, triangle sampling and conservative support.
- `godot/scripts/terrain_stream.gd`: terrain residency and detail selection.
- `swarmmind/viz/prop_stream.py` and `godot/scripts/prop_stream.gd`: procedural ruins.
- `godot/shaders/water.gdshader`: water reflections, fog-of-war and sector overlays.
- `scripts/design_terrain.py`: construct-only landscape review, with overview, river,
  mountain and settlement cameras; optionally exports an initial bridge fixture.
- `godot/scripts/unit_fleet.gd`: lazy variant meshes, visibility and animated batches.
- `godot/tests/*_check.gd`: executable streaming and fleet regressions, invoked by pytest
  when native Godot is installed.

Offline previews and native Compatibility-renderer screenshots were inspected on Apple
M1 with Godot 4.7.2. See [M-78](MEASUREMENTS.md#m-78--streamed-terrain-and-unit-detail)
for geometry counts, timings and their limits. `make check` includes the deterministic
small-fixture mission; no training or demo mission is required for these display checks.

Regenerate landscape previews without advancing a mission:

```bash
uv run python scripts/design_terrain.py
uv run python scripts/design_terrain.py --fixture /tmp/swarmmind-landscape-fixture.json
```

The images and `review.json` go to `runs/3d/landscape/`. The review checks that simulation
arrays, clock and RNG streams are unchanged. All models are generated locally in code;
there are no additional downloads, imported scenery assets or resident processes.
