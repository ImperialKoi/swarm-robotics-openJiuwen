# Terrain and visibility

The dashboard renders a damaged river valley from the frozen height/occupancy/water
messages. Meadow and bare soil patches give way to exposed rock on steep faces, with
subtle erosion ribs and strata. Gravel and damp sediment mark the river banks. The
land surface removes the old mean obstacle-height decoration, smooths within wet and
dry regions, and shares vertices between cells. This is an approximate visual
reconstruction: the terrain display preserves navigation, water traversal, perception and
the custom 2.5D kinematic simulation. The existing river courses and
mountain locations are retained; this is a rewrite of their display, not hydrology.

Original procedural models populate coherent forest, rock and settlement areas:
layered conifers, fallen trunks and stumps; angular, partly buried rocks; damaged plaster
walls, roof sections, rafters and masonry. Ordinary tall scenery stays inside actual dry blocked
footprints. Steep sites receive rock rather than houses. Passable rubble receives only
shallow fragments, and wet cells receive no structural scenery. Small irregular scree
stones replace rectangular plinths. These are visual assets, not new obstacles
or simulator objects; they do not alter what the robot detector sees.

Water preserves the wet-cell classification at every detail level, including dry
crossings. Shared shoreline vertices chamfer into wet cells by less than a quarter
cell, softening grid stair steps without flooding dry ground. Its surface is filtered
within wet regions, coloured by depth, and carries subtle animated reflections in Godot.
Both land and water use the same height-attenuated atmospheric haze. Geometry stays
static; the animated shader cannot submerge a crossing.

## Buried casualties

Casualty locations now follow the physical terrain as well. On each of the four demo
maps, 44 buried casualties occupy traversable rubble, 50 surface casualties sit within
3 m of solid obstacles, and 16 remain exposed. Placement keeps dry ground, robot
clearance, fine/coarse navigation access, collection-point exclusions and 14 m spacing.
The border of the map is not treated as shelter. This changes casualty coordinates,
not the terrain or obstacle layout; earlier mission scores used the previous placement.
See [TECHNICAL §3.3](TECHNICAL.md#33-victims) and [M-83](MEASUREMENTS.md#m-83--casualty-placement-follows-debris-and-shelter-19-sep).

The scenario defines which casualties are buried (44 of 110 in the demo). They already
have simulator debris that scoop robots must clear before a carrier can pick them up.
The dashboard now encloses each buried body in a solid rubble mound, with collapsed
slabs, timber and scattered masonry. A small exposed sleeve corresponds to the existing
detector's visible clue. These worksite piles represent debris on approachable ground;
they do not add navigation obstacles or alter the perception raster.

The cover remains while the casualty is hidden or found, and disappears only when its
state becomes cleared. The historical `buried` flag stays true after excavation, so it
must never be used alone to decide whether to draw rubble. The frozen bridge reports
state rather than continuous debris progress; the display does not invent a digging
countdown. Carried and rescued casualties no longer leave duplicate bodies on the ground.

Normal operator view shows piles only for confirmed buried contacts. The `V`/`G`
truth view can show undiscovered piles and the body underneath them, with normal depth
testing so the cover actually occludes it. Reconnection clears stale casualty data.
The separate detector raster and rescue-chain rules remain unchanged.

`swarmmind/viz/burial.py` is the model of record; `scripts/design_burial.py` exports
`godot/assets/victims/burial.json`, loaded by `godot/scripts/burial.gd`. One shared
216-triangle mesh costs 9,504 triangles for all 44 demo piles, in one MultiMesh batch.
The body preview reads the same existing casualty GLB that Godot renders.

Review the actual new sites without advancing a mission:

```bash
uv run python scripts/review_victim_placement.py
```

This writes `placement.png`, `buried.png`, `sheltered.png` and metrics for the four
demo maps under `runs/3d/victim_placement/`. In the top-down audit, gold marks buried
casualties and red marks surface casualties. The 3D frames use explicit ground-truth
view for inspection; they do not reveal these sites to the robots.

```bash
uv run python scripts/design_burial.py
```

This produces `casualty_buried.png` and `casualty_cleared.png` in
`runs/3d/landscape/`, using an isolated illustration of the cleared state without
advancing a mission. It also regenerates the checked-in rubble geometry.

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

Airborne rotors use a separate cached altitude envelope over this same surface. It
clears nearby terrain and water by at least 8 m, covers the full rotor footprint and
all terrain detail levels, and limits rise/run to 0.5 along either map axis. A mountain
therefore raises the approach altitude before the unit reaches its slope. Aircraft
remain level above the ground fleet and scenery; landing restores the terrain support
pose. This display follows the simulator's actual airborne flag, including in follow
cameras and picking. It does not add 3D physics or change detector optics.

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
M1 with Godot 4.7.2. See [M-78](MEASUREMENTS.md#m-78--streamed-terrain-and-unit-detail-17-sep)
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
