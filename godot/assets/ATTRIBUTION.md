# Third-party assets

## Original rescue fleet (`units/`)

The 15 unit models and their stylized blanket-covered rescue load are original
procedural geometry authored for SwarmMind. They use no third-party
meshes or textures. Source: `swarmmind/viz/units.py`; regenerate with
`uv run python scripts/design_units.py`. See [the unit design notes](../../docs/UNIT_MODELS.md).

The third-party environment and casualty assets below retain their existing credits.

These live inside the Godot project rather than the top-level `assets/` directory
because Godot can only load resources under its own project root -- a mesh outside
`godot/` has no `res://` path and simply cannot be referenced. `assets/` keeps scenario
YAML and model weights; meshes live here.

Every mesh shipped in this repository is listed here with its source and licence. Add a
row before adding an asset — an unattributed asset in a public demo is a liability, and
"I forgot where it came from" is not a defence.

---

## `ruins/` — 33 models, 6.9 MB

**Author: MakoviceMH.** Pack: RuinsGR.

| | |
|---|---|
| Format | glTF binary (`.glb`), one embedded texture per model |
| Exporter | `Khronos glTF Blender I/O v4.3.47` (glTF 2.0) |
| Copyright field | empty in all 56 source models |
| Licence file in pack | none present |

**Credit MakoviceMH in the demo credits and in any public write-up.**

Still worth recording before this is shown publicly, since the pack shipped no licence
file: the source URL and the exact licence terms. If those turn out to prohibit this use,
the map generator does not depend on these meshes — the dashboard renders procedural
geometry from the occupancy grid and the meshes are a visual layer on top. Removing them
costs a look, not a feature.

### What was taken

Structural geometry only — walls, pillars, slabs, stairs, fences. Metre-authored and
grid-friendly: walls are exactly 1.00 m thick and 6.6 m tall, which snaps to the
simulator's 1 m cells.

| group | n | notes |
|---|---:|---|
| `walls/` | 9 | normal + broken, XS/MD/XL, ends, gate. 1.0 x 1.6–6.5 x 6.6 m |
| `walls_broken/` | 6 | pre-collapsed variants — the best fit for a disaster zone |
| `pillars/` | 4 | round and square, full and snapped, 6.2–6.4 m tall |
| `blocks/` | 6 | 6.6 m square slabs, cracked and intact |
| `stairs/` | 3 | including an 11.8 m bridge span |
| `fences/` | 5 | barriers and debris lines |

Per-model dimensions are in `ruins/manifest.json`.

### What was deliberately left out

- **`Moss/`** — green fantasy growth. Wrong register for a dust-covered collapsed city.
- **`Accessories/`** — altar, treasure chests, ancient coins, vases. Fantasy loot.
- **`Skely/`** — rigged skeleton characters, 2.7 MB each. Wrong project entirely.
- **`Sounds/`, `scripts/`, `*Scenes/`, `.import`, `.gdshader`** — the scene files hardcode
  paths that break on move, Godot regenerates `.import`, and the shaders are moss sway.

That removed 25 MB of the pack's 41 MB.

---

## `victims/` — 1 model, 33 KB

**"Curled Up Silence" by bac213tv1** — sketchfab.com/3d-models/curled-up-silence-2d6186002f2a47b187623b23ce01a752, **CC-BY-4.0**. Commercial use allowed, attribution required.

Credit line required by the licence, to be reproduced in the demo credits and any public
write-up:

> This work is based on "Curled Up Silence"
> (https://sketchfab.com/3d-models/curled-up-silence-2d6186002f2a47b187623b23ce01a752)
> by bac213tv1 (https://sketchfab.com/bac213tv1) licensed under CC-BY-4.0
> (http://creativecommons.org/licenses/by/4.0/)

The same line is in the `.glb`'s glTF `asset.copyright` field, so it travels with the file.

A person lying curled on their side — the casualty mesh the ground-truth toggle draws at
every victim position. This is the only asset in the repository that depicts a person, and
it is deliberately the one thing the dashboard shows that the swarm cannot see.

### What was done to it

The download is a **52 MB photogrammetry scan**: 919,567 triangles, 572,560 vertices, and
three 2048² textures. That is 500x the triangle budget of a scattered prop and cannot be
instanced 110 times. The import (M-53) reduced it to **1,801 triangles / 33 KB**, a 1,570x
size cut:

| | source | shipped |
|---|---:|---:|
| triangles | 919,567 | 1,801 |
| vertices | 572,560 | 789 |
| textures | 3 × 2048² | none |
| bytes | 52.3 MB | 33.3 KB |

- **Grid-cluster decimation** to the triangle budget, positions/normals/colour averaged
  per cluster, degenerate and duplicated faces dropped.
- **Textures baked into vertex colours.** The base colour is sampled per vertex and
  multiplied by the same directional term `main.gd` and `viz/render3d.py` bake into the
  world (`LIGHT_DIR`, `n·L * 0.75 + 0.25`), because the marker material is
  `SHADING_MODE_UNSHADED`. The metallic-roughness and normal maps were dropped: an
  unshaded material cannot use either. `albedo_color` in `main.gd` supplies the casualty
  tint over the baked albedo.
- **Rescaled to metres and grounded**: 1.40 m head to feet, 0.49 m tall, origin at the
  contact point so a transform can place it straight onto the heightfield.
- Written back out as a self-contained `.glb` with `POSITION`, `NORMAL`, `COLOR_0` and
  `uint16` indices — no sidecar `.bin`, no external textures.

---

## `nature/` — 12 models, 3.1 MB

**Quaternius — Ultimate Nature Pack.** quaternius.com, CC0 (public domain).

No attribution is legally required under CC0; credited here anyway because it is the
decent thing and costs nothing.

Format is `.gltf` + `.bin` + external textures (not `.glb`). Ten dead trees and two grass
tufts, 4.6–6.2 m tall.

### What was taken, and why so little

The pack is **292 MB**; this import is **3.1 MB**. Two reasons:

1. **Only the dead trees and grass.** Birch, maple and pine, bushes, flowers — all lush
   and green, wrong for a dust-covered collapsed city.
2. **Normal maps dropped: 41 MB of two files.** `NormalTree_Bark_Normal.png` is 19 MB and
   `MapleTree_Bark_Normal.png` is 22 MB. The world shader is `render_mode unshaded` —
   directional shading is baked into vertex colours to match `viz/render3d.py` — so a
   normal map has **no effect whatsoever** on the rendered image.

   The `.gltf` files were rewritten rather than left pointing at absent files: the
   `normalTexture`/`occlusionTexture` entries were removed from each material and the
   now-orphaned textures, images and samplers deleted with all indices remapped. Every
   file is validated (see the import step) — texture sources, sampler indices, and the
   existence of every referenced `.bin` and image.

Per-model dimensions are in `nature/manifest.json`.
