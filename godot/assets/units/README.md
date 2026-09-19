# Original SwarmMind rescue units

Generated from `swarmmind/viz/units.py` by:

```bash
uv run python scripts/design_units.py
```

Fifteen original role/chassis combinations, with four named animation clips each.
Every GLB embeds its geometry, vertex colours and animations; there are no textures or
external downloads. `none` means scout, `scoop` digger, `gripper` carrier, and `antenna`
relay. The gripper/rotor combination is intentionally absent.

Use the GLBs as editable standalone assets. The dashboard uses `fleet.json` to batch
the same geometry into MultiMeshes, with GPU joint animation. Keep the JSON file in
the non-resource include filter when packaging a Godot export.

See [the design notes](../../../docs/UNIT_MODELS.md),
[animated gallery](../../../docs/units/index.html), and `catalog.json` for exact counts.
Geometry and preview images were authored for this repository; no third-party models
or textures were used for these units or their stylized rescue load.
