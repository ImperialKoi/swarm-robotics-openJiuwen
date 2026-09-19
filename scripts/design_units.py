#!/usr/bin/env python
"""Rebuild the original fleet, editable GLBs and offline animation gallery.

    uv run python scripts/design_units.py

No Godot, browser, Blender, Pillow or network is needed to build the previews.
"""

from __future__ import annotations

import argparse
import os
import struct
import time
import zlib
from pathlib import Path

import numpy as np

from swarmmind.viz import png
from swarmmind.viz.render3d import render_unit
from swarmmind.viz.unit_export import export_catalog
from swarmmind.viz.units import (
    CARRY,
    DIG,
    RELAY,
    REST,
    ROLE_NAMES,
    SCAN,
    build_model,
    valid_variants,
)

ROOT = Path(__file__).resolve().parents[1]
HEROES = (("none", "wheeled", SCAN), ("scoop", "tracked", DIG),
          ("gripper", "legged", CARRY), ("antenna", "rotor", RELAY))
PURPOSE = {
    "none": "Find casualties through an occluded camera. A compact stereo head makes the scout readable at a glance.",
    "scoop": "Uncover buried casualties. Reinforced lift arms, hydraulic rods and a toothed bucket show the excavation stroke.",
    "gripper": "Carry casualties to collection points. Padded jaws secure a visible rescue cradle and blanket.",
    "antenna": "Keep the swarm in contact. A telescopic mast and articulated panel antenna distinguish a relay holding its post.",
}
WORK = {"none": "scan", "scoop": "dig", "gripper": "carry", "antenna": "relay"}


def _apng(path: Path, frames, fps: int, count: int) -> None:
    """Stream full RGB APNG frames, keeping only one raster in memory."""
    with path.open("wb") as stream:
        sequence = 0
        for index, frame in enumerate(frames):
            h, w = frame.shape[:2]
            if index == 0:
                stream.write(b"\x89PNG\r\n\x1a\n")
                stream.write(png._chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
                stream.write(png._chunk(b"acTL", struct.pack(">II", count, 0)))
            stream.write(png._chunk(b"fcTL", struct.pack(">IIIIIHHBB", sequence,
                                                         w, h, 0, 0, 1, fps, 0, 0)))
            sequence += 1
            raw = np.hstack([np.zeros((h, 1), np.uint8), frame.reshape(h, -1)]).tobytes()
            compressed = zlib.compress(raw, 6)
            if index == 0:
                stream.write(png._chunk(b"IDAT", compressed))
            else:
                stream.write(png._chunk(b"fdAT", struct.pack(">I", sequence) + compressed))
                sequence += 1
        stream.write(png._chunk(b"IEND", b""))


def gallery(out: Path, animate: bool) -> None:
    out.mkdir(parents=True, exist_ok=True)
    asset_root = Path(os.path.relpath(ROOT / "godot" / "assets" / "units", out)).as_posix()
    notes_path = Path(os.path.relpath(ROOT / "docs" / "UNIT_MODELS.md", out)).as_posix()
    hero_frames = []
    for lane, chassis, state in HEROES:
        model = build_model(lane, chassis)
        frame = render_unit(model, time=.4, travel=.10, state=state, width=720, height=570)
        png.write(out / f"{model.name}.png", frame)
        hero_frames.append(frame)
        if animate:
            for clip in ("work", "move"):
                # All time-based work completes one full four-second cycle. Rotor
                # samples are dense enough to show the blades without a static alias.
                fps = 48 if chassis == "rotor" else 24
                count = fps * 4
                def frames(model=model, state=state, clip=clip, fps=fps, count=count,
                           chassis=chassis):
                    for n in range(count):
                        t = n / fps
                        travel = 0.0
                        if clip == "move" or state == CARRY:
                            travel = (np.pi / 2 if chassis in ("wheeled", "tracked") else 2) * t / 4
                        yield render_unit(model, time=t, travel=travel,
                                          state=REST if clip == "move" else state,
                                          width=440, height=370)
                _apng(out / f"{model.name}_{clip}.png", frames(), fps, count)
        print(f"  preview: {model.name}", flush=True)
    sheet = np.concatenate([np.concatenate(hero_frames[:2], axis=1),
                            np.concatenate(hero_frames[2:], axis=1)], axis=0)
    png.write(out / "fleet.png", sheet)
    for lane, chassis in valid_variants():
        model = build_model(lane, chassis)
        state = CARRY if lane == "gripper" else REST
        png.write(out / f"{model.name}_small.png",
                  render_unit(model, time=.4, state=state, width=330, height=285))

    cards, rows = [], []
    for index, (lane, chassis, _) in enumerate(HEROES):
        model = build_model(lane, chassis)
        src = f"{model.name}_work.png" if animate else f"{model.name}.png"
        cards.append(f'''<article class="unit lane-{lane}">
<div class="card-head"><span>0{index+1} / {lane.upper()}</span><span>{chassis.upper()} CHASSIS</span></div>
<img class="motion" src="{src}" alt="Animated {ROLE_NAMES[lane].lower()} robot" width="440" height="370">
<div class="copy"><h2>{ROLE_NAMES[lane]}</h2><p>{PURPOSE[lane]}</p>
<div class="clips"><button data-src="{model.name + '_work' if animate else model.name}.png" aria-pressed="true">{WORK[lane].title()}</button>
<button data-src="{model.name + '_move' if animate else model.name}.png" aria-pressed="false">Locomotion</button>
<a href="{asset_root}/{lane}_{chassis}.glb" download>Download GLB ↗</a></div></div></article>''')
    for lane, chassis in valid_variants():
        model = build_model(lane, chassis)
        rows.append(f'''<a class="variant lane-{lane}" href="{asset_root}/{lane}_{chassis}.glb" download>
<img loading="lazy" src="{model.name}_small.png" alt="{ROLE_NAMES[lane]} on {chassis} chassis" width="330" height="285">
<span>{ROLE_NAMES[lane]} <em>{chassis}</em></span><small>{model.triangle_count:,} triangles · 4 clips ↗</small></a>''')
    html = '''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SwarmMind / Rescue fleet</title><style>
:root{color-scheme:dark;font-family:ui-sans-serif,system-ui,sans-serif;background:#101b24;color:#dfebe9}
*{box-sizing:border-box}body{margin:0}main{max-width:1240px;margin:auto;padding:44px 32px 72px}
.eyebrow,.card-head,small,footer,.clips{font-family:ui-monospace,monospace;font-size:11px;letter-spacing:.08em}
.eyebrow{color:#72becf;display:flex;justify-content:space-between;padding-bottom:24px;border-bottom:1px solid #34424a}
header{padding:46px 0 38px;max-width:880px}h1{font-size:clamp(38px,6vw,74px);font-weight:500;letter-spacing:-.05em;line-height:1.02;margin:12px 0 22px}
header p{max-width:660px;font-size:17px;line-height:1.65;color:#acbfc5}a{color:inherit;text-decoration:none}a:hover{text-decoration:underline}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:22px}.unit{border:1px solid #34424a;background:#14212b;overflow:hidden}
.lane-none{--accent:#59c7ff}.lane-scoop{--accent:#ffbf3d}.lane-gripper{--accent:#78ff8c}.lane-antenna{--accent:#d682ff}
.card-head{color:var(--accent);display:flex;justify-content:space-between;padding:19px 23px 0;gap:12px}
.motion{width:100%;height:auto;display:block}.copy{padding:0 25px 25px}h2{font-size:31px;letter-spacing:-.025em;font-weight:500;margin:0 0 12px}
.copy p{color:#a9bcc4;font-size:14px;line-height:1.7;min-height:70px;margin:0 0 23px;max-width:470px}
.clips{display:flex;align-items:center;flex-wrap:wrap;gap:8px;letter-spacing:0}button{font:inherit;border:1px solid #4a5b64;background:none;color:#b9c9ce;padding:9px 12px;cursor:pointer}
button[aria-pressed=true]{background:var(--accent);border-color:var(--accent);color:#14212b}.clips a{margin-left:auto;padding:9px 0;color:var(--accent)}
.catalogue{margin-top:64px}.catalogue>p,.note{color:#99afb9;font-size:14px;line-height:1.7;max-width:780px}
.variants{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px;margin-top:28px}.variant{display:block;border:1px solid #34424a;padding-bottom:18px}
.variant img{display:block;width:100%;height:auto}.variant span,.variant small{display:block;margin:0 15px}.variant span{color:var(--accent);font-size:15px}.variant em{font-style:normal;color:#bac9cd;margin-left:5px}.variant small{color:#8096a1;font-size:10px;margin-top:9px;letter-spacing:0}
footer{margin-top:48px;border-top:1px solid #34424a;padding-top:23px;color:#8096a1;line-height:1.8}
@media(max-width:720px){main{padding:25px 16px}.grid{grid-template-columns:1fr}.variants{grid-template-columns:repeat(2,minmax(0,1fr))}.copy p{min-height:0}.eyebrow{font-size:9px}.card-head{font-size:10px}}
@media(prefers-reduced-motion:reduce){.motion{visibility:hidden}.unit{background-size:contain}}
</style><main><div class="eyebrow"><span>SWARMMIND / FIELD ROBOTICS</span><span>DESIGN SERIES 01</span></div>
<header><h1>One rescue.<br>Four kinds of help.</h1><p>A modular fleet for disaster search and rescue. Distinct tools make every role visible; interchangeable chassis make terrain capability visible. Fifteen original 3D models, each with named animation clips.</p></header>
<div class="grid">''' + "\n".join(cards) + '''</div>
<section class="catalogue"><h2>The full fleet</h2><p>Four purposes × four chassis. Carriers stay on the ground: rotor units cannot lift casualties. Select any model to download its editable, self-contained GLB.</p>
<div class="variants">''' + "\n".join(rows) + '''</div></section>
<p class="note">These are studio animation previews. In the live dashboard, measured travel drives wheels and legs, excavation telemetry drives buckets, and carrying activity controls the rescue load. Rotor mechanisms animate at ground height because the dashboard has no airborne telemetry.</p>
<footer>Original procedural geometry · no texture downloads · glTF 2.0<br>Display models for a custom 2.5D kinematic simulator. <a href="../UNIT_MODELS.md">Design and integration notes ↗</a></footer></main>
<script>
const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
document.querySelectorAll('.unit').forEach(card => {
  const img=card.querySelector('.motion');
  if(reduced){img.src=img.src.replace('_work.png','.png');img.style.visibility='visible'}
  card.querySelectorAll('button').forEach(button => button.addEventListener('click', () => {
    img.src=button.dataset.src;img.style.visibility='visible';
    card.querySelectorAll('button').forEach(b=>b.setAttribute('aria-pressed',String(b===button)));
  }));
});
</script></html>'''
    html = html.replace('href="../UNIT_MODELS.md"', f'href="{notes_path}"')
    (out / "index.html").write_text(html, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=ROOT / "docs" / "units")
    ap.add_argument("--static", action="store_true", help="only regenerate still previews")
    args = ap.parse_args()
    start = time.perf_counter()
    catalog = export_catalog(ROOT / "godot" / "assets" / "units")
    gallery(args.out, not args.static)
    print(f"{len(catalog['models'])} models, {catalog['total_bytes']:,} GLB bytes; "
          f"preview build {time.perf_counter()-start:.2f}s -> {args.out / 'index.html'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
