#!/usr/bin/env python
"""Export (camera frame, label grid) pairs to train the CV victim detector (D11).

    uv run python scripts/export_frames.py                                  # seeds 1-8
    uv run python scripts/export_frames.py --seeds 1 --every 20 --max-frames-per-seed 300

`swarmmind/perception/cnn.py` trains on Kaggle and runs forward here in numpy; this script
makes the thing it trains on. Output is one `.npz` shard per seed -- `frames` (N,48,48,3)
uint8 and `labels` (N,12,12) uint8, the CNN's exact input and output shapes, so the
notebook needs no geometry of its own and cannot disagree with this repo about any.

**The label needs no geometry inversion, which is the whole reason this is cheap.**
`CameraRig.capture` already returns, beside the pixels, the world cell each pixel is
looking at (`cells`, -1 where the ray left the map or a wall came first). A pixel is
positive when its cell is one the appearance raster paints casualty-coloured. No
projection, no back-projection, and no chance of the label pointing somewhere the rig
does not: the rig itself says where every pixel looked.

**Frames are tapped from the mission's own capture, never captured separately.** A second
`capture()` call draws from `world.rng["perception"]` for sensor noise and shifts every
later draw, so the exported frames would come from a world that had diverged from the one
the detector runs in. Wrapping the rig costs nothing, and makes the training distribution
the inference distribution by construction.

Ground truth is read here deliberately -- `world.victims` *is* the label, and there is no
other way to make one. `scripts/` sits outside the GUARDED trees (`nodes`, `control`,
`hivemind`, `training`) that `tests/test_no_ground_truth_leak.py` enforces, and nothing
here is imported by the swarm. Invariant #3 governs what a robot may read at run time; the
detector these weights end up in still sees nothing but pixels.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from swarmmind.mission import Mission
from swarmmind.perception.cnn import CELL_PX, GRID
from swarmmind.sim import grid
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import RESCUED

#: Training seeds. **Must never intersect `training/gate.HELD_OUT_SEEDS` (101-110)**,
#: which is where the CNN is compared against `perception/classical.py` to decide which
#: detector ships. A detector trained on a map it is then gated on measures memorisation,
#: and the one number the gate exists to produce becomes a lie. `main` re-checks the
#: parsed seeds against 101-110 and refuses rather than trusting this default, because the
#: flag is the easy way to walk into the held-out set by hand.
DEFAULT_SEEDS = "1-8"

#: Frame side, asserted against the CNN's grid so a change to either is caught here rather
#: than as a silent reshape error inside the notebook.
FRAME_PX = GRID * CELL_PX

#: Disc offset templates by radius in cells, mirroring `AppearanceRaster._discs`.
_DISCS: dict[int, tuple[np.ndarray, np.ndarray]] = {}


def parse_seeds(spec: str) -> list[int]:
    """A range, a comma list, or both -- 1-8 / 1,3,5 / 1-4,9 -> a sorted list."""
    out: set[int] = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part[1:]:
            lo, hi = part.split("-", 1)
            out.update(range(int(lo), int(hi) + 1))
        else:
            out.add(int(part))
    return sorted(out)


def victim_cells(world) -> np.ndarray:
    """Boolean (H,W): the cells the appearance raster paints casualty-coloured *now*.

    **Not just the cell the casualty stands in.** `AppearanceRaster.render` stamps each
    casualty as a disc of `0.6 + 0.9 * cleared` cells -- radius 1 while buried, 2 once dug
    out, so 5 or 13 cells of victim colour rather than 1. Labelling only the centre cell
    marks 4 of every 5 genuinely victim-coloured cells *negative* while buried, and 12 of
    13 once dug out -- which teaches the detector that casualty colour predicts absence,
    on the majority of the evidence. The radius formula below is copied
    from `_stamp_disc` on purpose: these two must round identically or the label is
    off by a cell ring.

    A RESCUED casualty is off the field and is not drawn, so it is not labelled either --
    otherwise every frame looking at the depot would carry a phantom positive.
    """
    h, w = world.shape
    mask = np.zeros((h, w), dtype=bool)
    for v in world.victims:
        if v.state == RESCUED:
            continue
        frac = 1.0 - float(np.clip(v.debris_remaining, 0.0, 1.0))
        r_cells = max(1, int(round(0.6 + 0.9 * frac)))
        tpl = _DISCS.get(r_cells)
        if tpl is None:
            tpl = _DISCS[r_cells] = grid.disc_template(float(r_cells))
        dy, dx = tpl
        cix, ciy = grid.world_to_cell(
            np.asarray(v.pos[0]), np.asarray(v.pos[1]), world.cell, world.shape
        )
        mask[np.clip(int(ciy) + dy, 0, h - 1), np.clip(int(cix) + dx, 0, w - 1)] = True
    return mask


def labels_for(cells: np.ndarray, vmask: np.ndarray) -> np.ndarray:
    """(K,48,48,2) pixel cells -> (K,12,12) uint8, one bit per 4x4 block."""
    iy, ix = cells[..., 0], cells[..., 1]
    seen = iy >= 0                       # -1: off-map, or behind a wall the rig marched
    # Index with the clamped pair and mask afterwards; gathering at -1 would wrap to the
    # far edge of the map and invent positives out of the opposite wall.
    pos = vmask[np.where(seen, iy, 0), np.where(seen, ix, 0)] & seen
    k = pos.shape[0]
    return (pos.reshape(k, GRID, CELL_PX, GRID, CELL_PX)
               .any(axis=(2, 4)).astype(np.uint8))


class Shard:
    """One seed's accumulating dataset, subsampled as it arrives."""

    def __init__(self, seed: int, every: float, neg_keep: float,
                 budget: int, per_tap: int) -> None:
        self.seed = seed
        self.every = every
        self.neg_keep = neg_keep
        self.budget = budget
        self.per_tap = per_tap
        self.next_t = 0.0
        self.n_seen = 0
        self.n_pos = 0
        self._frames: list[np.ndarray] = []
        self._labels: list[np.ndarray] = []
        self._kept = 0
        # Own generator, seeded from the mission seed. Invariant #6 forbids global
        # `np.random`, and drawing the subsample from one of `world.rng`'s streams would
        # make the swarm's trajectory depend on whether an export was running.
        self.rng = np.random.default_rng(90000 + seed)

    @property
    def full(self) -> bool:
        return self._kept >= self.budget

    def tap(self, world, frames: np.ndarray, cells: np.ndarray) -> None:
        """Take a slice of one perceive pass, if enough sim time has passed."""
        if self.full or world.t < self.next_t or len(frames) == 0:
            return
        self.next_t = world.t + self.every

        labels = labels_for(cells, victim_cells(world))
        pos = labels.any(axis=(1, 2))
        self.n_seen += len(pos)
        self.n_pos += int(pos.sum())

        # **Keep every positive, throw most negatives away.** Measured on demo seed 1, a
        # raw pass is ~1.9% positive frames: 512 robots mostly photograph rubble, and 8
        # casualties on a 360x240 m map are not in shot. Trained on that, the minimum-loss
        # detector is the constant "no casualty here" -- it scores 98% and finds nobody,
        # which is exactly the failure `CNNVictimDetector`'s docstring warns looks like a
        # swarm that cannot search. Dropping 85% of the empties lifts the rate ~6x for
        # free, and the notebook still reweights what is left.
        neg = ~pos & (self.rng.random(len(pos)) < self.neg_keep)
        take_p = np.nonzero(pos)[0]
        take_n = np.nonzero(neg)[0]
        # Per-tap slice of the budget, not first-come. One perceive pass hands over every
        # robot that moved -- up to 512 frames from a single instant -- so spending the
        # budget greedily buys a shard of nothing but t < 60 s: full fog, no casualty dug
        # out, no carry in progress. Positives are truncated last, since they are the
        # scarce half.
        room = min(self.per_tap, self.budget - self._kept)
        if len(take_p) + len(take_n) > room:
            take_p = self.rng.permutation(take_p)[:room]
            take_n = self.rng.permutation(take_n)[: room - len(take_p)]
        idx = np.sort(np.concatenate([take_p, take_n]).astype(np.int64))
        if len(idx) == 0:
            return
        # Fancy indexing copies, so the shard never holds a view onto a capture buffer.
        self._frames.append(frames[idx])
        self._labels.append(labels[idx])
        self._kept += len(idx)

    def write(self, out: Path) -> tuple[Path | None, int, int, int]:
        """Concatenate once and save. Returns (path, frames, positives, bytes)."""
        if not self._frames:
            return None, 0, 0, 0
        f = np.concatenate(self._frames)
        lab = np.concatenate(self._labels)
        self._frames.clear()
        self._labels.clear()
        out.mkdir(parents=True, exist_ok=True)
        path = out / f"seed{self.seed:02d}.npz"
        # Compressed: the labels are ~99% zeros and occluded pixels are near-black, so a
        # shard lands at a fraction of the 6.9 kB/frame the raw arrays would cost, and
        # Kaggle dataset uploads are the bottleneck here rather than decode time.
        np.savez_compressed(path, frames=f, labels=lab, seed=np.int32(self.seed))
        return path, len(f), int(lab.any(axis=(1, 2)).sum()), path.stat().st_size


def export_seed(scn: Scenario, seed: int, args) -> Shard:
    """Run one headless mission, tapping its camera passes."""
    n_taps = max(1, int(scn.mission_duration_s / args.every) + 1)
    shard = Shard(seed, args.every, args.negative_keep, args.max_frames_per_seed,
                  per_tap=max(1, -(-args.max_frames_per_seed // n_taps)))

    # `hivemind=False`: Tier 3 changes where robots go, not what a camera sees, and
    # invariant #1 means a mission must run without it. Nothing here should wait on a
    # 6 s model cycle for pixels.
    m = Mission(scn, seed, hivemind=False)
    w = m.world
    inner = m.rig.capture

    def capture(world, raster_img, idx):
        frames, cells = inner(world, raster_img, idx)
        shard.tap(world, frames, cells)
        return frames, cells

    m.rig.capture = capture
    while not w.done and w.t < scn.mission_duration_s and not shard.full:
        m.tick()
    return shard


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="export CV detector training shards")
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--seeds", default=DEFAULT_SEEDS,
                    help=f"range or comma list, e.g. {DEFAULT_SEEDS!r} or '1,3,5'; "
                         "must not overlap the gate's held-out 101-110")
    ap.add_argument("--every", type=float, default=5.0,
                    help="sim seconds between camera taps (default 5.0)")
    ap.add_argument("--out", default="runs/frames", type=Path)
    ap.add_argument("--max-frames-per-seed", type=int, default=2000,
                    help="cap per shard, spread evenly over the mission (default 2000)")
    ap.add_argument("--negative-keep", type=float, default=0.15,
                    help="fraction of casualty-free frames to keep (default 0.15)")
    args = ap.parse_args(argv)

    seeds = parse_seeds(args.seeds)
    held = sorted(set(seeds) & set(range(101, 111)))
    if held:
        ap.error(f"seeds {held} are the gate's held-out maps; training on them makes "
                 f"the CNN-vs-classical comparison meaningless")

    scn = Scenario.load(args.scenario)
    print(f"  {args.scenario}: {len(seeds)} seeds {seeds}, tap every {args.every:g} s of "
          f"{scn.mission_duration_s:g} s, keep {args.negative_keep:.0%} of empties, "
          f"<= {args.max_frames_per_seed} frames/seed", flush=True)
    assert FRAME_PX == 48, f"CNN grid implies {FRAME_PX}px frames, rig renders 48"

    tot_kept = tot_pos = tot_bytes = 0
    tot_seen = tot_seen_pos = 0
    for seed in seeds:
        shard = export_seed(scn, seed, args)
        path, n, npos, nbytes = shard.write(args.out)
        tot_kept += n
        tot_pos += npos
        tot_bytes += nbytes
        tot_seen += shard.n_seen
        tot_seen_pos += shard.n_pos
        raw = shard.n_pos / max(1, shard.n_seen)
        print(f"    seed {seed:>3}  {n:>6} frames  {npos:>5} positive ({npos/max(1,n):>6.1%})"
              f"  raw {raw:>6.2%}  {nbytes/1e6:>6.2f} MB  {path}", flush=True)

    print(f"\n  total {tot_kept} frames, {tot_pos} positive "
          f"({tot_pos/max(1,tot_kept):.1%}), {tot_bytes/1e6:.2f} MB in {args.out}")
    print(f"  subsampling lifted the positive rate from {tot_seen_pos/max(1,tot_seen):.2%} "
          f"of {tot_seen} captured frames -- "
          f"{tot_pos/max(1,tot_kept) / max(1e-9, tot_seen_pos/max(1,tot_seen)):.1f}x")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
