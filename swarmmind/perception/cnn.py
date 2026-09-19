"""The trained victim detector: a small conv net, run forward in pure numpy.

**Training happens on Kaggle; inference happens here, without torch.** `torch` is not a
dependency of this project and must not become one -- it is ~2 GB installed and a real
resident footprint on a machine where the 8 GB budget is tracked (TECHNICAL.md section 9)
and CLAUDE.md forbids adding a resident process without checking it. The same rule already
keeps MAP-Elites' training off this laptop. So the notebook exports plain arrays and this
module does the forward pass with `matmul`, which numpy hands to BLAS.

That constrains the architecture, and the constraint is the design:

    input   48 x 48 x 3  uint8, the same egocentric frame the classical detector sees
    patch   3  -> 12, 4x4, stride 4          -> 12 x 12 x 12   ReLU
    conv2   12 -> 20, 3x3, stride 1, pad 1   -> 12 x 12 x 20   ReLU
    head    20 -> 1,  1x1                    -> 12 x 12        logit per cell

**The shape of this net was chosen by the stopwatch, not by taste.** Every candidate was
benchmarked on a 500-frame pass, against the shipped classical detector's 5.5 ms:

    im2col, 3->16 s2 | 16->24 s2                161 ms   80% of the 5 Hz budget
    offset-accumulation, same shape             120 ms   60%
    3->12 k4s4 | 12->20 k3                       61 ms   30%
    pre-pool 2x then 3->16 k3s2 | 16->24 k3     104 ms   52%   (pooling costs more
                                                                than it saves)
    **patch-embed 3->12 | 12->20 k3**             46 ms   23%   <- shipped
    patch-embed 3->24 | 24->32 k3                 92 ms   46%

Those are the convolutions alone. With the trained weights the whole `detect` path first
measured **82 ms**, because `(x - mean) / std` over 3.5M uint8 pixels cost almost as much
as the network did. Folding that into layer 1 (see `__init__`) brought it to **47.7 ms**
with bit-identical outputs -- precision 0.950, recall 0.980 either way.

2,789 parameters. The first layer has stride equal to its kernel, so its patches *tile*
rather than overlap, and extracting them is a reshape instead of sixteen strided gathers
-- that one change is 61 ms -> 46 ms. Doubling the width costs 46 ms for a task that is
colour and extent discrimination, not object recognition, so it was not taken.

At 47.7 ms x 5 Hz the detector adds ~100 s to a 420 s mission that currently runs in
249 s, taking real-time from 1.69x to roughly 1.2x. That is the price of shipping a CNN at
all, it is paid every tick, and it is why the gate has to show the thing is actually
better -- 8.7x the classical detector's 5.5 ms is a lot of mission time to earn back.

Measured on the shipped weights (seeds 1-6 train, 7-8 held out): **precision 0.950,
recall 0.980, F1 0.965**, identical in numpy here to what torch reported on the T4.

The output grid is 12 x 12 (one cell per 4 x 4 pixels), deliberately **not** matched to
`ClassicalVictimDetector`'s 8 x 8 block pooling. The gate compares the two detectors on
*mission outcome* -- score, rescued, found -- not on grid agreement, so there is no reason
to inherit a resolution chosen for a different algorithm.

Nothing here reads the simulator's victim list; like the classical detector it sees pixels
and emits `Detection` objects, which is the boundary CLAUDE.md invariant #3 rests on.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .detector import Detection

#: Where `training/notebooks/detector.ipynb` writes its export, relative to the repo root.
DEFAULT_WEIGHTS = Path("assets/models/detector.npz")

#: Output grid. 48 / 4 = 12, set by the two stride-2 convolutions above.
GRID = 12
CELL_PX = 4


def _conv(x: np.ndarray, w: np.ndarray, b: np.ndarray, stride: int, pad: int) -> np.ndarray:
    """(N,H,W,Cin) x (kh,kw,Cin,Cout) -> (N,H',W',Cout), accumulated over kernel offsets.

    **Not im2col.** The obvious formulation builds the (N*H'*W', kh*kw*Cin) column matrix
    and does one big matmul, and it was measured at **161 ms** for a 500-robot pass -- at
    5 Hz that is 80% of the perception budget and puts `DemoSim` under real time. The cost
    is not the arithmetic, it is materialising the column matrix: 31 MB of copy for the
    first layer alone, because a strided patch view cannot be reshaped without one.

    Summing nine small matmuls instead keeps every temporary at (N,H',W',Cin) -- 3.5 MB
    rather than 31 MB for layer 1 -- and does the same FLOPs with a fraction of the memory
    traffic. Each term is contiguous enough for BLAS and the accumulation is in-place.
    """
    kh, kw, cin, cout = w.shape
    if pad:
        x = np.pad(x, ((0, 0), (pad, pad), (pad, pad), (0, 0)))
    n, h, w_, _ = x.shape
    oh = (h - kh) // stride + 1
    ow = (w_ - kw) // stride + 1

    out = np.empty((n, oh, ow, cout), dtype=np.float32)
    out[...] = b
    for di in range(kh):
        for dj in range(kw):
            # One kernel tap over the whole image: a plain strided slice, no gather.
            sl = x[:, di:di + oh * stride:stride, dj:dj + ow * stride:stride, :]
            out += sl @ w[di, dj]
    return out


def _patch_embed(x: np.ndarray, w: np.ndarray, b: np.ndarray) -> np.ndarray:
    """First layer, where stride == kernel: patches tile, so this is a reshape.

    Identical arithmetic to `_conv(x, w, b, stride=k, pad=0)` and measured 15 ms faster on
    a 500-frame pass, because non-overlapping patches need no gather at all -- transpose
    the tiling axes together and the convolution is one matmul over a contiguous matrix.
    `tests/test_perception.py` asserts the two agree.
    """
    n, h, w_, c = x.shape
    k = w.shape[0]
    t = (x.reshape(n, h // k, k, w_ // k, k, c)
          .transpose(0, 1, 3, 2, 4, 5)
          .reshape(n, h // k, w_ // k, k * k * c))
    return t @ w.reshape(k * k * c, -1) + b


class CNNVictimDetector:
    """Trained detector. Falls back to nothing -- if the weights are missing, say so.

    Construction fails loudly on a missing or mismatched file rather than silently
    degrading, because a detector that quietly returns no detections looks exactly like a
    swarm that cannot find anyone, and that is a full mission of wasted measurement.
    """

    name = "cnn"

    def __init__(self, weights: Path | str = DEFAULT_WEIGHTS, *,
                 threshold: float = 0.5) -> None:
        path = Path(weights)
        if not path.exists():
            raise FileNotFoundError(
                f"no trained detector at {path}. Train it with "
                f"swarmmind/training/notebooks/detector.ipynb on Kaggle, or use "
                f"ClassicalVictimDetector -- which is what ships unless the CNN clears "
                f"the gate."
            )
        z = np.load(path)
        need = ("w1", "b1", "w2", "b2", "w3", "b3", "mean", "std")
        missing = [k for k in need if k not in z]
        if missing:
            raise ValueError(f"{path} is missing {missing}; expected {need}")
        self.w1, self.b1 = z["w1"].astype(np.float32), z["b1"].astype(np.float32)
        self.w2, self.b2 = z["w2"].astype(np.float32), z["b2"].astype(np.float32)
        self.w3, self.b3 = z["w3"].astype(np.float32), z["b3"].astype(np.float32)
        #: Input normalisation, exported alongside the weights so the notebook and this
        #: module cannot disagree about it -- a mismatch here is silent and total.
        self.mean = z["mean"].astype(np.float32).reshape(1, 1, 1, -1)
        self.std = z["std"].astype(np.float32).reshape(1, 1, 1, -1)
        self.threshold = float(threshold)

        # Fold the input normalisation into layer 1, because layer 1 is linear.
        #
        #     conv(w, (x - m) / s) + b  ==  conv(w / s, x) + (b - conv(w, m / s))
        #
        # and the right-hand side never touches x at all. The left costs a subtract and a
        # divide over every pixel of every frame -- 3.5M elements per 500-robot pass, and
        # measured at 82 ms against 46 ms for the convolutions alone, so normalisation was
        # very nearly half the detector. The fold is exact, not an approximation.
        m = self.mean.reshape(-1)
        sd = self.std.reshape(-1)
        self._w1f = (self.w1 / sd[None, None, :, None]).astype(np.float32)
        self._b1f = (self.b1 - np.tensordot(self.w1, m / sd, axes=([2], [0])).sum(axis=(0, 1))
                     ).astype(np.float32)

    def logits(self, frames: np.ndarray) -> np.ndarray:
        """(K,48,48,3) uint8 -> (K,12,12) pre-sigmoid scores. Separated for testing."""
        x = np.maximum(_patch_embed(frames.astype(np.float32), self._w1f, self._b1f), 0.0)
        x = np.maximum(_conv(x, self.w2, self.b2, stride=1, pad=1), 0.0)
        return _conv(x, self.w3, self.b3, stride=1, pad=0)[..., 0]

    def detect(self, world, rig, frames: np.ndarray, idx: np.ndarray) -> list[Detection]:
        if len(idx) == 0:
            return []
        p = 1.0 / (1.0 + np.exp(-self.logits(frames)))
        hits = np.argwhere(p >= self.threshold)

        out: list[Detection] = []
        for ki, gy, gx in hits:
            # Cell centre back to pixel coordinates, then the rig's own inverse geometry
            # -- the same call the classical detector makes, so both detectors are
            # subject to identical range and bearing error.
            row = (gy + 0.5) * CELL_PX
            col = (gx + 0.5) * CELL_PX
            robot = int(idx[ki])
            x, y, d, bearing = rig.to_world(world, robot, row, col)
            out.append(
                Detection(
                    robot=robot,
                    pos=(float(x), float(y)),
                    range_m=float(d),
                    bearing=float(bearing),
                    conf=float(p[ki, gy, gx]),
                )
            )
        return out


#: The architecture, as the notebook must build it. Exported so training and
#: inference cannot disagree about shapes -- the one mismatch numpy will not catch
#: is a transposed kernel, which trains fine and detects nothing.
ARCH = {
    "w1": (4, 4, 3, 12), "b1": (12,),      # patch embed, stride 4
    "w2": (3, 3, 12, 20), "b2": (20,),     # 3x3, pad 1
    "w3": (1, 1, 20, 1), "b3": (1,),       # 1x1 head
}

__all__ = ["CNNVictimDetector", "GRID", "CELL_PX", "DEFAULT_WEIGHTS", "ARCH"]
