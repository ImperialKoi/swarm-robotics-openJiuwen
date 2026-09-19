"""Perception: appearance raster, occluded cameras, classical detector.

The honesty claim of this project rests on this boundary: robots report victims because
a detector saw pixels, not because the simulator told them. These tests guard the
properties that make that true rather than nominal.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.perception import raster as R
from swarmmind.perception.camera import CameraRig
from swarmmind.perception.classical import ClassicalVictimDetector
from swarmmind.perception.raster import AppearanceRaster
from swarmmind.sim import grid
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World


@pytest.fixture
def w():
    return World(Scenario.load("test"), 42)


def _warmth(c):
    return int(c[0]) - int(c[2])


# --- the raster must not make detection trivial ---------------------------------------


def test_warm_rubble_falls_inside_the_detector_band():
    """The whole point of warm rubble is to be confusable. If its warmth sits outside
    the band the detector keys on, it can never fire and every reported false positive
    would be a lie."""
    d = ClassicalVictimDetector()
    assert d.warm_lo <= _warmth(R.COLOR_RUBBLE_WARM) <= d.warm_hi
    assert R.COLOR_RUBBLE_WARM[0] >= d.red_lo


def test_victim_and_hazard_are_separable_by_the_band():
    d = ClassicalVictimDetector()
    assert d.warm_lo <= _warmth(R.COLOR_VICTIM) <= d.warm_hi
    assert _warmth(R.COLOR_HAZARD) > d.warm_hi, "fire must not be reported as a casualty"


def test_plain_rubble_and_floor_are_outside_the_band():
    d = ClassicalVictimDetector()
    assert _warmth(R.COLOR_RUBBLE) < d.warm_lo
    assert _warmth(R.COLOR_FLOOR) < d.warm_lo


def test_buried_victim_is_smaller_not_recoloured(w):
    """Blending a buried victim toward rubble makes it invisible to every camera, so it
    can never be found -- and therefore never dug out. Size carries the occlusion."""
    ras = AppearanceRaster(w)
    v = next(v for v in w.victims if v.buried)
    v.debris_remaining = 1.0
    buried = (ras.render(w) == R.COLOR_VICTIM).all(axis=-1).sum()
    v.debris_remaining = 0.0
    cleared = (ras.render(w) == R.COLOR_VICTIM).all(axis=-1).sum()
    assert buried > 0, "a fully buried victim must still show some victim-coloured pixels"
    assert cleared > buried, "clearing debris must make the victim visibly larger"


# --- optics ---------------------------------------------------------------------------


def test_frames_have_the_right_shape_and_type(w):
    ras, rig = AppearanceRaster(w), CameraRig(w)
    frames, cells = rig.capture(w, ras.render(w), np.arange(w.n))
    assert frames.shape == (w.n, rig.h, rig.w, 3) and frames.dtype == np.uint8
    assert cells.shape == (w.n, rig.h, rig.w, 2)


def test_walls_occlude(w):
    """A camera cannot see past a wall. Simulating that is optics, not a perception
    shortcut -- but if it is broken, fog-of-war is broken with it."""
    ras, rig = AppearanceRaster(w), CameraRig(w)
    img = ras.render(w)
    wiy, wix = np.nonzero(~w.passable)
    k = len(wix) // 2
    wx, wy = (wix[k] + 0.5) * w.cell, (wiy[k] + 0.5) * w.cell
    # Stand 2 m short of that wall, looking straight at it.
    w.pos[0] = np.array([wx - 2.0, wy])
    w.theta[0] = 0.0
    frames, cells = rig.capture(w, img, np.array([0]))
    occluded = (cells[0, ..., 0] < 0).mean()
    assert occluded > 0.15, f"only {occluded:.1%} of the frame was occluded by a wall"
    # Occluded pixels carry sensor read-noise, not exact zero -- a real camera's shadows
    # are noisy too. What matters is that no scene content survives, and that they stay
    # well below the detector's red threshold so shadow is never reported as a casualty.
    dark = frames[0][cells[0, ..., 0] < 0]
    assert dark.max() < ClassicalVictimDetector().red_lo // 2


def test_camera_geometry_round_trips(w):
    """Image coords -> world must land on the cell the frame actually sampled. A silent
    geometry error here would place every detection at the wrong world position."""
    ras, rig = AppearanceRaster(w), CameraRig(w)
    w.pos[0] = np.array([w.scn.map.width_m / 2, w.scn.map.height_m / 2])
    w.theta[0] = 0.7
    _, cells = rig.capture(w, ras.render(w), np.array([0]))
    for row, col in ((5, 5), (20, 30), (40, 12)):
        if cells[0, row, col, 0] < 0:
            continue
        x, y, _, _ = rig.to_world(w, 0, row, col)
        ix, iy = grid.world_to_cell(np.asarray(x), np.asarray(y), w.cell, w.shape)
        assert (int(iy), int(ix)) == tuple(cells[0, row, col]), f"mismatch at {(row, col)}"


def test_two_robots_see_independent_noise(w):
    """Shared noise would make corroboration between robots meaningless."""
    ras, rig = AppearanceRaster(w), CameraRig(w)
    w.pos[1] = w.pos[0].copy()
    w.theta[1] = w.theta[0]
    w.sensor_radius[1] = w.sensor_radius[0]
    frames, _ = rig.capture(w, ras.render(w), np.array([0, 1]))
    assert not np.array_equal(frames[0], frames[1])


# --- detector -------------------------------------------------------------------------


def _place_facing(w, dist):
    vp = np.array([v.pos for v in w.victims])
    for i in range(w.n):
        k = i % len(w.victims)
        ang = 2 * np.pi * i / w.n
        w.pos[i] = vp[k] + dist * np.array([np.cos(ang), np.sin(ang)])
        w.theta[i] = np.arctan2(vp[k][1] - w.pos[i][1], vp[k][0] - w.pos[i][0])
    return vp


def test_detector_finds_victims_at_close_range(w):
    ras, rig, det = AppearanceRaster(w), CameraRig(w), ClassicalVictimDetector()
    vp = _place_facing(w, 2.0)
    idx = np.arange(w.n)
    frames, _ = rig.capture(w, ras.render(w), idx)
    dets = det.detect(w, rig, frames, idx)
    seen = {
        int(np.argmin(np.linalg.norm(vp - np.array(d.pos), axis=1)))
        for d in dets
        if np.linalg.norm(vp - np.array(d.pos), axis=1).min() <= 3.0
    }
    assert len(seen) >= 6, f"only {len(seen)}/8 victims detected at 2 m"


def test_recall_degrades_with_range(w):
    """A detector that sees as well at 6.5 m as at 2 m is not doing vision.

    This used to assert *precision* degraded instead. That stopped holding once casualty
    placement was fixed to require clearance and nav-grid reachability: casualties now
    stand in open ground, so a robot facing one sees floor rather than the warm rubble
    that produces phantoms. Recall is the property that survives the change -- and
    phantoms are covered by test_phantoms_come_from_rubble below.
    """
    ras, rig, det = AppearanceRaster(w), CameraRig(w), ClassicalVictimDetector()

    def seen_at(dist):
        vp = _place_facing(w, dist)
        frames, _ = rig.capture(w, ras.render(w), np.arange(w.n))
        dets = det.detect(w, rig, frames, np.arange(w.n))
        return {
            int(np.argmin(np.linalg.norm(vp - np.array(d.pos), axis=1)))
            for d in dets
            if np.linalg.norm(vp - np.array(d.pos), axis=1).min() <= 3.0
        }

    near, far = len(seen_at(2.0)), len(seen_at(7.0))
    assert near >= 6, f"only {near}/8 casualties detected at 2 m"
    assert far < near, f"recall did not fall with range ({near} -> {far} of 8)"


def test_phantoms_come_from_rubble(w):
    """Warm-tinted rubble must be able to fool the detector, or the contact tracker's
    whole corroborate-then-resolve machinery is decoration."""
    ras, rig, det = AppearanceRaster(w), CameraRig(w), ClassicalVictimDetector()
    rng = np.random.default_rng(3)
    vp = np.array([v.pos for v in w.victims])

    # Park robots facing rubble, well away from any casualty.
    iy, ix = np.nonzero(w.occ == 2)
    placed = 0
    for k in rng.permutation(len(ix)):
        if placed >= w.n:
            break
        p = np.array([(ix[k] + 0.5) * w.cell, (iy[k] + 0.5) * w.cell])
        if np.linalg.norm(vp - p, axis=1).min() < 20.0:
            continue
        ang = rng.uniform(0, 2 * np.pi)
        w.pos[placed] = p + 4.0 * np.array([np.cos(ang), np.sin(ang)])
        w.theta[placed] = ang + np.pi
        placed += 1

    frames, _ = rig.capture(w, ras.render(w), np.arange(placed))
    dets = det.detect(w, rig, frames, np.arange(placed))
    phantoms = [d for d in dets
                if np.linalg.norm(vp - np.array(d.pos), axis=1).min() > 5.0]
    assert phantoms, "rubble never fooled the detector -- the confuser is not confusing"


def test_detector_is_deterministic(w):
    ras, rig, det = AppearanceRaster(w), CameraRig(w), ClassicalVictimDetector()
    _place_facing(w, 2.5)
    img = ras.render(w)
    w2 = World(Scenario.load("test"), 42)
    ras2, rig2 = AppearanceRaster(w2), CameraRig(w2)
    _place_facing(w2, 2.5)
    fa, _ = rig.capture(w, img, np.arange(w.n))
    fb, _ = rig2.capture(w2, ras2.render(w2), np.arange(w2.n))
    a = det.detect(w, rig, fa, np.arange(w.n))
    b = det.detect(w2, rig2, fb, np.arange(w2.n))
    assert [(d.robot, d.pos, d.conf) for d in a] == [(d.robot, d.pos, d.conf) for d in b]


# --- the trained detector -------------------------------------------------------------
#
# These guard the seam between Kaggle and this laptop. The CNN is trained in torch (OIHW,
# NCHW) and run here in numpy (HWIO, NHWC), and the one failure neither side catches on
# its own is a transposed kernel: it trains fine, exports fine, loads fine, and detects
# nothing. So the transpose is tested here rather than trusted.


def _reference_nchw(x_nhwc, w_oihw, b, stride, pad):
    """An independent convolution in torch's layout, written the slow obvious way.

    Deliberately not sharing code with `cnn.py` -- a reference that reuses the thing it
    is checking proves only that the code equals itself.
    """
    import numpy as np

    n, h, w_, cin = x_nhwc.shape
    cout, _, kh, kw = w_oihw.shape
    x = np.pad(x_nhwc, ((0, 0), (pad, pad), (pad, pad), (0, 0))) if pad else x_nhwc
    oh = (x.shape[1] - kh) // stride + 1
    ow = (x.shape[2] - kw) // stride + 1
    out = np.zeros((n, oh, ow, cout), dtype=np.float64)
    for i in range(oh):
        for j in range(ow):
            patch = x[:, i * stride:i * stride + kh, j * stride:j * stride + kw, :]
            for o in range(cout):
                # w_oihw[o] is (cin, kh, kw); patch is (n, kh, kw, cin).
                out[:, i, j, o] = (patch * w_oihw[o].transpose(1, 2, 0)).sum(axis=(1, 2, 3))
            out[:, i, j, :] += b
    return out


def test_patch_embed_equals_a_general_convolution():
    """Layer 1 is a reshape because its stride equals its kernel. If that ever stops
    being true the reshape silently reads the wrong pixels, so pin it."""
    import numpy as np

    from swarmmind.perception.cnn import ARCH, _conv, _patch_embed

    rng = np.random.default_rng(7)
    x = rng.standard_normal((3, 48, 48, 3)).astype(np.float32)
    w = rng.standard_normal(ARCH["w1"]).astype(np.float32)
    b = rng.standard_normal(ARCH["b1"]).astype(np.float32)
    assert np.allclose(_patch_embed(x, w, b), _conv(x, w, b, stride=4, pad=0), atol=2e-3)


def test_torch_layout_weights_survive_the_export_transpose(tmp_path):
    """**The transpose.** Weights in torch's OIHW, exported as the notebook exports them,
    must reproduce an independent NCHW reference when run through the numpy detector.

    This is the one bug that is invisible on both sides of the Kaggle boundary.
    """
    import numpy as np

    from swarmmind.perception.cnn import ARCH, CNNVictimDetector

    rng = np.random.default_rng(11)
    frames = (rng.random((4, 48, 48, 3)) * 255).astype(np.uint8)
    mean = np.array([90.0, 80.0, 70.0], dtype=np.float32)
    std = np.array([50.0, 50.0, 50.0], dtype=np.float32)

    # Weights as torch holds them: (out, in, kh, kw).
    oihw = {
        "w1": rng.standard_normal((12, 3, 4, 4)).astype(np.float32),
        "w2": rng.standard_normal((20, 12, 3, 3)).astype(np.float32),
        "w3": rng.standard_normal((1, 20, 1, 1)).astype(np.float32),
    }
    biases = {k: rng.standard_normal(v.shape[0]).astype(np.float32)
              for k, v in oihw.items()}

    # The reference, in torch's layout end to end.
    x = (frames.astype(np.float64) - mean) / std
    ref = np.maximum(_reference_nchw(x, oihw["w1"], biases["w1"], 4, 0), 0.0)
    ref = np.maximum(_reference_nchw(ref, oihw["w2"], biases["w2"], 1, 1), 0.0)
    ref = _reference_nchw(ref, oihw["w3"], biases["w3"], 1, 0)[..., 0]

    # The export, exactly as detector.ipynb writes it: OIHW -> HWIO.
    path = tmp_path / "detector.npz"
    np.savez(path, mean=mean, std=std,
             **{k: v.transpose(2, 3, 1, 0) for k, v in oihw.items()},
             **{f"b{k[-1]}": v for k, v in biases.items()})

    z = np.load(path)
    for k, shp in ARCH.items():
        assert z[k].shape == shp, f"{k}: exported {z[k].shape}, cnn.py wants {shp}"

    got = CNNVictimDetector(path).logits(frames)
    assert got.shape == (4, 12, 12)
    assert np.allclose(got, ref, atol=1e-2), (
        f"numpy inference disagrees with the torch-layout reference by "
        f"{np.abs(got - ref).max():.3g} -- the OIHW->HWIO transpose is wrong, and a "
        f"detector built from these weights would train fine and detect nothing"
    )


def test_a_missing_detector_file_fails_loudly(tmp_path):
    """A detector that silently returns nothing looks exactly like a swarm that cannot
    find anyone, and costs a whole mission of measurement to notice."""
    import pytest

    from swarmmind.perception.cnn import CNNVictimDetector

    with pytest.raises(FileNotFoundError, match="no trained detector"):
        CNNVictimDetector(tmp_path / "absent.npz")
