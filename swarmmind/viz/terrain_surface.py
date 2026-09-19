"""Continuous dashboard terrain and grounding; never used by simulation or perception.

The frozen wire height includes random obstacle decoration. Recover an *approximate*
landform by removing its known mean lift, then filtering within wet/dry regions.
Obstacles remain separate occupancy-faithful meshes. Cell-centred samples become one
shared corner lattice; mesh and robot grounding use the exact same triangle diagonal.
Godot's ``terrain_surface.gd`` is the Y-up port of this Z-up reference.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

from .water import river_frame, surface_color

WALL_LIFT = 1.55
RUBBLE_LIFT = 1.0
SMOOTH_PASSES = 2
CLEARANCE = 0.045
WET = 0.02
# Display altitude for the simulator's binary flight layer, in metres. A cached
# clearance envelope starts the climb before a ridge and stays level over ditches.
FLIGHT_CLEARANCE = 8.0
FLIGHT_SLOPE = 0.5
FLIGHT_FOOTPRINT = 2.5
FLIGHT_MAX_STRIDE = 8


def _smoothstep(low, high, value):
    t = np.clip((value - low) / (high - low), 0, 1)
    return t * t * (3 - 2 * t)


def _noise(x, y, scale):
    """Smooth world-space value noise, integer hash shared with the Godot port.

    The display never consumes a simulator random stream. Broad patches, rather than
    independent per-cell colours, let a hillside read as one geological feature.
    """
    x, y = np.asarray(x) / scale, np.asarray(y) / scale
    ix, iy = np.floor(x).astype(np.int64), np.floor(y).astype(np.int64)
    u, v = _smoothstep(0, 1, x-ix), _smoothstep(0, 1, y-iy)

    def hashed(dx, dy):
        n = ((ix+dx+1)*73856093) ^ ((iy+dy+1)*19349663)
        return (((n ^ (n >> 13))*1274126177) & 0x7fffffff) / 2147483647.0

    return (hashed(0, 0)*(1-u)+hashed(1, 0)*u)*(1-v) + (hashed(0, 1)*(1-u)+hashed(1, 1)*u)*v


def _region_smooth(values, wet, passes=2):
    """Filter within each wet/dry class so banks and dry crossings stay separate."""
    result = values.copy()
    h, w = values.shape
    classes = np.pad(wet, 1, mode="edge")
    for _ in range(passes):
        padded = np.pad(result, 1, mode="edge")
        total, weight = result * 4, np.full(values.shape, 4.0)
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            same = classes[1+dy:1+dy+h, 1+dx:1+dx+w] == wet
            total += padded[1+dy:1+dy+h, 1+dx:1+dx+w] * same
            weight += same
        result = total / weight
    return result


class MeshArrays(NamedTuple):
    vertices: np.ndarray
    triangles: np.ndarray
    colors: np.ndarray
    normals: np.ndarray


def _corners(values: np.ndarray) -> np.ndarray:
    pad = np.pad(values, 1, mode="edge")
    return (pad[:-1, :-1] + pad[1:, :-1] + pad[:-1, 1:] + pad[1:, 1:]) * .25


class TerrainSurface:
    def __init__(self, height, occ, water, cell: float, reference=None):
        height = np.asarray(height, dtype=np.float64)
        occ = np.asarray(occ)
        water = np.asarray(water, dtype=np.float64)
        if height.ndim != 2 or not height.size or occ.shape != height.shape or water.shape != height.shape:
            raise ValueError("height, occupancy and water must be equally sized nonempty 2D arrays")
        if cell <= 0 or not np.isfinite(cell) or not np.isfinite(height).all() or not np.isfinite(water).all():
            raise ValueError("terrain requires finite samples and a positive cell size")
        self.cell = float(cell)
        self.gh, self.gw = height.shape
        self.water = water.copy()
        # grid.WALL == 1; grid.RUBBLE == 2. No RNG call, no writes to world arrays.
        recovered = height - np.where(occ == 1, WALL_LIFT, np.where(occ == 2, RUBBLE_LIFT, 0.0))
        wet = water > WET
        smooth = _region_smooth(recovered, wet, SMOOTH_PASSES)
        self.corners = _corners(smooth)
        yy, xx = np.indices(self.corners.shape)
        wx, wy = xx * self.cell, yy * self.cell
        # Small eroded ribs on steep rock faces; no new mountain, road or channel.
        # Units and props sample this same displayed surface. Lowland stays exact.
        dy, dx = np.gradient(self.corners, self.cell)
        rocky = _smoothstep(.48, 1.15, np.hypot(dx, dy))
        count = _corners(wet.astype(float))
        erosion = (1.15*(_noise(wx+7, wy, 11)-.5)
                   + .42*(_noise(wx, wy+19, 3.7)-.5))
        self.corners += rocky * (1-count) * erosion
        self.minimum = float(self.corners.min())
        self.maximum = float(self.corners.max())
        # Shared water corners use ONLY wet neighbours; dry terrain must not tilt a
        # river into a staircase. A tiny shore clearance avoids coplanar flicker.
        water_level = _region_smooth(recovered + water, wet)
        # Round staircase corners inward by up to half a cell.
        # Shared world coordinates keep tile seams closed. No dry cell is flooded.
        cy, cx = np.indices(height.shape)
        mean_x = _corners((cx+.5)*self.cell*wet) / np.maximum(count, 1e-12)
        mean_y = _corners((cy+.5)*self.cell*wet) / np.maximum(count, 1e-12)
        shore = (count > 0) & (count < 1)
        water_x = wx + np.where(shore & (xx > 0) & (xx < self.gw), (mean_x-wx)*.95, 0)
        water_y = wy + np.where(shore & (yy > 0) & (yy < self.gh), (mean_y-wy)*.95, 0)
        self.water_xy = np.stack([water_x, water_y], axis=-1)
        self.water_frame = river_frame(water_x, water_y, reference)
        self.water_corners = np.maximum(
            _corners(water_level * wet) / np.maximum(count, 1e-12),
            self.height_at_world(water_x, water_y) + .025,
        )
        self.water_depths = _corners(water * wet) / np.maximum(count, 1e-12)
        self.rubble = _corners((occ == 2).astype(float))
        dy, dx = np.gradient(self.corners, self.cell)
        self.normals = np.stack([-dx, -dy, np.ones_like(dx)], axis=-1)
        self.normals /= np.linalg.norm(self.normals, axis=-1, keepdims=True)
        # Broad meadow/soil patches, exposed strata, and a damp gravel shore. This
        # palette belongs only to human display, never to the detector's raster.
        broad = _noise(wx+117, wy+53, 43)
        detail = _noise(wx+31, wy+91, 9)
        slope = _smoothstep(.22, .90, np.hypot(dx, dy))
        meadow, grass = np.array([.28, .37, .25]), np.array([.49, .52, .33])
        colors = meadow + (grass-meadow) * (broad*.8+detail*.2)[..., None]
        earth = _smoothstep(.46, .80, _noise(wx+271, wy+9, 22)) * .62
        colors += (np.array([.49, .43, .32])-colors) * earth[..., None]
        strata = .5+.5*np.sin(self.corners*1.9 + _noise(wx, wy, 14)*3)
        rock = np.array([.37, .40, .39]) + (strata*.09+detail*.10)[..., None]
        colors += (rock-colors) * slope[..., None]
        colors += (np.array([.48, .41, .32])-colors) * (self.rubble*.32)[..., None]
        # A short distance field widens the sediment bank without moving the river.
        distance = np.where(count > 0, 0.0, 1e6)
        for _ in range(max(1, int(np.ceil(4/self.cell)))):
            pad = np.pad(distance, 1, mode="edge")
            distance = np.minimum(distance, np.minimum.reduce([
                pad[:-2, 1:-1], pad[2:, 1:-1], pad[1:-1, :-2], pad[1:-1, 2:],
            ]) + self.cell)
        bank = (1-_smoothstep(.3, 4, distance)) * (1-slope*.6)
        sand = np.array([.59, .55, .43]) + (detail-.5)[..., None]*.08
        colors += (sand-colors) * bank[..., None]
        if reference is not None:
            from .reference_landscape import dress_colors

            colors = dress_colors(colors, wx, wy, detail, count, reference)
        colors *= (1-count*.22)[..., None]
        light = np.array([.45, .35, .82])
        light /= np.linalg.norm(light)
        shade = .40 + .60*np.clip(self.normals @ light, 0, 1)
        self.colors = np.concatenate([colors*shade[..., None], np.ones((self.gh+1, self.gw+1, 1))], axis=-1)
        self.flight_corners = None

    def _sample(self, x, y, stride=1, values=None):
        stride = max(1, int(stride))
        gx = np.clip(np.asarray(x, dtype=float) / self.cell, 0, self.gw)
        gy = np.clip(np.asarray(y, dtype=float) / self.cell, 0, self.gh)
        ix = np.minimum((gx/stride).astype(int)*stride, ((self.gw-1)//stride)*stride)
        iy = np.minimum((gy/stride).astype(int)*stride, ((self.gh-1)//stride)*stride)
        nx, ny = np.minimum(ix+stride, self.gw), np.minimum(iy+stride, self.gh)
        u, v = (gx-ix)/(nx-ix), (gy-iy)/(ny-iy)
        values = self.corners if values is None else values
        a, b = values[iy, ix], values[iy, nx]
        c, d = values[ny, ix], values[ny, nx]
        first = u+v <= 1
        z = np.where(first, a+(b-a)*u+(c-a)*v, d+(c-d)*(1-u)+(b-d)*(1-v))
        dx = np.where(first, b-a, d-c)/((nx-ix)*self.cell)
        dy = np.where(first, c-a, d-b)/((ny-iy)*self.cell)
        return z, dx, dy

    def height_at_world(self, x, y, stride=1):
        """Exact triangle interpolation, vectorized over scalars or arrays."""
        return self._sample(x, y, stride)[0]

    def normal_at_world(self, x, y, stride=1):
        _, dx, dy = self._sample(x, y, stride)
        normal = np.stack([-dx, -dy, np.ones_like(dx)], axis=-1)
        return normal / np.linalg.norm(normal, axis=-1, keepdims=True)

    def flight_height_at_world(self, x, y):
        """Continuous clearance above terrain, water, scenery and ground traffic.

        Dilate by the body footprint plus the largest terrain tile stride: even a
        coarse triangle spanning a valley cannot pierce the flight layer. The
        max-plus envelope limits each axis to FLIGHT_SLOPE, lifting approaches to
        mountains instead of snapping upward at their faces. Built once per map;
        sampling costs four lookups and never consumes simulation RNG/state.
        """
        if self.flight_corners is None:
            z = np.maximum(self.corners, np.where(self.water_depths > 0,
                                                 self.water_corners, self.corners))
            radius = int(np.ceil(FLIGHT_FOOTPRINT / self.cell)) + FLIGHT_MAX_STRIDE
            for axis in (0, 1):
                padding = [(0, 0), (0, 0)]
                padding[axis] = (radius, radius)
                padded = np.pad(z, padding, mode="edge")
                for offset in range(2*radius+1):
                    sl = [slice(None), slice(None)]
                    sl[axis] = slice(offset, offset+z.shape[axis])
                    np.maximum(z, padded[tuple(sl)], out=z)
            for axis in (0, 1):
                shape = [1, 1]
                shape[axis] = z.shape[axis]
                ramp = (np.arange(z.shape[axis])*self.cell*FLIGHT_SLOPE).reshape(shape)
                z = np.maximum.accumulate(z+ramp, axis=axis)-ramp
                z = np.flip(np.maximum.accumulate(np.flip(z-ramp, axis=axis), axis=axis), axis=axis)+ramp
            self.flight_corners = z + FLIGHT_CLEARANCE
        return self._sample(x, y, values=self.flight_corners)[0]

    def robot_pose(self, x: float, y: float, heading: float, chassis=0, stride=1,
                   *, airborne=False):
        """(3x3 basis, origin) in Z-up; its footprint rests above this same mesh.

        Flight requires the simulator's airborne telemetry. The footprint contains wheels/tracks,
        articulated feet and the scoop's forward reach. Lattice vertices bound crests
        between wheels that a centre sample or four contact samples would miss.
        """
        if airborne and chassis in (3, "rotor"):
            c, s = np.cos(heading), np.sin(heading)
            basis = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            return basis, np.array([x, y, self.flight_height_at_world(x, y)])
        normal = self.normal_at_world(x, y, stride)
        up = normal / max(normal[2], 1e-9)
        slope = np.linalg.norm(up[:2])
        if slope > .85:
            up[:2] *= .85/slope
        up /= np.linalg.norm(up)
        forward = np.array([np.cos(heading), np.sin(heading), 0.0])
        forward -= up * (forward @ up)
        forward /= np.linalg.norm(forward)
        left = np.cross(up, forward)
        basis = np.column_stack([forward, left, up])
        z = self.support_height(x, y, basis, chassis, stride)
        return basis, np.array([x, y, z])

    def support_height(self, x: float, y: float, basis, chassis=0, stride=1):
        """Conservative origin height for a fixed pose basis, including across LODs.

        A tile boundary may contain multiple strides. Take the maximum result over
        each neighbour's stride while retaining one basis; resampling contact points
        alone can miss a narrow ridge on the finer neighbouring mesh.
        """
        forward, left, up = np.asarray(basis).T
        rotor = chassis in (3, "rotor")
        half_width = 1.05 if rotor else (.92 if chassis in (2, "legged") else .73)
        centre = forward[:2] * (.4 if rotor else .5)
        extent = np.abs(forward[:2]) * (1.45 if rotor else 1.35) + np.abs(left[:2]) * half_width
        # Ground-minus-support-plane is linear on each rendered triangle. Including
        # every lattice vertex in the bounding cells therefore bounds the entire
        # footprint, including ridges that happen to fall between support samples.
        step = max(1, int(stride))
        lo = np.floor((centre-extent+[x, y])/(self.cell*step)).astype(int)*step
        hi = np.ceil((centre+extent+[x, y])/(self.cell*step)).astype(int)*step
        gx, gy = np.meshgrid(np.arange(lo[0], hi[0]+step, step), np.arange(lo[1], hi[1]+step, step))
        gx, gy = np.clip(gx, 0, self.gw), np.clip(gy, 0, self.gh)
        wx, wy = gx*self.cell, gy*self.cell
        plane = -(up[0]*(wx-x)+up[1]*(wy-y))/up[2]
        z = float(np.max(self.corners[gy, gx]-plane))
        z += .14 if chassis in (2, "legged") else CLEARANCE
        return z

    def tile_arrays(self, x0: int, y0: int, x1: int, y1: int, stride=1, *, skirts=True):
        """Indexed Z-up mesh for one cell rectangle. Tile edges must align to stride."""
        stride = max(1, int(stride))
        xs = np.unique(np.r_[np.arange(x0, x1, stride), x1]).astype(int)
        ys = np.unique(np.r_[np.arange(y0, y1, stride), y1]).astype(int)
        xx, yy = np.meshgrid(xs, ys)
        vertices = np.stack([xx*self.cell, yy*self.cell, self.corners[yy, xx]], axis=-1).reshape(-1, 3)
        colors = self.colors[yy, xx].reshape(-1, 4)
        normals = self.normals[yy, xx].reshape(-1, 3)
        ids = np.arange(xx.size).reshape(xx.shape)
        a, b, c, d = ids[:-1, :-1].ravel(), ids[:-1, 1:].ravel(), ids[1:, :-1].ravel(), ids[1:, 1:].ravel()
        triangles = np.concatenate([np.stack([a, b, c], -1), np.stack([b, d, c], -1)])
        if skirts:
            # Full-depth skirts close mixed LOD boundaries and the map's cutaway rim.
            perimeter = np.r_[ids[0], ids[1:, -1], ids[-1, -2::-1], ids[-2:0:-1, 0]]
            bottom = vertices[perimeter].copy()
            bottom[:, 2] = self.minimum - 1.0
            low = np.arange(len(bottom)) + len(vertices)
            nxt, low_next = np.roll(perimeter, -1), np.roll(low, -1)
            triangles = np.concatenate([triangles, np.stack([perimeter, low, nxt], -1), np.stack([nxt, low, low_next], -1)])
            vertices = np.concatenate([vertices, bottom])
            colors = np.concatenate([colors, colors[perimeter]*[.68, .68, .68, 1]])
            normals = np.concatenate([normals, normals[perimeter]])
        return MeshArrays(vertices, triangles.astype(np.int32), colors, normals)

    def water_arrays(self, x0: int, y0: int, x1: int, y1: int):
        """Full-resolution wet mask at every LOD: a coarse tile cannot erase a ford."""
        yy, xx = np.nonzero(self.water[y0:y1, x0:x1] > WET)
        xx, yy = xx+x0, yy+y0
        vx = np.stack([xx, xx+1, xx, xx+1], -1)
        vy = np.stack([yy, yy, yy+1, yy+1], -1)
        vertices = np.concatenate([self.water_xy[vy, vx], self.water_corners[vy, vx, None]], -1).reshape(-1, 3)
        depth = np.clip(self.water_depths[vy, vx].reshape(-1)/.85, 0, 1)
        rgb = surface_color(self.water_frame[vy, vx].reshape(-1, 2), depth)
        colors = np.column_stack([rgb, .84+.12*depth])
        ids = (np.arange(len(xx))*4)[:, None]
        triangles = np.concatenate([ids+[0, 1, 2], ids+[1, 3, 2]]).astype(np.int32)
        return MeshArrays(vertices, triangles, colors, np.tile([0, 0, 1], (len(vertices), 1)))
