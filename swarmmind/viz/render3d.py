"""Offline triangle/splat renderer with a numpy z-buffer.

Terrain, procedural ruins and articulated units share source geometry with Godot.
These PNG previews verify the mathematics before native rendering; they also run on
machines without Godot or a display. No simulator data or RNG state is modified.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..sim.robot import CHASSIS, LANES
from ..viz.render import LANE_COLOR
from .burial import BURIED_LIFT, SURFACE_LIFT, body_arrays, needs_excavation, rubble_parts
from .prop_stream import TILE_METRES, tile_primitives
from .prop_stream import TRIANGLES as PROP_TRIANGLES
from .terrain_surface import TerrainSurface
from .units import REST, animation_state, build_model, posed_parts

#: Sun direction for the fake diffuse term. Low and off to one side so slopes read.
_LIGHT = np.array([0.45, 0.35, 0.82], dtype=np.float32)
_LIGHT /= np.linalg.norm(_LIGHT)

#: Cool overcast daylight, with a warm sediment haze close to the ground.
SKY_TOP = np.array([115, 137, 145], dtype=np.float32)
SKY_HORIZON = np.array([177, 185, 178], dtype=np.float32)

#: Airborne dust, pooling low the way real haze does.
#:
#: Plain distance fog is wrong here: the operator's camera sits ~340 m out, so any
#: density thick enough to feel atmospheric in first-person erases the entire map from
#: the eagle view. Attenuating density with altitude fixes both at once -- an exponential
#: atmosphere is what actually happens, and it happens to be exactly what a dashboard
#: needs. Down among the rubble it is thick; from above you look straight through it.
FOG_COLOR = np.array([177, 185, 178], dtype=np.float32)
FOG_DENSITY = 0.020
FOG_SCALE_HEIGHT = 14.0
FOG_MAX = 0.93

#: **Display palette, deliberately separate from perception/raster.py.**
#:
#: The appearance raster is tuned for the detector: dark ground, warm rubble that is
#: genuinely confusable with a casualty, low contrast. Those are the right choices for
#: computer vision and the wrong ones for a human watching a dashboard. What the cameras
#: see and what the operator sees are different problems, so they get different palettes.
DISPLAY_FLOOR = np.array([104, 101, 86], dtype=np.float32)
DISPLAY_RUBBLE = np.array([116, 95, 73], dtype=np.float32)
DISPLAY_WALL = np.array([150, 146, 135], dtype=np.float32)
#: How much darker a vertical face is than a lit top.
SIDE_SHADE = 0.52

#: Water, drawn as a surface at `height + depth` rather than as a tint on the bed.
#:
#: Without it a river is a dry trench with a dark scratch down it, which is exactly what
#: the terrain looked like before: the `water` field gated traversal but nothing drew it,
#: so the one feature that divides the map was invisible to the person watching. Shallow
#: water keeps some of the bed's colour, deep water hides it, and both carry a wash of
#: sky because a flat surface under an overcast sky is mostly reflecting it.
WATER_SHALLOW = np.array([92, 108, 100], dtype=np.float32)
WATER_DEEP = np.array([38, 58, 76], dtype=np.float32)
WATER_SKY_MIX = 0.26
#: Depth at which water reaches its deep colour. Rivers run to ~1.1 m, marsh to ~0.4, so
#: the two tiers of barrier are visibly different tiers.
WATER_OPAQUE_M = 0.85


@dataclass
class Camera3D:
    pos: np.ndarray
    target: np.ndarray
    fov_deg: float = 62.0
    up: tuple[float, float, float] = (0.0, 0.0, 1.0)


def raster_triangles(img, zbuf, cam: Camera3D, vertices, triangles, color,
                     *, atmosphere: bool = False, shaded: bool = True,
                     two_sided: bool = False) -> None:
    """Small flat-shaded, perspective-correct mesh pass for close unit inspection.

    Bounding boxes are clipped before rasterisation. Subpixel and offscreen faces
    never allocate a pixel grid. Terrain can supply pre-shaded vertex colours.
    """
    height, width = img.shape[:2]
    fwd = cam.target - cam.pos
    fwd = fwd / np.linalg.norm(fwd)
    right = np.cross(fwd, cam.up)
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    d = vertices - cam.pos
    depth = d @ fwd
    f = width * .5 / np.tan(np.deg2rad(cam.fov_deg) * .5)
    screen = np.column_stack([width*.5 + (d @ right) / np.maximum(depth, .01) * f,
                              height*.5 - (d @ up) / np.maximum(depth, .01) * f])
    points = vertices[triangles]
    normals = np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0])
    norm = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(norm[:, None], 1e-9)
    diffuse = .40 + .60 * np.clip(normals @ _LIGHT, 0, 1)
    colors = np.asarray(color, dtype=float)
    colors = (np.broadcast_to(colors, (len(triangles), 3)).copy() if colors.ndim == 1
              else colors[triangles].mean(axis=1))
    if shaded:
        colors *= diffuse[:, None]
    if atmosphere:
        mid_z = (cam.pos[2] + points.mean(axis=1)[:, 2]) * .5
        density = FOG_DENSITY * np.exp(-np.maximum(mid_z, 0) / FOG_SCALE_HEIGHT)
        fogf = np.clip(1 - np.exp(-density * depth[triangles].mean(axis=1)), 0, FOG_MAX)
        colors = colors * (1 - fogf[:, None]) + FOG_COLOR * fogf[:, None]
    # Back-face culling also catches winding errors in generated/exported geometry.
    visible = ((norm > 1e-10) & (depth[triangles].min(axis=1) > .05)
               & (two_sided | (np.sum(normals * (cam.pos - points.mean(axis=1)), axis=1) > 0)))
    for k in np.flatnonzero(visible):
        tri = triangles[k]
        p = screen[tri]
        low = np.maximum(np.floor(p.min(axis=0)).astype(int), (0, 0))
        high = np.minimum(np.ceil(p.max(axis=0)).astype(int), (width-1, height-1))
        if np.any(high < low):
            continue
        x0, y0 = low
        x1, y1 = high
        a, b, c = p
        denom = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
        if abs(denom) < 1e-7:
            continue
        yy, xx = np.mgrid[y0:y1+1, x0:x1+1] + .5
        wa = ((b[1]-c[1])*(xx-c[0]) + (c[0]-b[0])*(yy-c[1])) / denom
        wb = ((c[1]-a[1])*(xx-c[0]) + (a[0]-c[0])*(yy-c[1])) / denom
        wc = 1 - wa - wb
        inside = (wa >= -1e-6) & (wb >= -1e-6) & (wc >= -1e-6)
        inverse = wa/depth[tri[0]] + wb/depth[tri[1]] + wc/depth[tri[2]]
        z = 1 / np.maximum(inverse, 1e-9)
        zb = zbuf[y0:y1+1, x0:x1+1]
        take = inside & (z < zb)
        img[y0:y1+1, x0:x1+1][take] = colors[k]
        zb[take] = z[take]


def render_unit(model, *, time: float = 0.0, travel: float = 0.0,
                state: int = REST, width: int = 640, height: int = 540,
                azimuth: float = -42.0, elevation: float = 24.0) -> np.ndarray:
    """Studio view of the actual rig; also used for PNG/APNG design sheets."""
    a, e = np.deg2rad([azimuth, elevation])
    target = np.array([.23 if model.lane == "scoop" else 0, 0, 1.04])
    eye = target + 5.8 * np.array([np.cos(a)*np.cos(e), np.sin(a)*np.cos(e), np.sin(e)])
    cam = Camera3D(eye, target, fov_deg=39)
    ramp = np.linspace(0, 1, height)[:, None, None]
    img = np.broadcast_to(np.array([17, 27, 36]) * (1-ramp)
                          + np.array([35, 49, 58]) * ramp, (height, width, 3)).copy()
    zbuf = np.full((height, width), np.inf)
    # The stage sits fully beyond the near plane at the studio camera distance.
    floor = np.array([[-3, -3, -.01], [3, -3, -.01], [3, 3, -.01], [-3, 3, -.01]])
    raster_triangles(img, zbuf, cam, floor, np.array([[0, 1, 2], [0, 2, 3]]), [38, 52, 61])
    # Concentric, coplanar shadow discs: neutral and deliberately independent of sim.
    for radius, shade in ((1.35, 35), (1.1, 30), (.85, 25)):
        ang = np.arange(49) * (2*np.pi/48)
        vertices = np.column_stack([np.cos(ang)*radius, np.sin(ang)*radius*.75,
                                    np.full(49, .002 + (1.35-radius)*.002)])
        vertices = np.concatenate([[[0, 0, vertices[0, 2]]], vertices])
        triangles = np.array([[0, k, k+1] for k in range(1, 49)])
        raster_triangles(img, zbuf, cam, vertices, triangles, [shade, shade+10, shade+17])
    for part, vertices in posed_parts(model, time=time, travel=travel, state=state):
        raster_triangles(img, zbuf, cam, vertices, part.triangles, np.asarray(part.color)*255)
    return np.clip(img, 0, 255).astype(np.uint8)


class Renderer3D:
    def __init__(self, world, width: int = 960, height: int = 600,
                 side_step: float = 0.5) -> None:
        self.w, self.h = width, height
        self.cell = world.cell
        self.shape = world.shape
        self.surface = TerrainSurface(world.height, world.occ, world.water, world.cell)
        self._tiles = {}
        self._active_strides = {}

    # ------------------------------------------------------------------ geometry

    def _draw_terrain(self, img, zbuf, world, cam, fog, sectors):
        """Draw the same continuous tiles and procedural ruins as the dashboard.

        Close follow views retain full terrain detail across the visible field. Orbit
        simplifies distant tiles. The cache only exists for this offline renderer;
        the live equivalent also unloads tiles after they leave the camera frustum.
        """
        forward = cam.target - cam.pos
        close = np.linalg.norm(forward) < 50
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, cam.up)
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        tan_h = np.tan(np.deg2rad(cam.fov_deg)*.5)
        tan_v = tan_h*self.h/self.w
        focal = self.w*.5/tan_h
        height, width = self.shape
        for y in range(0, height, 32):
            for x in range(0, width, 32):
                x1, y1 = min(x+32, width), min(y+32, height)
                values = self.surface.corners[y:y1+1, x:x1+1]
                centre = np.array([(x+x1)*self.cell*.5, (y+y1)*self.cell*.5,
                                   (values.min()+values.max())*.5])
                radius = np.linalg.norm([(x1-x)*self.cell, (y1-y)*self.cell,
                                         values.max()-values.min()+6])*.5
                d = centre-cam.pos
                depth = d @ forward
                distance = np.linalg.norm(d)
                pixels = self.cell*focal/max(1, distance-radius)
                stride = 1
                while not close and stride < 8 and pixels*stride < 5:
                    stride *= 2
                self._active_strides[(x//32, y//32)] = stride
                if (depth+radius < .05 or
                    abs(d @ right) > depth*tan_h+radius*np.sqrt(1+tan_h*tan_h) or
                    abs(d @ up) > depth*tan_v+radius*np.sqrt(1+tan_v*tan_v)):
                    continue
                key = (x, y, stride)
                if key not in self._tiles:
                    terrain = self.surface.tile_arrays(x, y, x1, y1, stride)
                    wet = self.surface.water_arrays(x, y, x1, y1)
                    self._tiles[key] = terrain, wet
                terrain, wet = self._tiles[key]
                for mesh in (terrain, wet):
                    if not len(mesh.vertices):
                        continue
                    colors = self._world_colors(world, mesh.vertices, mesh.colors[:, :3]*255,
                                                fog, sectors)
                    raster_triangles(img, zbuf, cam, mesh.vertices, mesh.triangles,
                                     colors, atmosphere=True, shaded=False)
        # Props use the runtime's 32m partition and the actual drawn ground LOD.
        step = max(1, int(TILE_METRES/self.cell))
        for y in range(0, height, step):
            for x in range(0, width, step):
                x1, y1 = min(x+step, width), min(y+step, height)
                values = self.surface.corners[y:y1+1, x:x1+1]
                centre = np.array([(x+x1)*self.cell*.5, (y+y1)*self.cell*.5,
                                   (values.min()+values.max()+8)*.5])
                radius = np.linalg.norm([(x1-x)*self.cell, (y1-y)*self.cell,
                                         values.max()-values.min()+8])*.5
                d = centre-cam.pos
                depth = d @ forward
                if (depth+radius < .05 or
                    abs(d @ right) > depth*tan_h+radius*np.sqrt(1+tan_h*tan_h) or
                    abs(d @ up) > depth*tan_v+radius*np.sqrt(1+tan_v*tan_v)):
                    continue
                distance = np.linalg.norm(d)
                lod = 0 if close or distance < 95 else (1 if distance < 210 else 2)
                props = tile_primitives(world.occ[y:y1, x:x1], self.cell,
                                        self._render_height, lod, (x, y),
                                        water=world.water[y:y1, x:x1])
                for prop in props:
                    colors = self._world_colors(world, prop.vertices,
                                                np.tile(np.array(prop.color)*255, (8, 1)),
                                                fog, sectors)
                    raster_triangles(img, zbuf, cam, prop.vertices, PROP_TRIANGLES,
                                     colors, atmosphere=True)

    def _stride_at(self, x, y):
        ix = int(np.clip(x/self.cell, 0, self.shape[1]-1))
        iy = int(np.clip(y/self.cell, 0, self.shape[0]-1))
        return self._active_strides.get((ix//32, iy//32), 1)

    def _render_height(self, x, y):
        return self.surface.height_at_world(x, y, self._stride_at(x, y))

    def _world_colors(self, world, vertices, colors, fog, sectors):
        ix = np.clip((vertices[:, 0]/self.cell).astype(int), 0, self.shape[1]-1)
        iy = np.clip((vertices[:, 1]/self.cell).astype(int), 0, self.shape[0]-1)
        colors = colors.copy()
        if fog:
            colors[~world.explored[iy, ix]] *= .28
        if sectors:
            ids = world.sector_of_cell[iy, ix]
            for k in range(len(world.sector_ids)):
                tint = (self.SECTOR_TINT_ABANDONED if world.sector_abandoned[k]
                        else self.SECTOR_TINT.get(int(world.sector_priority[k])))
                if tint is not None:
                    m = ids == k
                    colors[m] = colors[m]*(1-self.SECTOR_TINT_MIX)+tint*self.SECTOR_TINT_MIX
        return colors

    # ------------------------------------------------------------------ camera

    def orbit(self, world, azimuth_deg: float = 235.0, elevation_deg: float = 34.0,
              dist_frac: float = 1.15) -> Camera3D:
        """Operator's eagle view: an orbit around the map centre."""
        cx = world.scn.map.width_m * 0.5
        cy = world.scn.map.height_m * 0.5
        span = max(world.scn.map.width_m, world.scn.map.height_m) * dist_frac
        a, e = np.deg2rad(azimuth_deg), np.deg2rad(elevation_deg)
        pos = np.array([cx + np.cos(a) * np.cos(e) * span,
                        cy + np.sin(a) * np.cos(e) * span,
                        float(world.height.max()) + np.sin(e) * span], dtype=np.float32)
        target = np.array([cx, cy, self.surface.height_at_world(cx, cy)], dtype=np.float32)
        pos[2] = max(pos[2], self.surface.height_at_world(*pos[:2]) + self.CHASE_CLEARANCE)
        return Camera3D(pos=pos, target=target, fov_deg=48.0)

    def pov(self, world, i: int, eye: float = 1.1, look_ahead: float = 14.0) -> Camera3D:
        """What robot ``i`` is looking at. Eye height sits above its own chassis."""
        _, origin = self.unit_pose(world, i)
        ground = origin[2]
        th = float(world.theta[i])
        pos = np.array([world.pos[i, 0], world.pos[i, 1], ground + eye], dtype=np.float32)
        fwd = np.array([np.cos(th), np.sin(th), -0.12], dtype=np.float32)
        return Camera3D(pos=pos, target=pos + fwd * look_ahead, fov_deg=74.0)

    #: Third-person follow, the view between the 260 m orbit and the robot's own eye.
    #: The distance was chosen off `runs/3d/chase*.png`: at 4 m the 1.4 m chassis is
    #: about a fifth of the frame and unmistakably the subject, while the ground it is
    #: about to cross is still in shot -- which is the whole point of this view. An orbit
    #: shows the swarm, a POV shows the fog, and neither shows one unit *doing*
    #: something. At 5 m the near-field rubble starts winning the frame.
    CHASE_DIST = 4.0
    #: 17.2 deg is the 0.30 rad `CHASE_PITCH` the dashboard starts at, to the digit.
    CHASE_ELEV_DEG = 17.2
    #: **Vertical**, because that is what `Camera3D.fov` means in Godot under the default
    #: `KEEP_HEIGHT`, and this camera is close enough that mistaking one for the other
    #: changes the framing by a third. `Camera3D.fov_deg` here is horizontal, so the two
    #: are converted through the frame aspect rather than copied across as one number.
    CHASE_FOV_V = 55.0
    #: Look at the middle of the chassis, not its feet: the box is drawn 1.2 m tall
    #: centred 0.6 m up, so this is its waist and the unit sits on the frame centre.
    CHASE_ANCHOR_Z = 0.7
    #: Minimum height above the terrain under the eye. A chase camera reversing into a
    #: hillside is the classic third-person failure and the heightfield is already here.
    CHASE_CLEARANCE = 0.7

    def chase(self, world, i: int, dist: float = CHASE_DIST,
              elevation_deg: float = CHASE_ELEV_DEG, yaw_off_deg: float = 0.0,
              anchor_off=(0.0, 0.0, 0.0)) -> Camera3D:
        """Third-person camera behind robot ``i``, looking at it.

        Reference for `_chase_camera` in godot/scripts/main.gd -- same anchor height,
        same azimuth convention (0 = directly behind the heading, so the unit leads the
        frame and the camera swings round as it turns), same terrain clearance.
        The three arguments are the three controls: ``yaw_off_deg`` is the drag,
        ``dist`` the wheel, and ``anchor_off`` the right-drag that slides the unit
        around inside the frame.
        """
        _, origin = self.unit_pose(world, i)
        ground = origin[2]
        anchor = np.array([world.pos[i, 0], world.pos[i, 1],
                           ground + self.CHASE_ANCHOR_Z], dtype=np.float32)
        anchor = anchor + np.asarray(anchor_off, dtype=np.float32)
        az = float(world.theta[i]) + np.pi + np.deg2rad(yaw_off_deg)
        e = np.deg2rad(elevation_deg)
        pos = anchor + np.array([np.cos(az) * np.cos(e),
                                 np.sin(az) * np.cos(e),
                                 np.sin(e)], dtype=np.float32) * dist
        pos[2] = max(float(pos[2]), float(self.surface.height_at_world(*pos[:2]))
                     + self.CHASE_CLEARANCE)
        fov_h = np.rad2deg(2.0 * np.arctan(
            np.tan(np.deg2rad(self.CHASE_FOV_V) * 0.5) * self.w / self.h))
        return Camera3D(pos=pos, target=anchor, fov_deg=float(fov_h))

    # ------------------------------------------------------------------ render

    #: Tier 3 overlay tints, keyed by sector priority. Abandonment overrides all three.
    #: Deliberately weak (`SECTOR_TINT_MIX`) -- this is a wash over the terrain, not a
    #: repaint. An overlay that hides the map it annotates is worse than no overlay.
    SECTOR_TINT = {
        0: np.array([120, 200, 255], np.float32),   # high     -- cool, "look here"
        2: np.array([90, 90, 110], np.float32),     # low      -- drained
    }
    #: Amber, not red. Red is the hazard, and "on fire" and "written off by the hivemind"
    #: are different claims -- a judge who cannot tell them apart learns nothing from
    #: either. Matches the `sector_abandoned` colour in the dashboard's event feed.
    SECTOR_TINT_ABANDONED = np.array([255, 210, 74], np.float32)
    SECTOR_TINT_MIX = 0.30

    def render(self, world, cam: Camera3D, *, fog: bool = True,
               robots: bool = True, hide: int | None = None,
               sectors: bool = False, fx=None, activity=None,
               victims: bool = False, thermal_unit: int | None = None) -> np.ndarray:
        img = self._sky()
        zbuf = np.full((self.h, self.w), np.inf, dtype=np.float32)

        self._ground_fill(img, zbuf, cam)
        self._draw_terrain(img, zbuf, world, cam, fog, sectors)

        if victims and thermal_unit is None:  # explicit ground-truth view (V/G)
            for victim in world.victims:
                if victim.state >= 3 or np.linalg.norm(victim.pos-cam.pos[:2]) > 90:
                    continue
                x, y = victim.pos
                yaw = (x*1.7+y*3.1) % (2*np.pi)
                rotation, origin = self.surface.robot_pose(x, y, yaw, stride=self._stride_at(x, y))
                buried = needs_excavation(victim.state, victim.buried)
                vertices, triangles, colors = body_arrays()
                body = vertices @ rotation.T + origin + rotation[:, 2]*(BURIED_LIFT if buried else SURFACE_LIFT)
                raster_triangles(img, zbuf, cam, body, triangles, colors,
                                 atmosphere=True, shaded=False, two_sided=True)
                if buried:
                    for part in rubble_parts():
                        raster_triangles(img, zbuf, cam, part.vertices @ rotation.T+origin,
                                         part.triangles, np.asarray(part.color)*255, atmosphere=True)

        if robots:
            # Distant units keep the cheap legible markers. Close units share the
            # actual articulated triangles exported to Godot; no simulation mutation.
            detail = np.linalg.norm(world.pos - cam.pos[:2], axis=1) < 35.0
            rp, rc = self._robot_points(world, hide, exclude=detail)
            if len(rp):
                self._splat(img, zbuf, cam, rp, rc, scale=1.9)
            for i in np.flatnonzero(detail):
                if i == hide:
                    continue
                lane, chassis = int(world.actuator[i]), int(world.chassis[i])
                model = build_model(LANES[lane], CHASSIS[chassis])
                state = animation_state(lane, chassis, int(activity[i]) if activity is not None
                                        else 0, int(world.status[i]), world.pos[i], world.digging)
                heading, offset = self.unit_pose(world, i, self._stride_at(*world.pos[i]))
                for part, vertices in posed_parts(model, time=float(world.t), state=state):
                    colour = np.asarray(part.color) * 255
                    if world.status[i] == 1:
                        colour *= .45
                    elif world.status[i] >= 2:
                        colour *= np.array([.42, .16, .16])
                    raster_triangles(img, zbuf, cam, vertices @ heading.T + offset,
                                     part.triangles, colour, atmosphere=True)

        # Particles last, and they do not write depth. They are alpha-blended dust in
        # front of a solid world; letting them into the z-buffer would have every puff
        # occlude the one behind it and the plume would read as a stack of discs. This
        # is the pass godot/scripts/main.gd's particle MultiMesh is a port of -- the
        # integration and the per-kind constants come from swarmmind/viz/particles.py,
        # which both sides share.
        if fx is not None and len(fx):
            pp, pc, pa, ps = fx.points()
            self._splat(img, zbuf, cam, pp, pc, scale=ps, alpha=pa, write_depth=False)
        if thermal_unit is not None:
            from .thermal import draw_sources, screen_pass

            draw_sources(self, world, cam, img, zbuf, thermal_unit)
            img = screen_pass(img, world.t)
        return np.clip(img, 0, 255).astype(np.uint8)

    def unit_pose(self, world, i: int, stride=1):
        return self.surface.robot_pose(*world.pos[i], float(world.theta[i]),
                                       int(world.chassis[i]), stride,
                                       airborne=bool(world.airborne[i] and world.status[i] < 2))

    def _robot_points(self, world, hide: int | None, exclude=None):
        alive = world.status <= 1
        if exclude is not None:
            alive &= ~exclude
        idx = np.nonzero(alive)[0]
        if hide is not None:
            idx = idx[idx != hide]
        if len(idx) == 0:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.float32)
        ground = np.array([self._render_height(*world.pos[i]) for i in idx])
        flying = world.airborne[idx] & (world.chassis[idx] == 3)
        if flying.any():
            ground[flying] = self.surface.flight_height_at_world(*world.pos[idx[flying]].T)
        pts, cols = [], []
        for dz in (0.3, 0.6, 0.9, 1.2, 1.5):
            pts.append(np.stack([world.pos[idx, 0], world.pos[idx, 1], ground + dz], axis=1))
            shade = 1.0 if dz > 0.6 else 0.7
            c = np.array([LANE_COLOR[LANES[a]] for a in world.actuator[idx]], np.float32)
            c = c * shade
            c[world.status[idx] == 1] *= 0.45           # out of contact
            cols.append(c)
        return (np.concatenate(pts).astype(np.float32),
                np.concatenate(cols).astype(np.float32))

    def _ground_fill(self, img, zbuf, cam: Camera3D) -> None:
        """Paint the ground plane below the horizon before anything else.

        Splats alone cannot cover the near field from eye height: a 0.5 m cell two
        metres away subtends far more screen area than any sane splat cap, so the floor
        breaks into confetti and sky shows through it. Below the horizon every view ray
        meets the ground eventually, so filling it is not a hack -- it is the plane at
        infinity, and the splats then draw the detail on top.
        """
        fwd = cam.target - cam.pos
        fwd = fwd / np.linalg.norm(fwd)
        horiz = np.array([fwd[0], fwd[1], 0.0], dtype=np.float32)
        if np.linalg.norm(horiz) < 1e-6:
            return
        horiz /= np.linalg.norm(horiz)
        up = np.asarray(cam.up, dtype=np.float32)
        right = np.cross(fwd, up)
        right /= np.linalg.norm(right)
        upv = np.cross(right, fwd)

        far = cam.pos + horiz * 1e5
        d = far - cam.pos
        cz = float(d @ fwd)
        if cz <= 0:
            return
        f = (self.w * 0.5) / np.tan(np.deg2rad(cam.fov_deg) * 0.5)
        y_h = int(self.h * 0.5 - (d @ upv) / cz * f)
        if y_h >= self.h:
            return
        y0 = max(0, y_h)
        rows = self.h - y0
        if rows <= 0:
            return
        # Fade toward the horizon so the far plane reads as distance, not a flat band.
        t = np.linspace(0.0, 1.0, rows, dtype=np.float32)[:, None]
        haze = float(np.exp(-max(cam.pos[2], 0.0) / (FOG_SCALE_HEIGHT * 2.5)))
        far_col = FOG_COLOR * haze + np.array([114, 131, 131]) * (1.0 - haze)
        band = far_col[None, :] * (1 - t) + np.array([107, 124, 125])[None, :] * t
        img[y0:, :, :] = band[:, None, :]

    def _sky(self) -> np.ndarray:
        t = np.linspace(0.0, 1.0, self.h, dtype=np.float32)[:, None]
        band = SKY_TOP[None, :] * (1 - t) + SKY_HORIZON[None, :] * t
        return np.repeat(band[:, None, :], self.w, axis=1).copy()

    def _splat(self, img, zbuf, cam: Camera3D, pts, cols, scale, *,
               alpha=None, write_depth: bool = True) -> None:
        """Project points and paint them, nearest-first.

        ``scale`` is metres of splat width and may be a scalar or one value per point --
        particles carry their own size and grow over their life, so a single figure would
        draw a settling dust cloud at the size it was born.
        """
        fwd = cam.target - cam.pos
        fwd = fwd / np.linalg.norm(fwd)
        up = np.asarray(cam.up, dtype=np.float32)
        right = np.cross(fwd, up)
        right /= np.linalg.norm(right)
        upv = np.cross(right, fwd)

        d = pts - cam.pos[None, :]
        cx = d @ right
        cy = d @ upv
        cz = d @ fwd
        near = cz > 0.35
        if not near.any():
            return
        f = (self.w * 0.5) / np.tan(np.deg2rad(cam.fov_deg) * 0.5)
        sx = (self.w * 0.5 + cx[near] / cz[near] * f)
        sy = (self.h * 0.5 - cy[near] / cz[near] * f)
        sz = cz[near]
        sc = cols[near]
        scale = np.asarray(scale, dtype=np.float32)
        sscale = scale[near] if scale.ndim else scale
        salpha = None if alpha is None else np.asarray(alpha, dtype=np.float32)[near]

        # Exponential distance fog, applied per fragment before the depth test. Doing it
        # here rather than as a post-process keeps it renderer-independent -- the Godot
        # shader computes the identical term, so these frames stay a truthful preview.
        mid_z = (cam.pos[2] + pts[near][:, 2]) * 0.5
        density = FOG_DENSITY * np.exp(-np.maximum(mid_z, 0.0) / FOG_SCALE_HEIGHT)
        fogf = np.clip(1.0 - np.exp(-density * sz), 0.0, FOG_MAX)[:, None]
        sc = sc * (1.0 - fogf) + FOG_COLOR[None, :] * fogf

        # Cap generously: from eye height the nearest cells legitimately need tens of
        # pixels, and clamping them low is exactly what produced the confetti floor.
        raw = np.clip(sscale * f / sz, 1.0, 56.0)
        px = sx.astype(np.int32)
        py = sy.astype(np.int32)

        # Bucket by splat size so each size is one vectorised scatter rather than a
        # per-point Python loop. Buckets are fine where splats are small and numerous,
        # coarse where they are large and few.
        buckets = [1, 2, 3, 4, 5, 6, 8, 10, 13, 17, 22, 28, 36, 46, 56]
        edges = np.array(buckets, dtype=np.float32)
        size = edges[np.searchsorted(edges, raw, side="left").clip(0, len(edges) - 1)]
        for k in buckets:
            m = size == k
            if not m.any():
                continue
            off = np.arange(k) - k // 2
            oy, ox = np.meshgrid(off, off, indexing="ij")
            xs = (px[m][:, None] + ox.ravel()[None, :]).ravel()
            ys = (py[m][:, None] + oy.ravel()[None, :]).ravel()
            zs = np.repeat(sz[m], k * k)
            cs = np.repeat(sc[m], k * k, axis=0)
            av = None if salpha is None else np.repeat(salpha[m], k * k)
            ok = (xs >= 0) & (xs < self.w) & (ys >= 0) & (ys < self.h)
            xs, ys, zs, cs = xs[ok], ys[ok], zs[ok], cs[ok]
            if av is not None:
                av = av[ok]
            if not len(xs):
                continue
            if write_depth:
                # Two-pass depth test: reduce to the nearest z per pixel, then write only
                # the fragments that won. Assignment with duplicate indices is otherwise
                # order-dependent and would flicker.
                np.minimum.at(zbuf, (ys, xs), zs)
            win = zs <= zbuf[ys, xs]
            if av is None:
                img[ys[win], xs[win]] = cs[win]
            else:
                a = av[win][:, None]
                img[ys[win], xs[win]] = img[ys[win], xs[win]] * (1.0 - a) + cs[win] * a
