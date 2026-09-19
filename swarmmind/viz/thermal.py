"""Simulated thermal display, independent of the detector and all simulation RNGs.

Heat on a buried casualty's cover represents a stylised surface signature, not
infrared transmission through walls. Intensity is relative, never a measured °C.
The Godot port uses the same position hash, response and false-colour ramp.
"""

import numpy as np

MAX_RANGE = 26.0
PALETTE = np.array([[8, 12, 25], [39, 29, 73], [133, 45, 79],
                    [231, 100, 48], [255, 194, 98], [255, 246, 216]]) / 255.0


def variation(x, y):
    ix, iy = np.floor(np.asarray(x)*100), np.floor(np.asarray(y)*100)
    return np.mod(ix*73 + iy*151 + ix*iy*3, 997) / 997


def intensity(x, y, state, buried, time=0.0):
    if state >= 3:  # carried/rescued no longer lie at the truth row's original position
        return 0.0
    noise = variation(x, y)
    base = .34 if buried and state < 2 else .78
    return base + .10*(noise-.5) + .018*np.sin(time*1.7 + noise*6.283185307)


def in_sensor(occ, cell, origin, heading, target):
    """90° unit cone and wall occlusion; rendering adds the actual 3D depth test."""
    delta = np.asarray(target) - origin
    distance = np.linalg.norm(delta)
    forward = delta @ np.array([np.cos(heading), np.sin(heading)])
    lateral = delta @ np.array([-np.sin(heading), np.cos(heading)])
    if forward <= 0 or abs(lateral) > forward or distance > MAX_RANGE:
        return False
    samples = max(1, int(np.ceil(distance / (cell*.25))))
    points = np.asarray(origin) + np.arange(1, samples+1)[:, None]/samples * delta
    grid = np.floor(points/cell).astype(int)
    h, w = occ.shape
    if np.any((grid < 0) | (grid >= [w, h])):
        return False
    return not np.any(occ[grid[:, 1], grid[:, 0]] == 1)


def encoded_colors(vertices, strength):
    """Reserve saturated magenta for depth-tested heat surfaces before the screen pass."""
    vertices = np.asarray(vertices)
    texture = .035*np.sin(vertices[:, 0]*8 + vertices[:, 1]*5)
    signal = np.clip(strength + texture, 0, .94)
    return np.column_stack([np.ones(len(vertices)), signal, np.ones(len(vertices))])*255


def screen_pass(rgb, time=0.0):
    rgb = np.asarray(rgb)/255.0
    heat = (rgb[..., 0] > .98) & (rgb[..., 2] > .98) & (rgb[..., 1] < .95)
    value = np.where(heat, rgb[..., 1], .045 + .18*(rgb @ [.2126, .7152, .0722]))
    yy, xx = np.indices(value.shape, dtype=np.uint32)
    tick = np.uint32((int(np.floor(time*5))*83492791) & 0xffffffff)
    noise = ((xx+1)*np.uint32(73856093)) ^ ((yy+1)*np.uint32(19349663)) ^ tick
    noise = ((noise ^ (noise >> 13))*np.uint32(1274126177)) & np.uint32(0x7fffffff)
    grain = (noise/2147483647.0-.5)*.012
    value = np.clip(value+grain, 0, 1)
    scaled = value*(len(PALETTE)-1)
    index = np.minimum(scaled.astype(int), len(PALETTE)-2)
    fraction = (scaled-index)[..., None]
    return (PALETTE[index]*(1-fraction) + PALETTE[index+1]*fraction)*255


def draw_sources(renderer, world, cam, img, zbuf, unit):
    from .burial import SURFACE_LIFT, body_arrays, needs_excavation, rubble_parts
    from .render3d import raster_triangles

    for victim in world.victims:
        strength = intensity(*victim.pos, victim.state, victim.buried, world.t)
        if not strength or not in_sensor(world.occ, world.cell, world.pos[unit],
                                         world.theta[unit], victim.pos):
            continue
        x, y = victim.pos
        yaw = (x*1.7+y*3.1) % (2*np.pi)
        rotation, origin = renderer.surface.robot_pose(x, y, yaw,
                                                       stride=renderer._stride_at(x, y))
        if needs_excavation(victim.state, victim.buried):
            geometry = [(part.vertices, part.triangles) for part in rubble_parts()]
        else:
            vertices, triangles, _ = body_arrays()
            geometry = [(vertices + [0, 0, SURFACE_LIFT], triangles)]
        for vertices, triangles in geometry:
            raster_triangles(img, zbuf, cam, vertices @ rotation.T + origin, triangles,
                             encoded_colors(vertices, strength), shaded=False, two_sided=True)
