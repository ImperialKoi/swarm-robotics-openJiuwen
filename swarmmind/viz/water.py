"""Display-only river coordinates and a still of the dashboard water shader."""

import numpy as np


def river_frame(x, y, layout=None):
    """Metres along/across the closest authored channel, increasing downstream."""
    along, across = np.asarray(x) * .8 + np.asarray(y) * .6, -np.asarray(x) * .6 + np.asarray(y) * .8
    best = np.full(np.shape(x), np.inf)
    for channel in (layout or {}).get("channels", []):
        offset = 0.0
        points = channel["points"]
        for a, b in zip(points[:-1], points[1:], strict=True):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = np.hypot(dx, dy)
            if length < 1e-6:
                continue
            u = np.clip(((x-a[0])*dx + (y-a[1])*dy) / length**2, 0, 1)
            distance = (x-a[0]-u*dx)**2 + (y-a[1]-u*dy)**2
            closer = distance < best
            along = np.where(closer, offset+u*length, along)
            across = np.where(closer, ((y-a[1])*dx-(x-a[0])*dy)/length, across)
            best = np.minimum(best, distance)
            offset += length
    return np.stack([along, across], axis=-1)


def surface_color(frame, depth, time=0.0):
    """Same depth palette, travelling ripples and broken foam as water.gdshader."""
    s, t = frame[..., 0], frame[..., 1]
    depth = np.clip(depth, 0, 1)
    blend = depth * depth * (3 - 2 * depth)
    shallow, deep = np.array([.46, .57, .48]), np.array([.105, .31, .32])
    rgb = shallow + blend[..., None] * (deep-shallow)
    bend = np.sin(s*.17-time*.21) * .7 + np.sin(s*.071+t*.43) * .4
    ribbons = np.maximum(0, np.sin(t*1.9+bend))**10
    pulse = .5+.5*np.sin(s*.64-time*1.15+np.sin(t*.8))
    ripple = np.maximum(0, np.sin(s*1.7-time*2.1+bend))**12
    glint = ribbons*(.025+.075*pulse) + ripple*.035
    foam = (1-blend)**3 * ribbons * (.12+.16*pulse)
    rgb += glint[..., None]
    rgb += (np.array([.78, .83, .73])-rgb)*foam[..., None]
    return rgb
