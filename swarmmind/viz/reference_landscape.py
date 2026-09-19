"""Display dressing for an authored layout; never consumed by robot perception.

Godot reads an exported copy of the YAML layout, checked for equality in tests.
The existing wire height/water/occupancy remain the authority for geometry.
"""

import numpy as np


def biome_at(layout, x, y):
    for bx, by, width, depth in layout["buildings"]:
        if bx <= x <= bx+width and by <= y <= by+depth:
            return "ruin"
    for bx, by, rx, ry in layout["woodlands"]:
        if ((x-bx)/rx)**2 + ((y-by)/ry)**2 <= 1:
            return "forest"
    return "rock"


def dress_colors(colors, wx, wy, detail, wet, layout):
    """Metre-width gravel roads and flood sediment on the real terrain surface."""
    from ..sim.terrain import _segment_frame

    colors = colors.copy()
    for x, y, rx, ry in layout["woodlands"]:
        weight = np.clip((1-((wx-x)/rx)**2-((wy-y)/ry)**2)*3, 0, 1)
        tint = np.array([.19, .29, .18]) + detail[..., None]*.065
        colors += (tint-colors)*weight[..., None]*.85
    for x, y, rx, ry in layout["landslides"]:
        weight = np.clip((1-((wx-x)/rx)**2-((wy-y)/ry)**2)*5, 0, 1)
        tint = np.array([.51, .48, .41]) + (detail[..., None]-.5)*.08
        colors += (tint-colors)*weight[..., None]
    distance = np.full(wx.shape, np.inf)
    for path in layout["roads"]:
        for a, b in zip(path[:-1], path[1:], strict=True):
            d, _ = _segment_frame(wx, wy, np.asarray(a), np.asarray(b))
            distance = np.minimum(distance, d)
    road = np.clip(layout["road_width_m"]*.5+.6-distance, 0, 1)*(1-wet)
    gravel = np.array([.66, .63, .54]) + (detail[..., None]-.5)*.04
    return colors + (gravel-colors)*road[..., None]
