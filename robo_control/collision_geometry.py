"""Exact segment/axis-aligned rectangle distance for bounded field-mm geometry.

Runtime validates finite inputs before this numerical hot path. A rectangle is
its four OUTER bounds, not an object identity or a physical safety certificate.
"""
from __future__ import annotations


def _intersects(start, end, bounds):
    low, high = 0., 1.
    for p, q, minimum, maximum in ((start[0], end[0], bounds[0], bounds[2]),
                                    (start[1], end[1], bounds[1], bounds[3])):
        delta = q-p
        if delta == 0:
            if p < minimum or p > maximum:
                return False
            continue
        near, far = (minimum-p)/delta, (maximum-p)/delta
        if near > far:
            near, far = far, near
        low, high = max(low, near), min(high, far)
        if low > high:
            return False
    return True


def segment_rectangle_distance_sq(start, end, bounds):
    """Euclidean distance squared; includes edge, corner and zero-length cases."""
    x0, y0, x1, y1 = bounds
    if _intersects(start, end, bounds):
        return 0.
    best = min(max(x0-p[0], 0., p[0]-x1)**2 + max(y0-p[1], 0., p[1]-y1)**2 for p in (start, end))
    dx, dy = end[0]-start[0], end[1]-start[1]
    length_sq = dx*dx + dy*dy
    if length_sq == 0:
        return best
    # If disjoint, two line segments achieve their minimum at an endpoint of
    # at least one segment. Start/end-to-rectangle above + corner-to-segment
    # below therefore covers all four edges without general polygon scans.
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        t = min(1., max(0., ((x-start[0])*dx + (y-start[1])*dy)/length_sq))
        px, py = start[0]+t*dx-x, start[1]+t*dy-y
        best = min(best, px*px+py*py)
    return best


def segment_near_rectangle(start, end, bounds, clearance):
    """Exact rounded-corner clearance, with a cheap conservative broad phase."""
    x0, y0, x1, y1 = bounds
    # Every Euclidean clearance hit lies within this axis-expanded rectangle.
    # Being inside it alone is NOT a hit: corners need the exact distance.
    tolerance = clearance + 1e-9
    if not _intersects(start, end, (x0-tolerance, y0-tolerance, x1+tolerance, y1+tolerance)):
        return False
    return segment_rectangle_distance_sq(start, end, bounds) <= tolerance*tolerance
