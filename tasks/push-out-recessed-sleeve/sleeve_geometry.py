"""Convex radial pieces for a real through-bore and a free annular sleeve."""

import numpy as np

FACETS = 32
# Each prism starts with the four near-face vertices, clockwise in the yz plane.
FACES = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
         (2, 6, 7, 3), (3, 7, 4, 0)]


def annulus(length, inner_radius, *, outer_radius=None, outer_half=None):
    """Extrude wedges along x. The specified inner radius is the polygon's apothem."""
    angles = np.arange(FACETS) * (2 * np.pi / FACETS)
    rays = np.stack((np.cos(angles), np.sin(angles)), axis=1)
    inner = rays * (inner_radius / np.cos(np.pi / FACETS))
    if outer_half is None:
        outer = rays * outer_radius
    else:
        outer = rays / np.max(np.abs(rays) / outer_half, axis=1, keepdims=True)
    pieces = []
    for i in range(FACETS):
        j = (i + 1) % FACETS
        face = [inner[i], inner[j], outer[j], outer[i]]
        pieces.append(np.array([(x, y, z) for x in (-length / 2, length / 2) for y, z in face]))
    return tuple(pieces)
