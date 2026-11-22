"""Convex sectors for hinge eyes with open, chamfered bores along world y."""

import numpy as np

SECTORS = 16
PIN_TAIL = -.0325
PIN_TIP = .0325
PIN_TAPER = .004
FACES = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
         (2, 6, 7, 3), (3, 7, 4, 0)]


def eye(length, inner_radius, outer_radius, offset_y=0.):
    angles = np.arange(SECTORS) * 2 * np.pi / SECTORS
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    apothem = np.cos(np.pi / SECTORS)
    outer = rays * outer_radius
    positions = np.array([-length / 2, -length / 2 + .0015, length / 2 - .0015, length / 2]) + offset_y
    radii = [inner_radius + .0015, inner_radius, inner_radius, inner_radius + .0015]
    pieces = []
    for index in range(3):
        left, right = (rays * (radius / apothem) for radius in radii[index:index + 2])
        for i in range(SECTORS):
            j = (i + 1) % SECTORS
            a = (left[i], left[j], outer[j], outer[i])
            b = (right[i], right[j], outer[j], outer[i])
            pieces.append(np.array([(-u, y, v) for y, face in zip(positions[index:index + 2], (a, b)) for u, v in face]))
    return tuple(pieces)


def pin_nose():
    """Convex sectors forming a flat-ended, 4 mm-long lead-in."""
    angles = np.arange(SECTORS) * 2 * np.pi / SECTORS
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    inner = .0029 * rays
    rings = [.005 * rays, .0042 * rays]
    pieces = []
    for i in range(SECTORS):
        j = (i + 1) % SECTORS
        faces = [(inner[i], inner[j], outer[j], outer[i]) for outer in rings]
        pieces.append(np.array([(-u, y, v) for y, face in zip((PIN_TIP - PIN_TAPER, PIN_TIP), faces) for u, v in face]))
    return tuple(pieces)
