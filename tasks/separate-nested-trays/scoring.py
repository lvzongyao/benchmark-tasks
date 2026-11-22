"""Episode-local scoring for independent, released tray placement."""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class Inputs:
    invalid: bool = False
    unlocked: bool = False
    separate: bool = False
    upright: bool = False
    staged: bool = False
    clear: bool = False
    poses: tuple = ()


@dataclass
class Dwell:
    step: int = -1
    invalid: bool = False
    history: list = field(default_factory=list)
    result: float = 0.0


def stable(previous, current):
    for i in (0, 2):
        if np.linalg.norm(previous[i] - current[i]) > .002:
            return False
        cosine = (np.trace(previous[i + 1].T @ current[i + 1]) - 1) / 2
        if cosine < math.cos(math.radians(2)):
            return False
    return True


def score(inputs, dwell, step):
    """Read the final tier once per control step, including continuous release dwell."""
    if step == dwell.step:
        return dwell.result
    if step != dwell.step + 1:
        dwell.history.clear()
    dwell.step = step
    dwell.invalid |= inputs.invalid
    full_candidate = inputs.separate and inputs.upright and inputs.staged and inputs.clear
    if dwell.invalid or not full_candidate:
        dwell.history.clear()
    else:
        while dwell.history and not all(stable(p, inputs.poses) for p in dwell.history):
            dwell.history.pop(0)
        dwell.history.append(inputs.poses)
        dwell.history = dwell.history[-20:]
    if dwell.invalid:
        dwell.result = 0.0
    elif len(dwell.history) == 20:
        dwell.result = 1.0
    elif inputs.separate and inputs.upright:
        dwell.result = .75
    elif inputs.separate:
        dwell.result = .50
    elif inputs.unlocked:
        dwell.result = .25
    else:
        dwell.result = 0.0
    return dwell.result


def overlap_depth(tab, ledge, tab_rotation):
    """Minimum projection overlap on the separating axes of two rigid boxes."""
    world_axes = np.eye(3)
    tab_axes = tab_rotation.T
    crossed = np.cross(world_axes[:, None, :], tab_axes[None, :, :]).reshape(-1, 3)
    lengths = np.linalg.norm(crossed, axis=1)
    nonparallel = lengths > 1e-6
    axes = np.vstack((world_axes, tab_axes, crossed[nonparallel] / lengths[nonparallel, None]))
    a, b = tab @ axes.T, ledge @ axes.T
    overlap = np.minimum(a.max(axis=0), b.max(axis=0)) - np.maximum(a.min(axis=0), b.min(axis=0))
    return float(overlap.min())
