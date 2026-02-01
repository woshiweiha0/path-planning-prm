"""
Post-processing: path shortcutting

Given a collision-free path (list of (x,y)), try to shorten it by repeatedly
replacing a subpath with a straight-line shortcut if it is collision-free.
"""

from __future__ import annotations
from typing import List, Tuple, Optional
import numpy as np

from src.collision import segment_is_collision_free

Point = Tuple[float, float]


def path_length(path: List[Point]) -> float:
    """Compute total Euclidean length of a polyline path."""
    if len(path) < 2:
        return 0.0
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        total += float(np.hypot(x1 - x0, y1 - y0))
    return total


def shortcut_path(
    env,
    path: List[Point],
    maxrep: int = 500,
    segment_step: float = 0.05,
    seed: Optional[int] = None,
) -> List[Point]:
    """
    Repeatedly attempt shortcuts on a path.

    - pick indices i < j with j >= i + 2 (remove at least one waypoint)
    - if straight segment path[i] -> path[j] is collision-free:
        replace path[i:j+1] with [path[i], path[j]]
        i.e., new path becomes path[:i+1] + path[j:]
    """
    if path is None or len(path) <= 2:
        return path

    rng = np.random.default_rng(seed)
    out = list(path)

    for _ in range(maxrep):
        if len(out) <= 2:
            break

        # pick i < j with at least one point between them
        i = int(rng.integers(0, len(out) - 2))
        j = int(rng.integers(i + 2, len(out)))

        p_i = out[i]
        p_j = out[j]

        # try shortcut
        if segment_is_collision_free(env, p_i, p_j, step = segment_step):
            out = out[: i + 1] + out[j:]

    return out
