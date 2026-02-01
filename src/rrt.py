"""
RRT (Rapidly-exoring Random Tree) planner
"""

from __future__ import annotations

import math
import numpy as np

from src.collision import segment_is_collision_free


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _steer(q_near, q_rand, step_size: float):
    """Move from q_near toward q_rand by at most step_size."""
    dx = q_rand[0] - q_near[0]
    dy = q_rand[1] - q_near[1]
    d = math.hypot(dx, dy)
    if d <= step_size:
        return (q_rand[0], q_rand[1])
    s = step_size / d
    return (q_near[0] + s * dx, q_near[1] + s * dy)


def _backtrack(nodes, parent, goal_idx: int):
    path = []
    i = goal_idx
    while i != -1:
        path.append(nodes[i])
        i = parent[i]
    path.reverse()
    return path


def rrt_plan(
    env,
    q_start,
    q_goal,
    step_size = 0.3,
    max_iters = 5000,
    goal_radius = 0.3,
    seed = None,
    goal_bias = 0.05,
    segment_step = 0.05,
):
    """
    Returns: dict with
      - path: list[(x,y)] or None
      - nodes: list[(x,y)]
      - parent: list[int]
    """
    rng = np.random.default_rng(seed)

    # basic sanity: start/goal should be in free space
    if env.check_collision(q_start[0], q_start[1]) or env.check_collision(q_goal[0], q_goal[1]):
        return {"path": None, "nodes": [], "parent": []}

    nodes = [tuple(q_start)]
    parent = [-1]

    for _ in range(max_iters):
        # 1) sample random configuration (with small goal bias)
        if rng.random() < goal_bias:
            q_rand = tuple(q_goal)
        else:
            q = env.sample_free()
            if q is None:
                continue
            q_rand = tuple(q)

        # 2) nearest neighbor
        nearest_i = min(range(len(nodes)), key=lambda i: _dist(nodes[i], q_rand))
        q_near = nodes[nearest_i]

        # 3) steer
        q_new = _steer(q_near, q_rand, step_size)

        # reject if point is in collision
        if env.check_collision(q_new[0], q_new[1]):
            continue

        # 4) edge collision check
        if not segment_is_collision_free(env, q_near, q_new, step = segment_step):
            continue

        # add node
        nodes.append(q_new)
        parent.append(nearest_i)
        new_i = len(nodes) - 1

        # 5) goal check (either close enough OR can connect directly)
        if _dist(q_new, q_goal) <= goal_radius and segment_is_collision_free(env, q_new, q_goal, step = segment_step):
            # add goal as final node for clean backtrack
            nodes.append(tuple(q_goal))
            parent.append(new_i)
            goal_i = len(nodes) - 1
            path = _backtrack(nodes, parent, goal_i)
            return {"path": path, "nodes": nodes, "parent": parent}

    return {"path": None, "nodes": nodes, "parent": parent}
