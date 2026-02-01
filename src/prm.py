"""
PRM (Probabilistic Roadmap) planner
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math
import heapq

Point = Tuple[float, float]


@dataclass
class PRMResult:
    nodes: List[Point]                 # includes sampled nodes (not necessarily start/goal)
    edges: Dict[int, List[int]]        # adjacency list over nodes indices
    path: Optional[List[Point]]        # actual coordinate path (includes start/goal if found)
    path_length: Optional[float]


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dijkstra(nodes: List[Point], adj: Dict[int, List[int]], s: int, t: int) -> Optional[List[int]]:
    # Return list of node indices from s to t (inclusive), or None.
    INF = float("inf")
    n = len(nodes)
    d = [INF] * n
    parent = [-1] * n
    d[s] = 0.0
    pq = [(0.0, s)]

    while pq:
        curd, u = heapq.heappop(pq)
        if curd != d[u]:
            continue
        if u == t:
            break
        for v in adj.get(u, []):
            w = dist(nodes[u], nodes[v])
            nd = curd + w
            if nd < d[v]:
                d[v] = nd
                parent[v] = u
                heapq.heappush(pq, (nd, v))

    if d[t] == INF:
        return None

    # reconstruct
    path = []
    x = t
    while x != -1:
        path.append(x)
        x = parent[x]
    path.reverse()
    return path


def prm_plan(
    env,
    start: Point,
    goal: Point,
    n_samples: int = 200,
    k: int = 10,
    step: float = 0.05,
):
    """
    env provide:
      - env.sample_free() -> (x,y) or None
      - env.check_collision(x,y) -> True if in obstacle else False
    collision check:
      - env.segment_is_collision_free(p1,p2, step) OR use function from src.collision
    """

    # sample nodes in C_free
    nodes: List[Point] = []
    tries = 0
    max_tries = n_samples * 50

    while len(nodes) < n_samples and tries < max_tries:
        tries += 1
        p = env.sample_free()
        if p is None:
            continue
        nodes.append(p)

    # Append start/goal as extra nodes at the end
    start_idx = len(nodes)
    nodes.append(start)
    goal_idx = len(nodes)
    nodes.append(goal)

    # build adjacency list
    adj: Dict[int, List[int]] = {i: [] for i in range(len(nodes))}

    # helper: pick k nearest neighbors (brute force)
    def k_nearest(i: int, kk: int) -> List[int]:
        dists = []
        for j in range(len(nodes)):
            if j == i:
                continue
            dists.append((dist(nodes[i], nodes[j]), j))
        dists.sort(key=lambda x: x[0])
        return [j for _, j in dists[:kk]]

    # collision-free edge test
    if hasattr(env, "segment_is_collision_free"):
        edge_ok = lambda a, b: env.segment_is_collision_free(a, b, step=step)
    else:
        # fallback: use env.check_collision sampling
        def edge_ok(a: Point, b: Point) -> bool:
            length = dist(a, b)
            if length == 0:
                return True
            n = max(2, int(length / step))
            for t in range(n + 1):
                u = t / n
                x = a[0] + u * (b[0] - a[0])
                y = a[1] + u * (b[1] - a[1])
                if env.check_collision(x, y):
                    return False
            return True

    # connect all nodes using k-nearest
    for i in range(len(nodes)):
        for j in k_nearest(i, k):
            if j in adj[i]:
                continue
            if edge_ok(nodes[i], nodes[j]):
                adj[i].append(j)
                adj[j].append(i)

    # shortest path on roadmap
    idx_path = dijkstra(nodes, adj, start_idx, goal_idx)
    if idx_path is None:
        return PRMResult(nodes = nodes, edges = adj, path = None, path_length = None)

    coord_path = [nodes[i] for i in idx_path]

    # compute length
    total = 0.0
    for a, b in zip(coord_path, coord_path[1:]):
        total += dist(a, b)

    return PRMResult(nodes = nodes, edges = adj, path = coord_path, path_length = total)

