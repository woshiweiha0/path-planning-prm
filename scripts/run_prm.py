import os
import time
import matplotlib.pyplot as plt

from src.environment_2d import Environment2D
from src.prm import prm_plan


def main():
    seed = 4
    env = Environment2D(width = 10, height = 6, n_obs = 5, seed = seed)

    # sample two free points
    start = env.sample_free()
    goal = env.sample_free()
    if start is None or goal is None:
        print("Failed to sample start/goal.")
        return

    t0 = time.perf_counter()
    res = prm_plan(env, start, goal, n_samples = 250, k = 12, step = 0.05)
    dt = time.perf_counter() - t0

    # plot
    ax = env.plot()

    # draw edges lightly
    nodes = res.nodes
    for i, nbrs in res.edges.items():
        xi, yi = nodes[i]
        for j in nbrs:
            # draw each undirected edge once
            if j < i:
                continue
            xj, yj = nodes[j]
            ax.plot([xi, xj], [yi, yj], linewidth = 0.6, alpha = 0.4)

    # draw nodes
    xs = [p[0] for p in nodes]
    ys = [p[1] for p in nodes]
    ax.scatter(xs, ys, s=8)

    # draw start/goal
    ax.scatter([start[0]], [start[1]], s = 80, marker = "s")  # start square
    ax.scatter([goal[0]], [goal[1]], s = 100, marker = "*")  # goal star

    # draw path if found
    if res.path is not None:
        px = [p[0] for p in res.path]
        py = [p[1] for p in res.path]
        ax.plot(px, py, linewidth=2.5)
        print(f"PRM found path length = {res.path_length:.3f}")
    else:
        print("PRM did not find a path.")

    # stats
    # count edges (undirected count)
    edge_count = sum(len(v) for v in res.edges.values()) // 2
    print(f"nodes = {len(res.nodes)}, edges = {edge_count}, runtime = {dt * 1000:.1f} ms")

    os.makedirs("outputs/plots", exist_ok=True)
    out = f"outputs/plots/prm_seed{seed}.png"
    plt.savefig(out, dpi = 200)
    print(f"Saved plot to: {out}")


if __name__ == "__main__":
    main()
