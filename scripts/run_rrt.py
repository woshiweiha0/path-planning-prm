import os
import time
import matplotlib.pyplot as plt

from src.environment_2d import Environment2D
from src.rrt import rrt_plan


def main():
    seed = 2
    env = Environment2D(width = 10, height = 6, n_obs = 5, seed = seed)

    start = env.sample_free()
    goal = env.sample_free()
    if start is None or goal is None:
        print("Failed to sample start/goal.")
        return

    t0 = time.perf_counter()
    res = rrt_plan(
        env,
        start,
        goal,
        step_size = 0.35,
        max_iters = 6000,
        goal_radius = 0.35,
        seed = seed,
        goal_bias = 0.07,
        segment_step = 0.05,
    )
    dt = time.perf_counter() - t0

    # plot
    ax = env.plot()

    nodes = res["nodes"]
    parent = res["parent"]

    # draw tree edges lightly
    for i in range(1, len(nodes)):
        p = nodes[parent[i]]
        q = nodes[i]
        ax.plot([p[0], q[0]], [p[1], q[1]], linewidth = 0.6)

    # start/goal markers
    ax.scatter([start[0]], [start[1]], s = 60, marker = "s")
    ax.scatter([goal[0]], [goal[1]], s = 80, marker = "*")

    path = res["path"]
    if path is None:
        print(f"RRT failed, nodes expanded = {len(nodes)}, runtime = {dt * 1000:.1f} ms")
    else:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, linewidth = 3)
        print(f"RRT found path length = {len(path)}")
        print(f"nodes expanded = {len(nodes)}, runtime = {dt * 1000:.1f} ms")

    os.makedirs("outputs/plots", exist_ok = True)
    outpath = f"outputs/plots/rrt_seed{seed}.png"
    plt.savefig(outpath, dpi = 200)
    print(f"Saved plot to: {outpath}")


if __name__ == "__main__":
    main()
