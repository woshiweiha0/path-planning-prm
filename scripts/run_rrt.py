from src.environment_2d import Environment2D
from src.rrt import rrt_plan

def main():
    env = Environment2D(seed = 0)
    q_start = (1.0, 1.0)
    q_goal = (9.0, 5.0)

    path = rrt_plan(env, q_start, q_goal)
    print("RRT path:", path)

if __name__ == "__main__":
    main()
