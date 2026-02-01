from src.environment_2d import Environment2D
from src.prm import prm_plan

def main():
    env = Environment2D(seed = 0)
    q_start = (1.0, 1.0)
    q_goal = (9.0, 5.0)

    path = prm_plan(env, q_start, q_goal)
    print("PRM path:", path)

if __name__ == "__main__":
    main()
