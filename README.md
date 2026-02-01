# path-planning-prm

This repository implements the **Probabilistic Roadmap (PRM)** algorithm for solving a **2D motion planning problem with polygonal (triangular) obstacles**, following the standard sampling-based planning exercise.

The project demonstrates:
- Sampling collision-free configurations
- Building a roadmap in configuration space
- Connecting start and goal via graph search
- Visualizing the environment, roadmap, and final path

---

## Project Structure

path-planning-prm/
├── src/
│ ├── environment_2d.py # 2D environment + obstacle generation
│ ├── collision.py # Collision checking utilities
│ ├── prm.py # PRM implementation
│ ├── utils.py # Small helper functions (e.g. distance)
│ └── init.py
│
├── scripts/
│ ├── run_prm.py # Main script to run PRM and generate plots
│ ├── plot_env.py # Plot environment only (debug/visualization)
│ └── init.py
│
├── plots/
│ ├── env_debug.png # Environment visualization
│ └── prm_seed4.png # PRM result with roadmap and final path
│
├── outputs/
│ └── logs/ # (Optional) runtime or experiment logs
│
├── tests/
│ └── test_collision.py # Basic collision checking tests
│
├── requirements.txt
└── README.md


## How to Run

### 1. Install dependencies
pip install -r requirements.txt
2. Run PRM
python -m scripts.run_prm
