"""
2D environment with polygon (triangle) obstacles.
ToDo:
- random environment generation
- plotting helpers
- collision oracle via env.check_collision(x, y)
"""

class Environment2D:
    def __init__(self, width: float = 10.0, height: float = 6.0, n_obs: int = 5, seed: int | None = None):
        self.width = width
        self.height = height
        self.n_obs = n_obs
        self.seed = seed
        self.obstacles = []  # ToDo: list of triangles/polygons

    def check_collision(self, x: float, y: float) -> bool:
        """Return True if point (x,y) is inside any obstacle (collision)."""
        return False  #ToDo
