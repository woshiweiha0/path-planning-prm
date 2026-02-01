"""
Utility helpers (distance, sampling, plotting, timing, etc.)
"""

import math

def dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
