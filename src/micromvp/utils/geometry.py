import math


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x1 - x2, y1 - y2)


def check_collision(threshold: float, x1: float, y1: float, x2: float, y2: float) -> bool:
    return distance(x1, y1, x2, y2) <= threshold
