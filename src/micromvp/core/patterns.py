from __future__ import annotations

import math
from typing import List, Tuple

from micromvp.utils.config import Boundary


Point = Tuple[float, float]


def circle_pattern(num: int, bound: Boundary) -> List[List[Point]]:
    paths: List[List[Point]] = [[] for _ in range(num)]
    radius = 0.8*bound.height / 2
    center = ((bound.left + bound.right) / 2, (bound.top + bound.bottom) / 2)
    step1 = 2 * math.pi / num
    step2 = 2 * math.pi / 50
    for j in range(num):
        angle = j * step1
        x = center[0] + math.cos(angle) * radius
        y = center[1] + math.sin(angle) * radius
        paths[j].append((x, y))
        for _ in range(150):
            angle += step2
            x = center[0] + math.cos(angle) * radius
            y = center[1] + math.sin(angle) * radius
            paths[j].append((x, y))
    return paths


def figure8_pattern(num: int, bound: Boundary) -> List[List[Point]]:
    figure8_path: List[Point] = []
    center = ((bound.left + bound.right) / 2, (bound.top + bound.bottom) / 2)
    radius = 1.5 * bound.height / 2
    theta = math.pi / 2
    num_step = 1000
    step = 2 * math.pi / num_step
    for _ in range(num_step):
        theta_flip = theta if (theta > 3 * math.pi / 2 or theta < math.pi / 2) else 2 * math.pi - theta
        r = radius * math.cos(theta_flip) ** 2
        point = (r * math.cos(theta_flip) + center[0], r * math.sin(theta_flip) + center[1])
        figure8_path.append(point)
        theta = (theta + step) % (2 * math.pi)
    arc_path = _arc_length_split(figure8_path, 100)
    return _make_pattern(arc_path, num if num % 2 else num + 1)


def _arc_length_split(path: List[Point], num_steps: int) -> List[Point]:
    distances = [0.0]
    for p, q in zip(path[:-1], path[1:]):
        distances.append(math.hypot(p[0] - q[0], p[1] - q[1]))
    length = sum(distances)
    step_length = length / num_steps
    new_path = [path[0]]
    next_dist = step_length
    d = 0.0
    for i, delta in enumerate(distances):
        d += delta
        if d > next_dist:
            new_path.append(path[i])
            next_dist += step_length
    return new_path


def _make_pattern(path: List[Point], num_cars: int) -> List[List[Point]]:
    car_index = int(len(path) / num_cars)
    paths: List[List[Point]] = []
    for i in range(num_cars):
        index = i * car_index
        paths.append(path[index:] + path[:index])
    return paths
