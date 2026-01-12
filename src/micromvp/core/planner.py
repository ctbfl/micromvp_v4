#输出的是：“世界坐标系下的目标点/轨迹”
from __future__ import annotations

import math
import random
from typing import List, Sequence, Tuple

from munkres import Munkres

from micromvp.utils.config import Boundary
from micromvp.utils.geometry import distance, check_collision


Point = Tuple[float, float]


def shuffle_paths(locs: Sequence[Point], paths: Sequence[List[Point]]) -> List[List[Point]]:
    matrix: List[List[float]] = []
    for loc in locs:
        matrix.append([])
        for path in paths:
            matrix[-1].append(distance(loc[0], loc[1], path[0][0], path[0][1]))
    indexes = Munkres().compute(matrix)
    new_paths: List[List[Point]] = []
    for row, column in indexes:
        new_paths.append(paths[column])
    return new_paths


def refine_paths(paths: Sequence[List[Point]]) -> List[List[Point]]:
    length = max((len(path) for path in paths), default=0)
    paths = [list(path) for path in paths]
    for path in paths:
        while len(path) < length:
            path.append(path[-1])

    total = 0.0
    num = 0
    for path in paths:
        for i in range(len(path) - 1):
            if path[i] != path[i + 1]:
                total += distance(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1])
                num += 1
    if num == 0:
        return paths

    pts = (int(total / num) + 1) // 4
    if pts == 0:
        return paths

    new_paths: List[List[Point]] = [[] for _ in paths]
    for index, path in enumerate(paths):
        for i in range(len(path) - 1):
            new_paths[index].append(path[i])
            step_x = (path[i + 1][0] - path[i][0]) / pts
            step_y = (path[i + 1][1] - path[i][1]) / pts
            for _ in range(pts):
                last = new_paths[index][-1]
                new_paths[index].append((last[0] + step_x, last[1] + step_y))
        new_paths[index].append(path[-1])
    return new_paths


def random_goals(num: int, bound: Boundary, min_sep: float) -> List[Point]:
    goals: List[Point] = []
    while len(goals) < num:
        new_x = bound.width * random.random() + bound.left
        new_y = bound.height * random.random() + bound.top
        if _no_collision(goals, new_x, new_y, min_sep):
            goals.append((new_x, new_y))
    return goals


def random_arrangement(num: int, bound: Boundary, min_sep: float) -> List[Tuple[float, float, float]]:
    starts: List[Tuple[float, float, float]] = []
    while len(starts) < num:
        new_x = bound.width * random.random() + bound.left
        new_y = bound.height * random.random() + bound.top
        if _no_collision([(x, y) for x, y, _ in starts], new_x, new_y, min_sep):
            starts.append((new_x, new_y, random.random() * math.pi))
    return starts


def _no_collision(points: Sequence[Point], x: float, y: float, min_sep: float) -> bool:
    for obj in points:
        if check_collision(min_sep, x, y, obj[0], obj[1]):
            return False
    return True
