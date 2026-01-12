from __future__ import annotations

import math
from pathlib import Path
from typing import List

import pygame

from micromvp.core.models import WorldState
from micromvp.utils.config import Boundary, RGB_PATH_COLORS
from micromvp.utils.geometry import check_collision


class Renderer:
    def __init__(self, surface: pygame.Surface, boundary: Boundary, wheel_base: float):
        self._surface = surface
        self._boundary = boundary
        self._wheel_base = wheel_base
        self._font = pygame.font.SysFont("Arial", int(wheel_base))
        self._car_image = self._load_car_image()

    def _load_car_image(self) -> pygame.Surface:
        asset_path = Path(__file__).resolve().parents[3] / "assets" / "carImage.png"
        image = pygame.image.load(str(asset_path)).convert_alpha()
        scaled = pygame.transform.smoothscale(
            image,
            (
                int(self._wheel_base * 2.0 / (9.0 / 8.0)),
                int(self._wheel_base * 2.0),
            ),
        )
        return scaled

    def draw(self, world: WorldState, animate_goals: bool, time_s: float) -> None:
        self._surface.fill((255, 255, 255))
        pygame.draw.rect(
            self._surface,
            (128, 128, 128),
            pygame.Rect(
                self._boundary.left,
                self._boundary.top,
                self._boundary.width,
                self._boundary.height,
            ),
            1,
        )

        for idx, car in enumerate(world.cars):
            color = RGB_PATH_COLORS[idx % len(RGB_PATH_COLORS)]
            if len(car.path) > 1:
                pygame.draw.lines(self._surface, color, False, car.path, 3)
                if animate_goals:
                    goal_index = int(time_s * 5) % len(car.path)
                    goal = car.path[goal_index]
                else:
                    goal = car.path[-1]
                pygame.draw.circle(self._surface, color, (int(goal[0]), int(goal[1])), 5)
                text = self._font.render(f"Goal{car.car_id}", True, color)
                self._surface.blit(text, goal)

        for idx, car in enumerate(world.cars):
            color = RGB_PATH_COLORS[idx % len(RGB_PATH_COLORS)]
            rotated = pygame.transform.rotate(
                self._car_image,
                -180.0 * car.theta / math.pi - 90.0,
            )
            rect = rotated.get_rect(center=(car.x, car.y))
            self._surface.blit(rotated, rect.topleft)
            pygame.draw.circle(self._surface, (255, 0, 0), (int(car.x), int(car.y)), 4)
            text = self._font.render(str(car.car_id), True, color)
            self._surface.blit(text, (car.x - self._wheel_base / 2, car.y - self._wheel_base / 2))

        for i, car in enumerate(world.cars):
            for j, other in enumerate(world.cars):
                if i == j:
                    continue
                if check_collision(self._wheel_base * 1.5, car.x, car.y, other.x, other.y):
                    text = self._font.render("TOO CLOSE!", True, (0, 0, 0))
                    self._surface.blit(text, ((car.x + other.x) / 2, (car.y + other.y) / 2))
