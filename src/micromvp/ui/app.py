from __future__ import annotations

import time
from typing import Optional

import pygame
import pygame_gui

from micromvp.core.controller import Command, Controller
from micromvp.core.patterns import circle_pattern, figure8_pattern
from micromvp.core.planner import refine_paths, shuffle_paths
from micromvp.utils.config import AppConfig
from micromvp.ui.renderer import Renderer


class App:
    def __init__(self, config: AppConfig, controller: Controller) -> None:
        pygame.init()
        self._config = config
        self._controller = controller
        self._size = config.container_size
        self._screen = pygame.display.set_mode(self._size)
        pygame.display.set_caption("microMVP")

        self._ui = pygame_gui.UIManager(self._size)
        self._clock = pygame.time.Clock()
        self._running = True

        self._canvas_offset = (140 + config.spacer, config.spacer)
        self._canvas_rect = pygame.Rect(
            self._canvas_offset[0],
            self._canvas_offset[1],
            config.painter_size[0],
            config.painter_size[1],
        )
        self._canvas_surface = pygame.Surface(config.painter_size)

        self._renderer = Renderer(self._canvas_surface, config.boundary(), config.wheel_base)
        self._recording = False
        self._animate_goals = False

        self._build_ui()

    def _build_ui(self) -> None:
        panel = pygame.Rect(0, 0, 140, self._size[1])
        self._run_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(20, 20, 100, 30),
            text="Run",
            manager=self._ui,
            container=None,
        )
        self._stop_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(20, 60, 100, 30),
            text="Stop",
            manager=self._ui,
        )
        self._clear_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(20, 100, 100, 30),
            text="Clear",
            manager=self._ui,
        )

        pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, 150, 120, 20),
            text="Speed",
            manager=self._ui,
        )
        self._speed_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pygame.Rect(10, 175, 120, 20),
            start_value=50,
            value_range=(0, 50),
            manager=self._ui,
        )

        pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, 210, 120, 20),
            text="Car",
            manager=self._ui,
        )
        car_options = [str(car_id) for car_id, _ in self._config.car_info]
        self._car_select = pygame_gui.elements.UIDropDownMenu(
            options_list=car_options,
            starting_option=car_options[0],
            relative_rect=pygame.Rect(10, 235, 120, 25),
            manager=self._ui,
        )

        pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(10, 270, 120, 20),
            text="Pattern",
            manager=self._ui,
        )
        self._pattern_select = pygame_gui.elements.UIDropDownMenu(
            options_list=["circle", "figure8"],
            starting_option="circle",
            relative_rect=pygame.Rect(10, 295, 120, 25),
            manager=self._ui,
        )
        self._pattern_button = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(20, 330, 100, 30),
            text="Apply",
            manager=self._ui,
        )

    def run(self) -> None:
        while self._running:
            time_delta = self._clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1 and self._canvas_rect.collidepoint(event.pos):
                        self._recording = True
                if event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self._recording = False
                if event.type == pygame_gui.UI_BUTTON_PRESSED:
                    if event.ui_element == self._run_button:
                        self._controller.enqueue(Command("set_speed_scale", 1.0))
                    if event.ui_element == self._stop_button:
                        self._controller.enqueue(Command("set_speed_scale", 0.0))
                    if event.ui_element == self._clear_button:
                        self._controller.enqueue(Command("clear_paths"))
                    if event.ui_element == self._pattern_button:
                        self._apply_pattern()
                if event.type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                    if event.ui_element == self._speed_slider:
                        self._controller.enqueue(Command("set_speed_scale", event.value / 50.0))

                self._ui.process_events(event)

            if self._recording:
                mouse_pos = pygame.mouse.get_pos()
                if self._canvas_rect.collidepoint(mouse_pos):
                    car_id = int(self._car_select.selected_option)
                    point = (
                        mouse_pos[0] - self._canvas_offset[0],
                        mouse_pos[1] - self._canvas_offset[1],
                    )
                    self._controller.enqueue(Command("add_point", (car_id, point)))
                    self._animate_goals = False

            world = self._controller.snapshot()
            self._renderer.draw(world, self._animate_goals, pygame.time.get_ticks() / 1000.0)
            self._screen.blit(self._canvas_surface, self._canvas_offset)
            self._ui.update(time_delta)
            self._ui.draw_ui(self._screen)
            pygame.display.flip()

        pygame.quit()

    def _apply_pattern(self) -> None:
        world = self._controller.snapshot()
        locs = [(car.x, car.y) for car in world.cars]
        bound = self._config.boundary()
        if self._pattern_select.selected_option == "figure8":
            paths = figure8_pattern(len(world.cars), bound)
        else:
            paths = circle_pattern(len(world.cars), bound)
        paths = shuffle_paths(locs, paths)
        paths = refine_paths(paths)
        self._controller.enqueue(Command("set_paths", paths))
        self._animate_goals = True
