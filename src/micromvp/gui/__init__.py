"""
MicroMVP GUI Module

Provides a PyQt6-based graphical interface for the MicroMVP robot control system.

Main Components:
- MVPWindow: Main application window with sidebar and canvas
- MVPCanvas: Workspace visualization with car rendering and drawing API
- Sidebar: Control panel and car inspector
- Widgets: Dynamic control widgets (Toggle, Slider, Options, Input, etc.)

Usage:
    from micromvp.gui import MVPWindow

    gui_config = {
        "canvas": {"draw_curve_callback": True},
        "control_panel": [
            {"type": "toggle", "label": "Enable", "callback_name": "toggle_enable"},
            {"type": "continuous_slider", "label": "Speed", "range": [0, 1], "callback_name": "set_speed"},
        ]
    }

    gui = MVPWindow(gui_config, workspace_config)
    gui.register_callback("toggle_enable", my_handler)
    gui.run()
"""

from .window import MVPWindow

__all__ = ["MVPWindow"]
