"""
MicroMVP Coordinator Module.

Provides coordinator interfaces for orchestrating robot control:
- Coordinator: Abstract base class
- KeyboardCoordinator: Manual keyboard control with robot selection
"""

from .base import Coordinator
from .keyboard_coordinator import KeyboardCoordinator

__all__ = ["Coordinator", "KeyboardCoordinator"]
