"""
MicroMVP Coordinator Module.

Provides coordinator interfaces for orchestrating robot control:
- Coordinator: Abstract base class
- KeyboardCoordinator: Manual keyboard control with robot selection
- FollowPathCoordinator: Path following with user-drawn curves
"""

from .base import Coordinator
from .keyboard_coordinator import KeyboardCoordinator
from .follow_path_coordinator import FollowPathCoordinator

__all__ = ["Coordinator", "KeyboardCoordinator", "FollowPathCoordinator"]
