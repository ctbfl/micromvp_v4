"""
MicroMVP Coordinator Module.

Provides coordinator interfaces for orchestrating robot control:
- Coordinator: Abstract base class
- KeyboardCoordinator: Manual keyboard control with robot selection
- FollowPathCoordinator: Path following with user-drawn curves
- FormationCoordinator: Multi-robot formation control with patterns
- NavigationCoordinator: Single robot navigation with web API and obstacle avoidance
"""

from .base import Coordinator
from .keyboard_coordinator import KeyboardCoordinator
from .follow_path_coordinator import FollowPathCoordinator
from .formation_coordinator import FormationCoordinator
from .navigation_coordinator import NavigationCoordinator

__all__ = [
    "Coordinator",
    "KeyboardCoordinator",
    "FollowPathCoordinator",
    "FormationCoordinator",
    "NavigationCoordinator",
]
