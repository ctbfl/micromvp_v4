"""
Utility configuration classes for MicroMVP.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from micromvp.core.models import WorkspaceConfig


@dataclass
class Boundary:
    """
    Rectangular boundary definition for pattern generation.

    Coordinates use standard math convention:
    - Origin at bottom-left
    - X increases rightward
    - Y increases upward
    """
    left: float
    right: float
    bottom: float
    top: float

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def height(self) -> float:
        return self.top - self.bottom

    @property
    def center(self) -> tuple[float, float]:
        return ((self.left + self.right) / 2, (self.bottom + self.top) / 2)

    @classmethod
    def from_workspace_config(
        cls,
        ws_config: "WorkspaceConfig",
        margin_ratio: float = 0.1,
    ) -> "Boundary":
        """
        Create a boundary from WorkspaceConfig with optional margin.

        Args:
            ws_config: Workspace configuration
            margin_ratio: Margin as fraction of workspace size (default 0.1 = 10%)

        Returns:
            Boundary with margins applied
        """
        margin_x = ws_config.width * margin_ratio
        margin_y = ws_config.height * margin_ratio
        return cls(
            left=margin_x,
            right=ws_config.width - margin_x,
            bottom=margin_y,
            top=ws_config.height - margin_y,
        )
