# This is the env corresponding to the 1 Spring St Setup.

# The workspace size is pre-determined.

from micromvp.env.base import Environment
from micromvp.core.models import Action, RobotObservation


class RealPushEnvironment(Environment):
    """ 
    The environment for 1 Spring St workspace.
    """
    def __init__(self):
        """
        It will initilize a observer process, and a sender.
        The observer process will continuously fetch the newest car information frame. (operate at it's maxinum speed, keep the latest frame in a buffer)
        """
        self.latest_observation = None
        self.observer
        pass

    def observe(self) -> Dict[int, RobotObservation]:
        """
        Get current observations for all robots.

        Returns:
            Dict mapping robot_id to RobotObservation.
            Not all robots may have valid observations (e.g., tracking lost).
        """
        raise NotImplementedError
    
    def apply_actions(self, actions: Dict[int, Action]) -> None:
        """
        Apply actions to robots.

        Args:
            actions: Dict mapping robot_id to Action.
                     Robots not in the dict will maintain previous action.
        """
        raise NotImplementedError
    

