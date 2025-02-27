from dataclasses import dataclass
import numpy as np
from typing import List, Tuple

@dataclass
class Robot:
    id: int
    position: tuple
    status: bool

    def get_distance_to(self, position: Tuple) -> float:
        """
        Get the distance between the robot and a given position.

        Args:
            position (tuple): Position to calculate the distance to.

        Returns:
            float: Distance between the robot and the given position.
        """
        x1, y1 = self.position
        x2, y2 = position

        return np.linalg.norm(np.array([x2 - x1, y2 - y1]))
