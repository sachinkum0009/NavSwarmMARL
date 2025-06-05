#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from abc import ABC, abstractmethod


@dataclass
class Scenario:
    name: str
    points: List[Tuple[float, float]]
    distance_covered: List[float]
    reached_points: int = 0
    total_points: int = 0
    delay: Optional[float] = None

    def __post_init__(self):
        self.total_points = len(self.points)
        # self.distance_covered = [0.0] * len(self.points)


class BaseScenario(ABC):
    """
    Base class for scenarios
    """

    def __init__(self, file_path: str):
        """
        Initialize the scenario
        """
        self.scenarios: List[Scenario] = []
        self.read_scenario(file_path)

    @abstractmethod
    def update_distance(self, index: int, distance: float) -> bool:
        """
        Update the distance covered by a robot at a given
        index in the list of scenarios
        :param
        index: Index of the robot in the list of scenarios
        distance: Distance covered by the robot at the given index in the list of scenarios
        :return: True if the distance was updated successfully,
        False otherwise
        """
        pass

    @abstractmethod
    def read_scenario(self, file_path: str) -> bool:
        """
        Read the scenario from a file
        :param file_path: Path to the scenario file
        :return: True if the scenario was read successfully, False otherwise
        """
        pass

    @abstractmethod
    def increment_reached_points(self, index: int) -> None:
        """
        Increment the number of reached points
        :return: None
        """
        pass

    @abstractmethod
    def run_scenario(self) -> bool:
        """
        Run the scenario
        :return: True if the scenario was run successfully, False otherwise
        """
        pass

    @abstractmethod
    def print_report(self) -> None:
        """
        Print the report of the scenario
        :return: None
        """
        pass
