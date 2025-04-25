#!/usr/bin/env python3

from typing import Any, Dict, List, Optional, Tuple
import yaml

from scenario import BaseScenario, Scenario
from tabulate import tabulate


class MultiRobotScenario(BaseScenario):
    """
    A class to represent and manage multiple robot scenarios.
    This class provides functionality to read scenarios from a YAML file,
    execute them, and generate a report summarizing the results.
    Attributes:
        scenarios (list): A list of Scenario objects representing the loaded scenarios.
    Methods:
        __init__(file_path: str):
            Initializes the MultiRobotScenario with the given file path.
        read_scenario(file_path: str) -> bool:
            Reads and parses the scenario data from a YAML file.
        run_scenario() -> bool:
            Executes all the loaded scenarios and prints their names.
        print_report() -> None:
            Prints a tabulated report of the scenarios, including their points,
            reached points, total points, and delay.
    """

    def __init__(self, file_path):
        super().__init__(file_path=file_path)

    def read_scenario(self, file_path: str) -> bool:
        self.scenarios = []
        try:
            with open(file_path, "r") as f:
                data = yaml.safe_load(f)
                for scenario in data["scenarios"]:
                    self.scenarios.append(Scenario(**scenario))
        except Exception as e:
            print(f"Error reading scenario file: {e}")
            return False
        return True

    def run_scenario(self) -> bool:
        for scenario in self.scenarios:
            print(f"Running scenario: {scenario.name}")
        return True

    def increment_reached_points(self, index: int) -> None:
        if 0 <= index < len(self.scenarios):
            self.scenarios[index].reached_points += 1
        else:
            print(f"Index {index} is out of range for scenarios list.")
        

    def print_report(self) -> None:
        table_data = []
        for scenario in self.scenarios:
            table_data.append(
                [
                    scenario.name,
                    scenario.points,
                    scenario.reached_points,
                    scenario.total_points,
                    scenario.delay,
                ]
            )

        headers = ["Scenario Name", "Points", "Reached Points", "Total Points, Delay"]
        print(tabulate(table_data, headers=headers, tablefmt="grid"))
        print("Report printed successfully.")
