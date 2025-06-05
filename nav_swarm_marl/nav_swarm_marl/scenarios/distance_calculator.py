#!/usr/bin/env python3

from dataclasses import dataclass
import math
from typing import List


@dataclass
class DistanceCalculator:
    distances: List[float]
    prev_x: List[float]
    prev_y: List[float]

    def __init__(self, number: int):
        self.distances = [0.0] * number
        self.prev_x = [0.0] * number
        self.prev_y = [0.0] * number

    def update_distance(self, index: int, x: float, y: float):
        dx = x - self.prev_x[index]
        dy = y - self.prev_y[index]

        distance = math.sqrt(dx**2 + dy**2)
        self.distances[index] += distance
        self.prev_x[index] = x
        self.prev_y[index] = y

    def get_distance(self, index: int):
        return self.distances[index]

    def reset_distance(self, index: int):
        self.distances[index] = 0.0
