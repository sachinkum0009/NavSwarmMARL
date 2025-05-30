#!/usr/bin/env python3

"""
Decentralized Multi-Agent Task Allocator

This module implements a decentralized task allocation algorithm for multi-agent systems.

Author: Sachin Kumar
Date: 2025/02/27

Description:
This script provides the functionality to allocate tasks among multiple agents in a decentralized manner. 
It is part of the NavSwarmMARL project, which focuses on navigation and task allocation for multi-agent reinforcement learning systems.

Usage:
To use this module, import it into your main application and call the appropriate functions to allocate tasks to agents.

"""

from nav_swarm_marl.datatypes.robot import Robot
from nav_swarm_marl.datatypes.task import Task
from typing import List


class DecentralizedMATA(object):
    """
    DecentralizedMATA is a class that implements a decentralized multi-agent task allocation (MATA) strategy. 
    It is designed to allocate tasks to robots in a distributed manner based on certain criteria, such as 
    minimizing the distance between a robot and a task.
    Methods:
        __init__():
            Initializes the DecentralizedMATA object.
        allocate_tasks(robots: List[Robot], task: Task) -> int:
            Allocates a task to the most suitable robot based on proximity and availability.
                robots (List[Robot]): A list of Robot objects representing the available robots.
                task (Task): A Task object representing the task to be allocated.
                int: The ID of the robot to which the task is allocated. Returns -1 if no suitable robot is found.
    """

    def __init__(self):
        pass

    def allocate_tasks(self, robots: List[Robot], task: Task) -> int:
        """
        Allocate tasks to robots in a decentralized manner.

        Args:
            robots (List[Robot]): List of robots to allocate tasks to.
            tasks (List[Task]): List of tasks to be allocated.

        Returns:
            int: task allocated to the robot with id
        """
        min_distance = float('inf')
        selected_robot_id = -1

        for robot in robots:
            if robot.status:
                continue
            distance = robot.get_distance_to(task.position)
            if distance < min_distance:
                min_distance = distance
                selected_robot_id = robot.id

        return selected_robot_id
