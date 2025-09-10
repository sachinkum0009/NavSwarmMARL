#!/usr/bin/env python3

"""
PID controller which will be used for cmd_vel
"""

import numpy as np
from dataclasses import dataclass

from typing import Tuple

@dataclass
class PidParams:
    """
    Data class representing the parameters for a PID (Proportional-Integral-Derivative) controller.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        setpoint (float, optional): Desired target value for the controller. Defaults to 0.0.
    """

    kp: float
    ki: float
    kd: float
    setpoint: float = 0.0


class PidController:
    """
    A simple PID (Proportional-Integral-Derivative) controller class.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        setpoint (float): Desired target value for the controller.
        prev_error (float): Error value from the previous update step.
        integral (float): Accumulated integral of the error.

    Methods:
        update(measured_value: float, dt: float) -> float:
            Calculates the PID control output based on the measured value and time step.

        setpoint (property):
            Gets or sets the setpoint for the controller.
    """

    def __init__(self, params: PidParams):
        self.kp = params.kp
        self.ki = params.ki
        self.kd = params.kd
        self._setpoint = params.setpoint
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, measured_value: float, dt: float) -> float:
        error = self._setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    @property
    def setpoint(self) -> float:
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value: float) -> None:
        self._setpoint = value

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0


class TwistPidController:
    """
    A controller that manages separate PID controllers for linear and angular velocities.

    Args:
        l_pid_param (PidParams): Parameters for the linear velocity PID controller.
        a_pid_param (PidParams): Parameters for the angular velocity PID controller.

    Methods:
        update(l_vel: float, a_vel: float, dt: float) -> tuple[float, float]:
            Updates both linear and angular PID controllers with the given velocities and timestep.
            Returns a tuple containing the outputs of the linear and angular PID controllers.
    """

    def __init__(self, l_pid_param: PidParams, a_pid_param: PidParams):
        self._linear_pid = PidController(l_pid_param)
        self._angular_pid = PidController(a_pid_param)

    def update(self, l_vel: float, a_vel: float, dt: float) -> Tuple[float, float]:
        l_out = self._linear_pid.update(l_vel, dt)
        a_out = self._angular_pid.update(a_vel, dt)
        return l_out, a_out

    @property
    def linear_setpoint(self) -> float:
        return self._linear_pid.setpoint

    @linear_setpoint.setter
    def linear_setpoint(self, value: float) -> None:
        self._linear_pid.setpoint = value

    @property
    def angular_setpoint(self) -> float:
        return self._angular_pid.setpoint

    @angular_setpoint.setter
    def angular_setpoint(self, value: float) -> None:
        self._angular_pid.setpoint = value
