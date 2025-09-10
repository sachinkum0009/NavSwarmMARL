#!/usr/bin/env python3

"""
Optimized PID Controller implementation for robot navigation.

This module provides high-performance PID controllers specifically designed for
generating cmd_vel commands in ROS2 robotic navigation systems.

Author: NavSwarmMARL Team
"""

import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    from geometry_msgs.msg import Twist
except ImportError:
    # Mock Twist class for testing without ROS2
    class Twist:
        def __init__(self):
            self.linear = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            self.angular = type('obj', (object,), {'x': 0.0, 'y': 0.0, 'z': 0.0})()


@dataclass
class PidParams:
    """
    Immutable PID parameters container with validation.

    Provides efficient parameter storage and validation for PID controllers.
    """

    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -float('inf')
    output_max: float = float('inf')
    integral_min: float = -float('inf')
    integral_max: float = float('inf')

    def __post_init__(self) -> None:
        """Validate parameters after initialization."""
        if self.output_min >= self.output_max:
            raise ValueError("output_min must be less than output_max")
        if self.integral_min >= self.integral_max:
            raise ValueError("integral_min must be less than integral_max")

    @property
    def has_integral(self) -> bool:
        """Check if integral term is enabled."""
        return self.ki != 0.0

    @property
    def has_derivative(self) -> bool:
        """Check if derivative term is enabled."""
        return self.kd != 0.0


class PidController:
    """
    High-performance PID controller with optimized computations.

    Features:
    - Efficient computation with minimal object allocation
    - Configurable output and integral limits
    - Automatic derivative smoothing
    - Reset capability for discontinuous setpoints
    """

    __slots__ = (
        '_params', '_setpoint', '_last_error', '_integral',
        '_last_time', '_last_derivative', '_derivative_alpha'
    )

    def __init__(self, params: PidParams, derivative_alpha: float = 0.8) -> None:
        """
        Initialize PID controller.

        Args:
            params: PID parameters
            derivative_alpha: Derivative smoothing factor (0-1), higher = more smoothing
        """
        self._params = params
        self._setpoint = 0.0
        self._last_error: Optional[float] = None
        self._integral = 0.0
        self._last_time: Optional[float] = None
        self._last_derivative = 0.0
        self._derivative_alpha = max(0.0, min(1.0, derivative_alpha))

    @property
    def setpoint(self) -> float:
        """Current setpoint value."""
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value: float) -> None:
        """Set new setpoint and reset if it's a significant change."""
        if abs(value - self._setpoint) > 1e-6:
            self._setpoint = value
            # Reset on significant setpoint change to avoid derivative spikes
            self._last_error = None
            self._last_time = None

    @property
    def integral(self) -> float:
        """Current integral term value."""
        return self._integral

    def reset(self) -> None:
        """Reset controller state."""
        self._last_error = None
        self._integral = 0.0
        self._last_time = None
        self._last_derivative = 0.0

    def update(self, current_value: float, dt: Optional[float] = None) -> float:
        """
        Update PID controller and return control output.

        Args:
            current_value: Current process variable value
            dt: Time delta in seconds. If None, uses system time.

        Returns:
            Control output value
        """
        current_time = time.time()
        error = self._setpoint - current_value

        # Calculate time delta
        if dt is None:
            if self._last_time is None:
                dt = 0.0
            else:
                dt = current_time - self._last_time

        # Proportional term
        output = self._params.kp * error

        # Integral term (with anti-windup)
        if self._params.has_integral and dt > 0:
            integral_candidate = self._integral + error * dt
            # Clamp integral
            self._integral = max(
                self._params.integral_min,
                min(self._params.integral_max, integral_candidate)
            )
            output += self._params.ki * self._integral

        # Derivative term (with smoothing)
        if self._params.has_derivative and dt > 0 and self._last_error is not None:
            derivative = (error - self._last_error) / dt
            # Apply exponential smoothing
            self._last_derivative = (
                self._derivative_alpha * self._last_derivative +
                (1.0 - self._derivative_alpha) * derivative
            )
            output += self._params.kd * self._last_derivative

        # Store state for next iteration
        self._last_error = error
        self._last_time = current_time

        # Clamp output
        return max(self._params.output_min, min(self._params.output_max, output))


class TwistPidController:
    """
    Dual PID controller for generating Twist messages (cmd_vel commands).

    Manages separate PID controllers for linear and angular velocity
    with optimized Twist message generation.
    """

    __slots__ = ('_linear_pid', '_angular_pid', '_twist_msg')

    def __init__(
        self,
        linear_params: PidParams,
        angular_params: PidParams,
        derivative_alpha: float = 0.8
    ) -> None:
        """
        Initialize twist PID controller.

        Args:
            linear_params: PID parameters for linear velocity control
            angular_params: PID parameters for angular velocity control
            derivative_alpha: Derivative smoothing factor for both controllers
        """
        self._linear_pid = PidController(linear_params, derivative_alpha)
        self._angular_pid = PidController(angular_params, derivative_alpha)
        # Pre-allocate Twist message to avoid repeated allocations
        self._twist_msg = Twist()

    @property
    def linear_controller(self) -> PidController:
        """Access to linear velocity PID controller."""
        return self._linear_pid

    @property
    def angular_controller(self) -> PidController:
        """Access to angular velocity PID controller."""
        return self._angular_pid

    def set_targets(self, linear_target: float, angular_target: float) -> None:
        """
        Set target values for both controllers.

        Args:
            linear_target: Target linear velocity
            angular_target: Target angular velocity
        """
        self._linear_pid.setpoint = linear_target
        self._angular_pid.setpoint = angular_target

    def reset(self) -> None:
        """Reset both PID controllers."""
        self._linear_pid.reset()
        self._angular_pid.reset()

    def update(
        self,
        current_linear: float,
        current_angular: float,
        dt: Optional[float] = None
    ) -> Twist:
        """
        Update both PID controllers and return Twist command.

        Args:
            current_linear: Current linear velocity
            current_angular: Current angular velocity
            dt: Time delta in seconds. If None, uses system time.

        Returns:
            Twist message with updated velocity commands
        """
        # Update PID controllers
        linear_output = self._linear_pid.update(current_linear, dt)
        angular_output = self._angular_pid.update(current_angular, dt)

        # Update reused Twist message
        self._twist_msg.linear.x = linear_output
        self._twist_msg.linear.y = 0.0
        self._twist_msg.linear.z = 0.0
        self._twist_msg.angular.x = 0.0
        self._twist_msg.angular.y = 0.0
        self._twist_msg.angular.z = angular_output

        return self._twist_msg

    def update_linear_only(self, current_linear: float, dt: Optional[float] = None) -> float:
        """
        Update only linear PID controller.

        Args:
            current_linear: Current linear velocity
            dt: Time delta in seconds

        Returns:
            Linear velocity command
        """
        return self._linear_pid.update(current_linear, dt)

    def update_angular_only(self, current_angular: float, dt: Optional[float] = None) -> float:
        """
        Update only angular PID controller.

        Args:
            current_angular: Current angular velocity
            dt: Time delta in seconds

        Returns:
            Angular velocity command
        """
        return self._angular_pid.update(current_angular, dt)

    def get_status(self) -> Tuple[float, float, float, float]:
        """
        Get status information from both controllers.

        Returns:
            Tuple of (linear_setpoint, linear_integral, angular_setpoint, angular_integral)
        """
        return (
            self._linear_pid.setpoint,
            self._linear_pid.integral,
            self._angular_pid.setpoint,
            self._angular_pid.integral
        )
