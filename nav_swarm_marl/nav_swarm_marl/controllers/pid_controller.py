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
    Immutable PID parameters container with validation and optimized properties.

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
        """Validate parameters after initialization and cache properties."""
        if self.output_min >= self.output_max:
            raise ValueError("output_min must be less than output_max")
        if self.integral_min >= self.integral_max:
            raise ValueError("integral_min must be less than integral_max")

        # Cache properties for faster access
        object.__setattr__(self, '_has_integral', self.ki != 0.0)
        object.__setattr__(self, '_has_derivative', self.kd != 0.0)

    @property
    def has_integral(self) -> bool:
        """Check if integral term is enabled (cached)."""
        return getattr(self, '_has_integral', self.ki != 0.0)

    @property
    def has_derivative(self) -> bool:
        """Check if derivative term is enabled (cached)."""
        return getattr(self, '_has_derivative', self.kd != 0.0)


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
        '_last_time', '_last_derivative', '_derivative_alpha',
        '_has_integral', '_has_derivative'  # Cache property values
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

        # Cache frequently accessed properties for performance
        self._has_integral = params.ki != 0.0
        self._has_derivative = params.kd != 0.0

    @property
    def setpoint(self) -> float:
        """Current setpoint value."""
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value: float) -> None:
        """Set new setpoint and reset if it's a significant change."""
        # Optimized comparison with early return
        if abs(value - self._setpoint) <= 1e-6:
            return

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

    def _refresh_cache(self) -> None:
        """Refresh cached property values after parameter changes."""
        self._has_integral = self._params.ki != 0.0
        self._has_derivative = self._params.kd != 0.0

    def update(self, current_value: float, dt: Optional[float] = None) -> float:
        """
        Update PID controller and return control output.

        Args:
            current_value: Current process variable value
            dt: Time delta in seconds. If None, uses system time.

        Returns:
            Control output value
        """
        error = self._setpoint - current_value

        # Optimize time handling - only get current_time when needed
        if dt is None:
            current_time = time.time()
            dt = 0.0 if self._last_time is None else current_time - self._last_time
            self._last_time = current_time

        # Proportional term
        output = self._params.kp * error

        # Integral term (with optimized anti-windup) - use cached value
        if self._has_integral and dt > 0:
            integral_candidate = self._integral + error * dt
            # Optimized clamping using conditional assignment
            if integral_candidate < self._params.integral_min:
                self._integral = self._params.integral_min
            elif integral_candidate > self._params.integral_max:
                self._integral = self._params.integral_max
            else:
                self._integral = integral_candidate
            output += self._params.ki * self._integral

        # Derivative term (with optimized smoothing) - use cached value
        if self._has_derivative and dt > 0 and self._last_error is not None:
            derivative = (error - self._last_error) / dt
            # Optimized exponential smoothing
            alpha = self._derivative_alpha
            self._last_derivative = alpha * self._last_derivative + (1.0 - alpha) * derivative
            output += self._params.kd * self._last_derivative

        # Store error for next iteration
        self._last_error = error

        # Optimized output clamping
        if output < self._params.output_min:
            return self._params.output_min
        elif output > self._params.output_max:
            return self._params.output_max
        else:
            return output


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
        Set target values for both controllers efficiently.

        Args:
            linear_target: Target linear velocity
            angular_target: Target angular velocity
        """
        # Optimized: set both targets without intermediate calculations
        if abs(linear_target - self._linear_pid._setpoint) > 1e-6:
            self._linear_pid.setpoint = linear_target
        if abs(angular_target - self._angular_pid._setpoint) > 1e-6:
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

        # Optimized Twist message update - avoid repeated zero assignments
        twist = self._twist_msg
        twist.linear.x = linear_output
        twist.angular.z = angular_output

        # Only set other values if they're not already zero (optimization)
        if twist.linear.y != 0.0:
            twist.linear.y = 0.0
        if twist.linear.z != 0.0:
            twist.linear.z = 0.0
        if twist.angular.x != 0.0:
            twist.angular.x = 0.0
        if twist.angular.y != 0.0:
            twist.angular.y = 0.0

        return twist

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
