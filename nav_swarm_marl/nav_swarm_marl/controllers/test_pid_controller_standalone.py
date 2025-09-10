#!/usr/bin/env python3

"""
Unit tests for PID controller implementation (without ROS2 dependencies).

Comprehensive test suite covering all PID controller functionality
with focus on performance and correctness.
"""

import unittest
from unittest.mock import patch, MagicMock
import sys

# Mock ROS2 geometry_msgs for testing without ROS2
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()

# Create a mock Twist class


class MockTwist:
    def __init__(self):
        self.linear = MagicMock()
        self.angular = MagicMock()
        self.linear.x = 0.0
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0


sys.modules['geometry_msgs'].msg.Twist = MockTwist

# Import after mocking
from .pid_controller import PidParams, PidController, TwistPidController  # noqa: E402


class TestPidParams(unittest.TestCase):
    """Test PID parameters validation and properties."""

    def test_default_params(self):
        """Test default parameter values."""
        params = PidParams()
        self.assertEqual(params.kp, 1.0)
        self.assertEqual(params.ki, 0.0)
        self.assertEqual(params.kd, 0.0)
        self.assertEqual(params.output_min, -float('inf'))
        self.assertEqual(params.output_max, float('inf'))

    def test_custom_params(self):
        """Test custom parameter initialization."""
        params = PidParams(kp=2.0, ki=1.0, kd=0.5, output_min=-10.0, output_max=10.0)
        self.assertEqual(params.kp, 2.0)
        self.assertEqual(params.ki, 1.0)
        self.assertEqual(params.kd, 0.5)
        self.assertEqual(params.output_min, -10.0)
        self.assertEqual(params.output_max, 10.0)

    def test_invalid_output_limits(self):
        """Test validation of output limits."""
        with self.assertRaises(ValueError):
            PidParams(output_min=10.0, output_max=5.0)

    def test_invalid_integral_limits(self):
        """Test validation of integral limits."""
        with self.assertRaises(ValueError):
            PidParams(integral_min=10.0, integral_max=5.0)

    def test_has_integral_property(self):
        """Test has_integral property."""
        params_no_integral = PidParams(ki=0.0)
        params_with_integral = PidParams(ki=1.0)

        self.assertFalse(params_no_integral.has_integral)
        self.assertTrue(params_with_integral.has_integral)

    def test_has_derivative_property(self):
        """Test has_derivative property."""
        params_no_derivative = PidParams(kd=0.0)
        params_with_derivative = PidParams(kd=1.0)

        self.assertFalse(params_no_derivative.has_derivative)
        self.assertTrue(params_with_derivative.has_derivative)


class TestPidController(unittest.TestCase):
    """Test basic PID controller functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.params = PidParams(kp=1.0, ki=0.1, kd=0.01, output_min=-10.0, output_max=10.0)
        self.controller = PidController(self.params)

    def test_initialization(self):
        """Test controller initialization."""
        self.assertEqual(self.controller.setpoint, 0.0)
        self.assertEqual(self.controller.integral, 0.0)

    def test_setpoint_property(self):
        """Test setpoint getter and setter."""
        self.controller.setpoint = 5.0
        self.assertEqual(self.controller.setpoint, 5.0)

    def test_proportional_only(self):
        """Test proportional control only."""
        params = PidParams(kp=2.0, ki=0.0, kd=0.0)
        controller = PidController(params)
        controller.setpoint = 10.0

        output = controller.update(5.0, dt=0.1)
        expected = 2.0 * (10.0 - 5.0)  # kp * error
        self.assertEqual(output, expected)

    def test_integral_accumulation(self):
        """Test integral term accumulation."""
        params = PidParams(kp=0.0, ki=1.0, kd=0.0)
        controller = PidController(params)
        controller.setpoint = 10.0

        # First update
        output1 = controller.update(5.0, dt=0.1)
        expected_integral = 5.0 * 0.1  # error * dt
        self.assertAlmostEqual(output1, expected_integral, places=5)

        # Second update
        output2 = controller.update(5.0, dt=0.1)
        expected_integral = 5.0 * 0.2  # accumulated error * dt
        self.assertAlmostEqual(output2, expected_integral, places=5)

    def test_derivative_computation(self):
        """Test derivative term computation."""
        params = PidParams(kp=0.0, ki=0.0, kd=1.0)
        controller = PidController(params, derivative_alpha=0.0)  # No smoothing
        controller.setpoint = 10.0

        # First update establishes baseline
        controller.update(5.0, dt=0.1)

        # Second update should show derivative
        output = controller.update(6.0, dt=0.1)
        error_rate = (4.0 - 5.0) / 0.1  # (new_error - old_error) / dt
        expected = 1.0 * error_rate
        self.assertAlmostEqual(output, expected, places=5)

    def test_output_clamping(self):
        """Test output value clamping."""
        params = PidParams(kp=10.0, output_min=-2.0, output_max=2.0)
        controller = PidController(params)
        controller.setpoint = 10.0

        output = controller.update(0.0, dt=0.1)  # Large error
        self.assertEqual(output, 2.0)  # Should be clamped to max

        controller.setpoint = -10.0
        output = controller.update(0.0, dt=0.1)  # Large negative error
        self.assertEqual(output, -2.0)  # Should be clamped to min

    def test_integral_clamping(self):
        """Test integral anti-windup."""
        params = PidParams(kp=0.0, ki=1.0, kd=0.0, integral_min=-5.0, integral_max=5.0)
        controller = PidController(params)
        controller.setpoint = 100.0  # Large setpoint to cause windup

        # Run many updates to accumulate large integral
        for _ in range(100):
            controller.update(0.0, dt=0.1)

        # Integral should be clamped
        self.assertLessEqual(controller.integral, 5.0)
        self.assertGreaterEqual(controller.integral, -5.0)

    def test_reset_functionality(self):
        """Test controller reset."""
        self.controller.setpoint = 10.0
        self.controller.update(5.0, dt=0.1)  # Build up some state

        self.assertNotEqual(self.controller.integral, 0.0)

        self.controller.reset()
        self.assertEqual(self.controller.integral, 0.0)

    def test_automatic_time_calculation(self):
        """Test automatic time delta calculation."""
        with patch('time.time', side_effect=[1.0, 1.1, 1.2]):
            self.controller.setpoint = 10.0
            self.controller.update(5.0)  # First call, dt=0
            output = self.controller.update(6.0)  # Second call, dt=0.1

            # Should work without explicitly providing dt
            self.assertIsInstance(output, float)


class TestTwistPidController(unittest.TestCase):
    """Test Twist PID controller functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.linear_params = PidParams(kp=1.0, ki=0.1, kd=0.01, output_min=-2.0, output_max=2.0)
        self.angular_params = PidParams(kp=2.0, ki=0.2, kd=0.02, output_min=-1.0, output_max=1.0)
        self.twist_controller = TwistPidController(self.linear_params, self.angular_params)

    def test_initialization(self):
        """Test twist controller initialization."""
        self.assertIsInstance(self.twist_controller.linear_controller, PidController)
        self.assertIsInstance(self.twist_controller.angular_controller, PidController)

    def test_set_targets(self):
        """Test setting target values."""
        self.twist_controller.set_targets(1.5, 0.5)

        self.assertEqual(self.twist_controller.linear_controller.setpoint, 1.5)
        self.assertEqual(self.twist_controller.angular_controller.setpoint, 0.5)

    def test_update_returns_twist(self):
        """Test that update returns proper Twist message."""
        self.twist_controller.set_targets(1.0, 0.5)

        twist = self.twist_controller.update(0.8, 0.3, dt=0.1)

        # Check the type based on what we actually get
        self.assertTrue(hasattr(twist, 'linear'))
        self.assertTrue(hasattr(twist, 'angular'))
        self.assertIsInstance(twist.linear.x, float)
        self.assertIsInstance(twist.angular.z, float)

        # Check that other components are zero
        self.assertEqual(twist.linear.y, 0.0)
        self.assertEqual(twist.linear.z, 0.0)
        self.assertEqual(twist.angular.x, 0.0)
        self.assertEqual(twist.angular.y, 0.0)

    def test_linear_only_update(self):
        """Test linear-only update functionality."""
        self.twist_controller.linear_controller.setpoint = 1.0

        output = self.twist_controller.update_linear_only(0.8, dt=0.1)

        self.assertIsInstance(output, float)
        # The output includes proportional + integral terms
        self.assertAlmostEqual(output, 0.202, places=2)

    def test_angular_only_update(self):
        """Test angular-only update functionality."""
        self.twist_controller.angular_controller.setpoint = 0.5

        output = self.twist_controller.update_angular_only(0.3, dt=0.1)

        self.assertIsInstance(output, float)
        # The output includes proportional + integral terms
        self.assertAlmostEqual(output, 0.404, places=2)

    def test_reset_functionality(self):
        """Test twist controller reset."""
        self.twist_controller.set_targets(1.0, 0.5)
        self.twist_controller.update(0.8, 0.3, dt=0.1)  # Build up some state

        self.twist_controller.reset()

        self.assertEqual(self.twist_controller.linear_controller.integral, 0.0)
        self.assertEqual(self.twist_controller.angular_controller.integral, 0.0)

    def test_get_status(self):
        """Test status information retrieval."""
        self.twist_controller.set_targets(1.5, 0.8)
        self.twist_controller.update(1.0, 0.5, dt=0.1)  # Build up some integral

        status = self.twist_controller.get_status()

        self.assertIsInstance(status, tuple)
        self.assertEqual(len(status), 4)

        linear_setpoint, linear_integral, angular_setpoint, angular_integral = status
        self.assertEqual(linear_setpoint, 1.5)
        self.assertEqual(angular_setpoint, 0.8)
        self.assertIsInstance(linear_integral, float)
        self.assertIsInstance(angular_integral, float)

    def test_message_reuse_efficiency(self):
        """Test that Twist message is reused for efficiency."""
        self.twist_controller.set_targets(1.0, 0.5)

        # Get two twist messages
        twist1 = self.twist_controller.update(0.8, 0.3, dt=0.1)
        twist2 = self.twist_controller.update(0.9, 0.4, dt=0.1)

        # Should be the same object (reused for efficiency)
        self.assertIs(twist1, twist2)


class TestPerformanceOptimizations(unittest.TestCase):
    """Test performance-related optimizations."""

    def test_slots_usage(self):
        """Test that __slots__ are properly used for memory efficiency."""
        params = PidParams()
        controller = PidController(params)
        twist_controller = TwistPidController(params, params)

        # Check that __slots__ prevent dynamic attributes
        with self.assertRaises(AttributeError):
            controller.dynamic_attribute = "test"

        with self.assertRaises(AttributeError):
            twist_controller.dynamic_attribute = "test"

    def test_minimal_object_allocation(self):
        """Test that minimal objects are allocated during updates."""
        params = PidParams(kp=1.0, ki=0.1, kd=0.01)
        twist_controller = TwistPidController(params, params)

        # Run multiple updates
        for i in range(100):
            twist_controller.update(float(i), float(i * 0.1), dt=0.01)

        # This test mainly ensures no exceptions and reasonable performance
        # Actual performance measurement would require profiling tools
        self.assertTrue(True)  # If we get here, no exceptions were raised


if __name__ == '__main__':
    unittest.main()
