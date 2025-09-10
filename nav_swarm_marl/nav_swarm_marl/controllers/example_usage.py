#!/usr/bin/env python3

"""
Example usage of the optimized PID controllers for robot navigation.

This script demonstrates how to use the PidParams, PidController, and
TwistPidController classes for cmd_vel command generation.
"""

import time
from .pid_controller import PidParams, PidController, TwistPidController


def demonstrate_basic_pid():
    """Demonstrate basic PID controller usage."""
    print("=== Basic PID Controller Demo ===")
    
    # Create PID parameters for position control
    params = PidParams(
        kp=2.0,           # Proportional gain
        ki=0.1,           # Integral gain
        kd=0.05,          # Derivative gain
        output_min=-1.0,  # Minimum output
        output_max=1.0    # Maximum output
    )
    
    # Create controller
    controller = PidController(params)
    controller.setpoint = 5.0  # Target position
    
    # Simulate control loop
    current_position = 0.0
    dt = 0.1
    
    print(f"Target: {controller.setpoint}")
    print("Time\tPosition\tOutput")
    
    for i in range(20):
        # Update controller
        output = controller.update(current_position, dt=dt)
        
        # Simulate system response (simple integration)
        current_position += output * dt
        
        print(f"{i*dt:.1f}\t{current_position:.3f}\t\t{output:.3f}")
        
        # Stop when close enough
        if abs(current_position - controller.setpoint) < 0.01:
            break
    
    print(f"Final position: {current_position:.3f}")
    print(f"Final integral: {controller.integral:.3f}")


def demonstrate_twist_pid():
    """Demonstrate TwistPidController usage for robot navigation."""
    print("\n=== Twist PID Controller Demo ===")
    
    # Create separate parameters for linear and angular control
    linear_params = PidParams(
        kp=1.5,           # Linear velocity proportional gain
        ki=0.05,          # Linear velocity integral gain
        kd=0.02,          # Linear velocity derivative gain
        output_min=-2.0,  # Max reverse speed (m/s)
        output_max=2.0    # Max forward speed (m/s)
    )
    
    angular_params = PidParams(
        kp=3.0,           # Angular velocity proportional gain
        ki=0.1,           # Angular velocity integral gain
        kd=0.05,          # Angular velocity derivative gain
        output_min=-1.57, # Max CCW rotation (rad/s)
        output_max=1.57   # Max CW rotation (rad/s)
    )
    
    # Create twist controller
    twist_controller = TwistPidController(linear_params, angular_params)
    
    # Set navigation targets
    target_linear = 1.0   # Target linear velocity (m/s)
    target_angular = 0.5  # Target angular velocity (rad/s)
    twist_controller.set_targets(target_linear, target_angular)
    
    # Simulate robot with current velocities
    current_linear = 0.0
    current_angular = 0.0
    dt = 0.1
    
    print(f"Targets: linear={target_linear} m/s, angular={target_angular} rad/s")
    print("Time\tLinear\t\tAngular\t\tCmd_Linear\tCmd_Angular")
    
    for i in range(15):
        # Update controller and get Twist command
        twist_cmd = twist_controller.update(current_linear, current_angular, dt=dt)
        
        # Simulate robot dynamics (simple model)
        current_linear += twist_cmd.linear.x * dt * 0.8  # Some lag
        current_angular += twist_cmd.angular.z * dt * 0.8
        
        print(f"{i*dt:.1f}\t{current_linear:.3f}\t\t{current_angular:.3f}\t\t"
              f"{twist_cmd.linear.x:.3f}\t\t{twist_cmd.angular.z:.3f}")
        
        # Stop when close enough to targets
        if (abs(current_linear - target_linear) < 0.01 and 
            abs(current_angular - target_angular) < 0.01):
            break
    
    print(f"Final velocities: linear={current_linear:.3f} m/s, "
          f"angular={current_angular:.3f} rad/s")
    
    # Show controller status
    status = twist_controller.get_status()
    print(f"Controller status: linear_setpoint={status[0]:.3f}, "
          f"linear_integral={status[1]:.3f}, angular_setpoint={status[2]:.3f}, "
          f"angular_integral={status[3]:.3f}")


def demonstrate_performance_features():
    """Demonstrate performance and robustness features."""
    print("\n=== Performance Features Demo ===")
    
    # Test derivative smoothing
    params = PidParams(kp=1.0, ki=0.0, kd=1.0)
    
    # Controller with heavy derivative smoothing
    smooth_controller = PidController(params, derivative_alpha=0.9)
    smooth_controller.setpoint = 10.0
    
    # Controller with no smoothing
    noisy_controller = PidController(params, derivative_alpha=0.0)
    noisy_controller.setpoint = 10.0
    
    print("Derivative smoothing comparison with noisy input:")
    print("Time\tInput\tSmooth_Output\tNoisy_Output")
    
    # Simulate noisy measurements
    import random
    for i in range(10):
        # Add noise to measurement
        measurement = 5.0 + random.uniform(-0.5, 0.5)
        
        smooth_output = smooth_controller.update(measurement, dt=0.1)
        noisy_output = noisy_controller.update(measurement, dt=0.1)
        
        print(f"{i*0.1:.1f}\t{measurement:.3f}\t{smooth_output:.3f}\t\t{noisy_output:.3f}")
    
    # Test integral anti-windup
    print("\nIntegral anti-windup demonstration:")
    windup_params = PidParams(
        kp=0.0, ki=1.0, kd=0.0,
        integral_min=-5.0, integral_max=5.0
    )
    windup_controller = PidController(windup_params)
    windup_controller.setpoint = 100.0  # Large setpoint to cause windup
    
    print("Time\tIntegral\tOutput")
    for i in range(10):
        output = windup_controller.update(0.0, dt=0.1)
        print(f"{i*0.1:.1f}\t{windup_controller.integral:.3f}\t\t{output:.3f}")
    
    print("Note: Integral is clamped between -5.0 and 5.0")


def main():
    """Run all demonstrations."""
    print("Optimized PID Controller Demonstration")
    print("=" * 50)
    
    demonstrate_basic_pid()
    demonstrate_twist_pid()
    demonstrate_performance_features()
    
    print("\nDemo complete! The PID controllers are ready for use in your navigation system.")


if __name__ == "__main__":
    main()