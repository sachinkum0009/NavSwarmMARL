# Optimized PID Controllers for NavSwarmMARL

This module provides high-performance PID controller implementations specifically designed for robot navigation and cmd_vel command generation in ROS2 environments.

## Overview

The implementation consists of three main classes:

1. **`PidParams`** - Immutable parameter container with validation
2. **`PidController`** - Basic PID controller with optimized computations
3. **`TwistPidController`** - Dual PID controller for generating Twist messages

## Key Optimizations

### Performance Features
- **Memory efficiency**: Uses `__slots__` to minimize memory footprint
- **Computational efficiency**: Minimal object allocation, pre-allocated Twist messages
- **Derivative smoothing**: Exponential smoothing to reduce noise (configurable)
- **Anti-windup protection**: Integral clamping prevents windup in saturated conditions

### Code Quality
- **Type hints**: Comprehensive type annotations throughout
- **Properties**: Clean getter/setter interfaces
- **Documentation**: Detailed docstrings and comments
- **Error handling**: Parameter validation and graceful degradation
- **Modular design**: Clean separation of concerns

## Quick Start

```python
from nav_swarm_marl.controllers import PidParams, PidController, TwistPidController

# Basic PID controller
params = PidParams(kp=1.0, ki=0.1, kd=0.05, output_min=-2.0, output_max=2.0)
controller = PidController(params)
controller.setpoint = 5.0

# Update control loop
output = controller.update(current_value, dt=0.1)

# Twist PID controller for robot navigation
linear_params = PidParams(kp=1.5, ki=0.05, kd=0.02, output_min=-2.0, output_max=2.0)
angular_params = PidParams(kp=3.0, ki=0.1, kd=0.05, output_min=-1.57, output_max=1.57)

twist_controller = TwistPidController(linear_params, angular_params)
twist_controller.set_targets(1.0, 0.5)  # linear_vel, angular_vel

# Generate cmd_vel commands
twist_msg = twist_controller.update(current_linear, current_angular, dt=0.1)
```

## API Reference

### PidParams

```python
@dataclass
class PidParams:
    kp: float = 1.0                    # Proportional gain
    ki: float = 0.0                    # Integral gain  
    kd: float = 0.0                    # Derivative gain
    output_min: float = -float('inf')  # Minimum output limit
    output_max: float = float('inf')   # Maximum output limit
    integral_min: float = -float('inf') # Minimum integral limit
    integral_max: float = float('inf')  # Maximum integral limit
```

**Properties:**
- `has_integral: bool` - True if integral term is enabled (ki != 0)
- `has_derivative: bool` - True if derivative term is enabled (kd != 0)

### PidController

```python
class PidController:
    def __init__(self, params: PidParams, derivative_alpha: float = 0.8)
```

**Properties:**
- `setpoint: float` - Current target value (get/set)
- `integral: float` - Current integral accumulation (read-only)

**Methods:**
- `update(current_value: float, dt: Optional[float] = None) -> float`
- `reset() -> None` - Reset controller state

### TwistPidController

```python
class TwistPidController:
    def __init__(self, linear_params: PidParams, angular_params: PidParams, 
                 derivative_alpha: float = 0.8)
```

**Properties:**
- `linear_controller: PidController` - Access to linear controller
- `angular_controller: PidController` - Access to angular controller

**Methods:**
- `set_targets(linear_target: float, angular_target: float) -> None`
- `update(current_linear: float, current_angular: float, dt: Optional[float] = None) -> Twist`
- `update_linear_only(current_linear: float, dt: Optional[float] = None) -> float`
- `update_angular_only(current_angular: float, dt: Optional[float] = None) -> float`
- `reset() -> None` - Reset both controllers
- `get_status() -> Tuple[float, float, float, float]` - Get status info

## Advanced Features

### Derivative Smoothing
The derivative term uses exponential smoothing to reduce noise:
```python
# More smoothing (0.9) vs less smoothing (0.1)
controller = PidController(params, derivative_alpha=0.9)
```

### Integral Anti-Windup
Prevent integral windup by setting limits:
```python
params = PidParams(ki=1.0, integral_min=-10.0, integral_max=10.0)
```

### Automatic Setpoint Reset
Controllers automatically reset derivative state on significant setpoint changes to avoid spikes.

### Message Reuse
TwistPidController reuses Twist messages for efficiency in high-frequency control loops.

## Testing

The implementation includes comprehensive tests:

```bash
# Run with ROS2 environment
cd nav_swarm_marl
python3 -m pytest nav_swarm_marl/controllers/test_pid_controller.py -v

# Run standalone (without ROS2)
python3 -m pytest nav_swarm_marl/controllers/test_pid_controller_standalone.py -v
```

## Example Usage

See `nav_swarm_marl/controllers/example_usage.py` for detailed examples including:
- Basic PID control demonstration
- Twist controller for robot navigation
- Performance features (smoothing, anti-windup)

## Integration with NavSwarmMARL

The controllers are designed to integrate seamlessly with the existing NavSwarmMARL navigation system:

```python
# In your navigation node
from nav_swarm_marl.controllers import TwistPidController, PidParams

# Initialize in your node's __init__
linear_params = PidParams(kp=1.0, ki=0.1, kd=0.05, output_min=-0.5, output_max=0.5)
angular_params = PidParams(kp=2.0, ki=0.1, kd=0.1, output_min=-1.0, output_max=1.0)
self.pid_controller = TwistPidController(linear_params, angular_params)

# In your control callback
def control_callback(self):
    # Get current state from sensors/odometry
    current_linear = self.get_current_linear_velocity()
    current_angular = self.get_current_angular_velocity()
    
    # Update controller
    cmd_vel = self.pid_controller.update(current_linear, current_angular)
    
    # Publish command
    self.cmd_vel_publisher.publish(cmd_vel)
```

## License

This implementation follows the same MIT license as the NavSwarmMARL project.