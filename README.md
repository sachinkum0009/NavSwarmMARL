# NavSwarmMARL
Multi Agent Reinforcement Learning based Navigation for Swarm of Robots

## Description

This package contains the code for running navigation for swarm of robots using Multi-Agent Reinforcement Learning (MARL). The system enables multiple TurtleBot3 robots to navigate collaboratively in hospital environments using trained RL agents for intelligent task allocation and collision-free navigation.

### Features
- Multi-agent reinforcement learning for robot swarm navigation
- Hospital environment simulation with realistic scenarios
- Task allocation system for efficient robot coordination
- Laser scan filtering for improved perception
- Trained agent inference for real-time decision making

## Quick Start Commands

```bash
# Launch laser scan filter for sensor preprocessing
ros2 launch nav_swarm_marl laserscan_filter.launch.py

# Start RL inference server for trained agents
ros2 launch nav_swarm_marl rl_inference_server.launch.py

# Launch task allocator for robot coordination
ros2 launch nav_swarm_marl task_allocator.launch.py

# Run scenario executor for testing scenarios
ros2 run nav_swarm_marl scenario_executor_node
```

## Complete Setup Commands

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd

ros2 launch nav_swarm_marl hospital_world.launch.py

ros2 launch nav_swarm_marl spawn_tb3.launch.py

ros2 launch nav_swarm_marl static_tfs.launch.py 

rviz2

ros2 launch nav_swarm_marl rl_inference_server.launch.py

ros2 launch nav_swarm_marl task_allocator.launch.py

ros2 launch nav_swarm_marl rl_trained_agent.launch.py
```

## Commands related to Robot

```bash
start_zenohd

ros2 launch turtlebot3_bringup robot.launch.py

ros2 launch nav_swarm_marl laserscan_filter.launch.py
```

## Installation

```bash
sudo apt-get install libsdl2-dev

pip install stablebaseline3
```

## Commands for tb3

```bash
start_zenohd

ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb1

ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb2


```

## Commands for mapping

```bash
ros2 launch nav_swarm_marl cartographer.launch.py namespace:=tb2

ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb2/cmd_vel

ros2 launch nav_swarm_marl multi_nav2_bringup.launch.py namespace:=tb2

ros2 run nav2_map_server map_saver_cli -f map --ros-args -r __ns:=/tb2
```



## Commands to test on the tb3 with navigation2

```bash
ros2 launch nav_swarm_marl navigation_server.launch.py
ros2 launch nav_swarm_marl task_allocator.launch.py
```

# Final Commands for running multiple sim tests

```bash
ros2 launch nav_swarm_marl rl_inference_server.launch.py

ros2 launch nav_swarm_marl task_allocator.launch.py

ros2 run nav_swarm_marl scenario_executor_node

# to start the laserscan filter for real robots
ros2 launch nav_swarm_marl laserscan_filter.launch.py
```

### 4 September 2025

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd

ros2 launch nav_swarm_marl laserscan_filter.launch.py

ros2 launch nav_swarm_marl rl_inference_server.launch.py

ros2 launch nav_swarm_marl task_allocator.launch.py

ros2 run nav_swarm_marl scenario_executor_node
```

 Scenario Name   | Points                      |   Reached Points |   Total Points |   Delay | Distance Covered                       |
+=================+=============================+==================+================+=========+========================================+
| Test 1          | [[10, 0], [10, 0], [10, 0]] |                1 |              3 |     0.5 | [2.506572723388672, 9.930667877197266] |
+-----------------+-----------------------------+---------