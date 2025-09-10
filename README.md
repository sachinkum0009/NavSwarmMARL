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

ros2 launch nav_swarm_marl task_allocator.launch.py

ros2 launch nav_swarm_marl rl_trained_agent.launch.py


```
