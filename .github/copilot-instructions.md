# NavSwarmMARL - Multi-Agent Reinforcement Learning Navigation

NavSwarmMARL is a ROS2-based multi-agent reinforcement learning system for navigation of robot swarms. The system uses TurtleBot3 robots in a Gazebo-simulated hospital environment with RVIZ visualization for task allocation and reinforcement learning-based navigation.

**ALWAYS reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.**

## Working Effectively

### Initial Environment Setup
1. **Install ROS2 Humble** (REQUIRED):
   ```bash
   # Add ROS2 repository
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   # Install ROS2 and dependencies
   sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
   ```
   - NEVER CANCEL: Installation takes 10-20 minutes. Set timeout to 30+ minutes.

2. **Install TurtleBot3 packages**:
   ```bash
   sudo apt install -y ros-humble-turtlebot3* ros-humble-gazebo-*
   ```
   - Takes 5-10 minutes. NEVER CANCEL. Set timeout to 15+ minutes.

3. **Set environment variables**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
   echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
   source ~/.bashrc
   ```

### Building the Workspace
1. **Source ROS2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Build the workspace**:
   ```bash
   cd NavSwarmMARL
   colcon build --packages-select nav_swarm_inter nav_swarm_marl
   ```
   - NEVER CANCEL: Build takes 2-5 minutes. Set timeout to 10+ minutes.
   - Always build both packages as nav_swarm_marl depends on nav_swarm_inter service definitions.

3. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

### Running the System
**CRITICAL**: Always run commands in the exact order listed below. Each command must be run in a separate terminal.

1. **Terminal 1 - ROS2 Middleware**:
   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ```

2. **Terminal 2 - Launch Hospital World**:
   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 launch nav_swarm_marl hospital_world.launch.py
   ```
   - NEVER CANCEL: Gazebo startup takes 30-60 seconds. Set timeout to 5+ minutes.
   - Wait until you see "Loading gazebo_ros..." complete before proceeding.

3. **Terminal 3 - Spawn TurtleBot3 Robots**:
   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 launch nav_swarm_marl spawn_tb3.launch.py
   ```
   - Spawns 3 TurtleBot3 robots (tb3_0, tb3_1, tb3_2) in hospital environment.

4. **Terminal 4 - Static Transformations**:
   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 launch nav_swarm_marl static_tfs.launch.py
   ```

5. **Terminal 5 - RVIZ Visualization**:
   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash
   rviz2
   ```

6. **Terminal 6 - Task Allocator**:
   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 launch nav_swarm_marl task_allocator.launch.py
   ```

7. **Terminal 7 - RL Trained Agent**:
   ```bash
   source /opt/ros/humble/setup.bash && source install/setup.bash
   ros2 launch nav_swarm_marl rl_trained_agent.launch.py
   ```

### Testing and Validation

1. **Run Python linting** (ALWAYS run before committing):
   ```bash
   cd nav_swarm_marl
   flake8 nav_swarm_marl/ --max-line-length=100
   ```
   - Fix all linting errors before committing changes.

2. **Run unit tests**:
   ```bash
   cd nav_swarm_marl
   python3 -m pytest test/ -v
   ```
   - NEVER CANCEL: Tests take 1-2 minutes. Set timeout to 5+ minutes.

3. **Colcon test** (when ament dependencies available):
   ```bash
   colcon test --packages-select nav_swarm_inter nav_swarm_marl
   colcon test-result --verbose
   ```

### Manual Validation Scenarios

**CRITICAL**: After making any changes, you MUST test these complete scenarios:

1. **Basic System Launch**:
   - Launch all 7 components in order as shown above
   - Verify Gazebo opens with hospital environment
   - Verify 3 TurtleBot3 robots spawn at positions: (1,16), (1,11), (3,9)
   - Verify RVIZ opens without errors

2. **Task Allocation Workflow**:
   - In RVIZ, set a "2D Nav Goal" using the toolbar
   - Verify task allocator node receives the goal
   - Check terminal output for task allocation decisions
   - Verify robots begin navigation toward assigned goals

3. **Multi-Robot Coordination**:
   - Set multiple goals in RVIZ to test multi-agent task allocation
   - Verify robots coordinate and avoid collisions
   - Check that closest robot is typically assigned to each task

## Key Project Structure

### Packages
- **nav_swarm_inter**: CMake-based package containing ROS2 service definitions
  - `srv/Goal.srv`: Service interface for goal requests
- **nav_swarm_marl**: Python-based main package containing navigation and MARL logic
  - `nav_swarm_marl/`: Main Python module
  - `launch/`: ROS2 launch files for system components
  - `test/`: Unit tests and linting checks

### Important Files
- **Launch Files** (in nav_swarm_marl/launch/):
  - `hospital_world.launch.py`: Gazebo hospital environment
  - `spawn_tb3.launch.py`: TurtleBot3 robot spawning
  - `task_allocator.launch.py`: Task allocation node
  - `rl_trained_agent.launch.py`: RL navigation agent

- **Core Nodes**:
  - `task_allocator_node.py`: Multi-agent task allocation logic
  - `scenario_executor_node.py`: Scenario execution management
  - `laserscan_filter_node.py`: Laser scan data filtering

- **Algorithms**:
  - `models/decentralized_multi_agent_task_allocator.py`: MATA algorithm implementation
  - `scenarios/`: Predefined navigation scenarios

### Dependencies
- **ROS2**: rclpy, sensor_msgs, geometry_msgs, nav_msgs
- **Python**: numpy, pytest
- **Simulation**: gazebo_ros, turtlebot3_gazebo
- **Middleware**: rmw_zenoh_cpp (for performance)

## Troubleshooting Common Issues

1. **"No module named 'sensor_msgs'"**: Source ROS2 environment first
2. **Gazebo fails to load**: Install gazebo-ros packages and set GAZEBO_MODEL_PATH
3. **Robots don't spawn**: Ensure hospital_world.launch.py runs first and completes loading
4. **Task allocator not responding**: Check that all static transforms are published
5. **Build errors**: Always build nav_swarm_inter before nav_swarm_marl

## Development Guidelines

1. **Before making changes**: Always test the complete launch sequence
2. **After making changes**: Run linting, unit tests, and full validation scenarios
3. **Code style**: Follow PEP8 with 100-character line limit
4. **Testing**: Add unit tests for new functionality in the test/ directory
5. **Dependencies**: Avoid adding new dependencies without updating package.xml

## Performance Notes

- **System Requirements**: 4GB+ RAM recommended for Gazebo simulation
- **Network**: Zenoh middleware provides better performance than default DDS
- **Scaling**: System tested with 3 robots, can scale to more with parameter adjustments
- **Real-time**: Simulation runs in real-time; adjust time scaling if needed

**REMEMBER**: Always source both ROS2 environment AND workspace setup in every new terminal before running any ros2 commands.