## 🤖 Autonomous Exploration with Explore Lite

This workspace supports autonomous exploration using the [explore_lite](https://github.com/ccny-ros-pkg/explore_lite) package. Explore Lite enables the TurtleBot3 to autonomously navigate and map unknown environments by sending exploration goals to the navigation stack.

### How to Use Explore Lite

1. **Launch the Simulation Environment:**
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_small_house_simple.launch.py
   ```

2. **Start SLAM with Cartographer:**
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
   ```

3. **Launch the Navigation Stack:**
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
   ```

4. **Start Explore Lite:**
   ```bash
   ros2 launch turtlebot3_autonomous_exploration autonomous_exploration.launch.py
   ```

The robot will begin exploring the environment, automatically selecting unexplored frontiers and navigating to them until the map is complete or all frontiers are explored.

> **Tip:** Ensure all required packages are installed and sourced, and that `use_sim_time` is set to `True` for all nodes when running in simulation.

