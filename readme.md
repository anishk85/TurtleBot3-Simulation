# TurtleBot3 Simulation Workspace

This workspace contains TurtleBot3 simulation packages for ROS 2, including Gazebo simulation, Cartographer SLAM, and various world models.
## Autonomous Exploration with Explore Lite

This workspace supports autonomous exploration using the [explore_lite](https://github.com/ros-planning/explore_lite) package. Explore Lite enables the TurtleBot3 to autonomously explore unknown environments and build maps without manual teleoperation.


## Workspace Structure

```
ps1_ws/
├── src/
│   ├── turtlebot3/
│   │   └── turtlebot3_cartographer/
│   │       └── config/
│   │           └── turtlebot3_lds_2d.lua
│   └── turtlebot3_simulations/
│       └── turtlebot3_gazebo/
│           ├── worlds/
│           ├── models/
│           ├── launch/
│           └── CMakeLists.txt
└── readme.md
```

## Prerequisites

- ROS 2 (Humble/Foxy/Galactic)
- Gazebo 11+
- TurtleBot3 packages
- Cartographer ROS

## Installation

1. **Clone the workspace:**
   ```bash
   cd ~/
   git clone <repository-url> ps1_ws
   cd ps1_ws
   ```

2. **Install dependencies:**
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Configuration Files

### Cartographer Configuration (`turtlebot3_lds_2d.lua`)

The Cartographer configuration is optimized for TurtleBot3 LDS (Laser Distance Sensor) with the following key settings:

#### Main Parameters:
- **Map Frame**: `map`
- **Tracking Frame**: `imu_link` 
- **Published Frame**: `odom`
- **Laser Scans**: 1 scan topic
- **Range**: 0.12m - 8.0m

#### Trajectory Builder 2D:
- **Motion Filter**: 
  - Max angle: 0.1 radians
  - Max distance: 0.1 meters
  - Max time: 0.5 seconds
- **Scan Matching**:
  - Linear search window: 0.15m
  - Angular search window: 35 degrees
- **Submaps**:
  - Range data per submap: 160
  - Grid resolution: 0.05m
  - Hit probability: 0.7
  - Miss probability: 0.4

#### Pose Graph Optimization:
- **Constraint Building**:
  - Min score: 0.65
  - Global localization min score: 0.7
  - Max constraint distance: 25.0m
- **Loop Closure**:
  - Optimization every 90 nodes
  - Linear search window: 7.0m
  - Angular search window: 30 degrees

### Gazebo Models

The workspace includes various AWS RoboMaker residential models:

#### Air Conditioner Model (`aws_robomaker_residential_AirconditionerA_01`)
- **Type**: Static model
- **Mass**: 1 kg
- **Meshes**: 
  - Collision: `aws_AirconditionerA_01_collision.DAE`
  - Visual: `aws_AirconditionerA_01_visual.DAE`
- **Layer**: 1 (for rendering optimization)

## Usage

### 1. Launch Gazebo Simulation

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch Gazebo with TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Start Cartographer SLAM

```bash
# Launch Cartographer with custom configuration
ros2 launch turtlebot3_cartographer cartographer.launch.py \
    configuration_directory:=$(ros2 pkg prefix turtlebot3_cartographer)/share/turtlebot3_cartographer/config \
    configuration_basename:=turtlebot3_lds_2d.lua
```

### 3. Control the Robot

```bash
# Teleop control
ros2 run turtlebot3_teleop teleop_keyboard

# Or use autonomous navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

### 4. Save the Map

```bash
# Save generated map
ros2 run nav2_map_server map_saver_cli -f ~/map
```

# TurtleBot3 Autonomous Navigation in Custom Gazebo Environment

This project demonstrates autonomous navigation using TurtleBot3 in a custom Gazebo simulation environment. The robot can create maps using SLAM (Simultaneous Localization and Mapping) and perform autonomous navigation to specified goals.

## 🎯 Project Overview

- **Robot Platform**: TurtleBot3 Waffle
- **Simulation Environment**: Custom small house world in Gazebo
- **SLAM Algorithm**: Google Cartographer
- **Navigation Stack**: Nav2 (Navigation2)
- **ROS Version**: ROS 2 Humble

## 📋 Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo 11
- Python 3.10+

### Required ROS 2 Packages
- TurtleBot3 packages
- Cartographer ROS
- Nav2 navigation stack

## Workspace Structure

```
ps1_ws/
├── src/
│   ├── turtlebot3/
│   │   └── turtlebot3_cartographer/
│   │       └── config/
│   │           └── turtlebot3_lds_2d.lua
│   └── turtlebot3_simulations/
│       └── turtlebot3_gazebo/
│           ├── worlds/
│           ├── models/
│           ├── launch/
│           └── CMakeLists.txt
└── readme.md
```

## 🛠️ Installation

1. **Clone the repository:**
   ```bash
   cd ~/
   git clone <repository-url> ps1_ws
   cd ps1_ws
   ```

2. **Set up environment variables** - Add the following to your `~/.bashrc` file:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ps1_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
   export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ps1_ws/src
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ps1_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
   ```

3. **Reload environment variables:**
   ```bash
   source ~/.bashrc
   ```

4. **Install dependencies:**
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🚀 Usage

### Step 1: Launch Gazebo Simulation
Start the custom small house environment with TurtleBot3:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Step 2: SLAM Mapping
Launch Cartographer for simultaneous localization and mapping:
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Step 3: Manual Robot Control
Use keyboard teleoperation to drive the robot and create the map:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Teleop Controls:**
- `w` - Move forward
- `s` - Move backward
- `a` - Turn left
- `d` - Turn right
- `x` - Stop
- `Space` - Emergency stop

### Step 4: Save Map
After mapping the environment, save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Step 5: Autonomous Navigation
Launch the navigation stack with your saved map:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```
## 🤖 Autonomous Exploration with TurtleBot3

To perform autonomous exploration in the custom Gazebo environment, follow these steps:

1. **Launch the Small House Simulation:**
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

4. **Start Autonomous Exploration:**
   ```bash
   ros2 launch turtlebot3_autonomous_exploration autonomous_exploration.launch.py
   ```

> **Tip:** Make sure all environment variables are set and the workspace is sourced before running these commands.


## 🎮 Navigation Controls in RViz
- **Set Initial Pose**: Use "2D Pose Estimate" tool to set robot's starting position
- **Set Navigation Goal**: Use "2D Goal Pose" tool to set destination
- **Monitor Progress**: Watch the planned path (green line) and local path (purple line)

## 📁 Project Structure
- **Cartographer Config**: `src/turtlebot3_simulations/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua`
- **Launch Files**: `launch/`
- **World Files**: `src/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/small_house/`
- **Environment Setup**: Load ps1_ws workspace before running

## Configuration Files

### Cartographer Configuration (`turtlebot3_lds_2d.lua`)

The Cartographer configuration is optimized for TurtleBot3 LDS (Laser Distance Sensor) with the following key settings:

#### Main Parameters:
- **Map Frame**: `map`
- **Tracking Frame**: `imu_link` 
- **Published Frame**: `odom`
- **Laser Scans**: 1 scan topic
- **Range**: 0.12m - 8.0m

#### Trajectory Builder 2D:
- **Motion Filter**: 
  - Max angle: 0.1 radians
  - Max distance: 0.1 meters
  - Max time: 0.5 seconds
- **Scan Matching**:
  - Linear search window: 0.15m
  - Angular search window: 35 degrees
- **Submaps**:
  - Range data per submap: 160
  - Grid resolution: 0.05m
  - Hit probability: 0.7
  - Miss probability: 0.4

#### Pose Graph Optimization:
- **Constraint Building**:
  - Min score: 0.65
  - Global localization min score: 0.7
  - Max constraint distance: 25.0m
- **Loop Closure**:
  - Optimization every 90 nodes
  - Linear search window: 7.0m
  - Angular search window: 30 degrees

### Gazebo Models

The workspace includes various AWS RoboMaker residential models:

#### Air Conditioner Model (`aws_robomaker_residential_AirconditionerA_01`)
- **Type**: Static model
- **Mass**: 1 kg
- **Meshes**: 
  - Collision: `aws_AirconditionerA_01_collision.DAE`
  - Visual: `aws_AirconditionerA_01_visual.DAE`
- **Layer**: 1 (for rendering optimization)

## 📊 Features

✅ **SLAM Mapping**: Create maps using laser scan data  
✅ **Autonomous Navigation**: Navigate to goals while avoiding obstacles  
✅ **Path Planning**: Global and local path planning with obstacle avoidance  
✅ **Localization**: AMCL-based robot localization  
✅ **Custom Environment**: Detailed house simulation with furniture and rooms  
✅ **Real-time Visualization**: RViz integration for monitoring and control  
✅ **CycloneDDS**: Enhanced DDS implementation for better performance  

## 🎥 Demo

The robot can:
- **Map Creation**: Drive through the house to create a detailed occupancy grid map
- **Goal Navigation**: Automatically navigate between rooms avoiding obstacles
- **Re-localization**: Recover if localization is lost during navigation
- **Dynamic Replanning**: Adapt paths when encountering new obstacles

## ⚙️ Environment Variables Explained

| Variable | Purpose |
|----------|---------|
| `TURTLEBOT3_MODEL=waffle` | Sets TurtleBot3 model variant |
| `RMW_IMPLEMENTATION=rmw_cyclonedx_cpp` | Uses CycloneDDS for better performance |
| `GAZEBO_MODEL_PATH` | Tells Gazebo where to find custom models |
| `GAZEBO_RESOURCE_PATH` | Additional Gazebo resource directories |
| `GAZEBO_PLUGIN_PATH` | Gazebo plugin search paths |

## Build System

The CMakeLists.txt includes:

### Dependencies:
- `ament_cmake`
- `gazebo` and `gazebo_ros_pkgs`
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`
- `rclcpp`
- `tf2`

### Executables:
- **turtlebot3_drive**: Main driving node

### Libraries:
- **obstacle1, obstacle2**: Individual obstacle plugins
- **obstacles**: Combined obstacle plugin for DQN world

### Installation:
- Executables → `lib/${PROJECT_NAME}/`
- Resources → `share/${PROJECT_NAME}/`
- Headers → `include/`

## 🐛 Troubleshooting

### Common Issues:

1. **"Map frame does not exist"**
   - **Solution**: Set initial pose using "2D Pose Estimate" in RViz

2. **Navigation goals fail**
   - **Solution**: Ensure goals are set in free space (white areas) on the map
   - Avoid setting goals near walls or in unexplored areas

3. **Robot doesn't move**
   - Check if `use_sim_time:=True` is set in all launch commands
   - Verify TurtleBot3 model is set correctly: `echo $TURTLEBOT3_MODEL`

4. **Map quality issues**
   - Drive slower during mapping
   - Ensure good lighting in simulation
   - Make multiple passes through areas

5. **Gazebo models not loading:**
   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ps1_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
   ```

6. **SLAM not working properly:**
   - Check laser scan topic: `ros2 topic echo /scan`
   - Verify TF tree: `ros2 run tf2_tools view_frames`
   - Adjust Cartographer parameters in `turtlebot3_lds_2d.lua`

7. **Build errors:**
   ```bash
   # Clean build
   colcon build --cmake-clean-cache
   
   # Check dependencies
   rosdep check --from-paths src --ignore-src
   ```

### Quick Environment Check
```bash
# Verify environment variables
echo $TURTLEBOT3_MODEL
echo $RMW_IMPLEMENTATION
echo $GAZEBO_MODEL_PATH

# Check if workspace is sourced
source /home/anish/ps1_ws/install/setup.bash
```

## Performance Tuning

### For Better SLAM Performance:
- Increase `num_range_data` for denser submaps
- Adjust `hit_probability` and `miss_probability` for different environments
- Tune `optimize_every_n_nodes` based on computational resources

### For Gazebo Performance:
- Reduce visual complexity in model SDF files
- Use appropriate physics step sizes
- Enable/disable real-time factor as needed

## Development

### Adding New Models:
1. Create model directory in `models/`
2. Add `model.sdf` and `model.config`
3. Include meshes in `meshes/` subdirectory
4. Update CMakeLists.txt if needed

### Modifying SLAM Parameters:
1. Edit `turtlebot3_lds_2d.lua`
2. Rebuild workspace: `colcon build`
3. Test with different environments

## 📝 License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📧 Contact

- **Project Author**: Anish Kumar
- **GitHub**: @anishk85

## 🙏 Acknowledgments

- **ROBOTIS TurtleBot3** - Robot platform
- **Google Cartographer** - SLAM algorithm
- **Navigation2** - Navigation stack
- **Gazebo Worlds Dataset** - Custom simulation environments

## References

- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)

---

**Note**: This project was developed as part of a robotics navigation and SLAM demonstration using ROS 2 and modern robotic simulation tools. Make sure to load the workspace at ps1_ws and set all environment variables before running the simulation.
