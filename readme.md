
---

# 🤖 **TurtleBot3 Simulation Workspace**


<div align="center">

### 🎥 **Demo Videos**

[![TurtleBot3 Simulation Demo](https://img.youtube.com/vi/KL_PmrV-Q5o/0.jpg)](https://youtu.be/KL_PmrV-Q5o)
*TurtleBot3 Simulation Demo*

[![TurtleBot3 Autonomous Exploration](https://img.youtube.com/vi/18np9hjbdX8/0.jpg)](https://youtu.be/18np9hjbdX8)
*TurtleBot3 Autonomous Exploration*

</div>

> *Watch TurtleBot3 autonomously explore, map, and navigate a custom house simulation!*

---

## 🗂️ **Workspace Overview**

This workspace provides **TurtleBot3 simulation packages** for **ROS 2**, including:

* Gazebo simulation
* **Google Cartographer SLAM**
* Custom world models
* **Autonomous exploration** with `explore_lite`

---

## 📌 **Features**

✅ SLAM Mapping
✅ Autonomous Navigation (Nav2)
✅ Path Planning & Obstacle Avoidance
✅ AMCL Localization
✅ Realistic Custom House World
✅ RViz Visualization
✅ CycloneDDS for better ROS 2 performance
✅ **Explore Lite** for fully autonomous exploration of unknown spaces

---

## 🗂️ **Workspace Structure**

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
└── README.md
```

---

## ⚙️ **Prerequisites**

* **OS**: Ubuntu 22.04 LTS
* **ROS 2**: Humble Hawksbill
* **Simulator**: Gazebo 11+
* **Python**: 3.10+
* Required Packages:

  * `turtlebot3`
  * `cartographer_ros`
  * `navigation2`

---

## 📥 **Installation**

1️⃣ **Clone the repository**

```bash
cd ~/
git clone <repository-url> ps1_ws
cd ps1_ws
```

2️⃣ **Add environment variables to `~/.bashrc`**

```bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ps1_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ps1_ws/src
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ps1_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```

3️⃣ **Reload your terminal**

```bash
source ~/.bashrc
```

4️⃣ **Install dependencies**

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

5️⃣ **Build the workspace**

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 **How to Use**

### 🏠 Launch the Custom House Simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### 🗺️ Start SLAM with Cartographer

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 🎮 Teleop Drive for Mapping

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Teleop Controls:**
`w` = forward | `s` = backward | `a` = left | `d` = right | `x` = stop | `Space` = emergency stop

### 💾 Save the Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 🧭 Launch Autonomous Navigation

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

### 🤖 Fully Autonomous Exploration with Explore Lite

```bash
ros2 launch turtlebot3_autonomous_exploration autonomous_exploration.launch.py
```

---

## 🧩 **Key Configuration**

### 📄 **Cartographer Config**

Location: `src/turtlebot3/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua`

Key settings:

* 📌 **Frames**: `map` → `odom` → `imu_link`
* 📌 **Sensor**: LDS Laser Scan, range 0.12m–8.0m
* 📌 **Submaps**: 160 scans/submap @ 0.05m resolution
* 📌 **Pose Graph**: Loop closure every 90 nodes

---

## 🗃️ **Gazebo Models**

Includes **AWS RoboMaker** residential models:

* Air Conditioners
* Furniture & rooms
* Realistic textures and collision meshes

---

## 🎯 **RViz Controls**

* **2D Pose Estimate**: Set initial pose
* **2D Nav Goal**: Click to set a goal
* Monitor paths: *Green* (global) & *Purple* (local)

---

## 🐞 **Troubleshooting**

| Issue                 | Solution                                    |
| --------------------- | ------------------------------------------- |
| **Map frame missing** | Use RViz **2D Pose Estimate**               |
| **Goals fail**        | Set goals in free space only                |
| **Robot won’t move**  | Verify `use_sim_time` & `$TURTLEBOT3_MODEL` |
| **Missing models**    | Check `$GAZEBO_MODEL_PATH`                  |
| **SLAM issues**       | Verify `/scan` topic & TF tree              |

---

## ⚙️ **Performance Tips**

✅ Tune `num_range_data`, `hit_probability`, `miss_probability` in `turtlebot3_lds_2d.lua`
✅ Keep visual meshes simple for faster Gazebo performance
✅ Use `CycloneDDS` for robust ROS 2 communication

---

## 🧩 **Development**

* **Add models**: `models/` folder → `model.sdf` + `model.config` + meshes
* **Update CMake**: Add new plugins or worlds as needed
* **Test configs**: Edit `.lua` → rebuild → test in sim

---

## 📜 **License**

Apache License 2.0 — see `LICENSE`.

---

## ✨ **Acknowledgments**

Special thanks to:

* **ROBOTIS TurtleBot3**
* **Google Cartographer**
* **Nav2**
* **Gazebo** simulation team

📚 **References:**

* [TurtleBot3 Docs](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
* [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/)
* [Gazebo Tutorials](http://gazebosim.org/tutorials)
* [ROS 2 Docs](https://docs.ros.org/en/humble/)
* [Explore Lite](https://github.com/ccny-ros-pkg/explore_lite)

---

## 🙌 **Contact**

**Author:** Anish Kumar
**GitHub:** [@anishk85](https://github.com/anishk85)

---

✨ *Enjoy mapping & navigating with your TurtleBot3!*

---

