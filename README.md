# Robot Navigation with Obstacle Avoidance

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Language](https://img.shields.io/badge/Language-Python3-green)
![Simulation](https://img.shields.io/badge/Simulation-Gazebo-orange)
![License](https://img.shields.io/badge/License-MIT-yellow)

**Author:** Adarsh Anand
**Role:** Robotics Software Apprentice Assignment - 10xConstruction

## 1. Executive Summary
This project implements a modular autonomous navigation stack designed for differential drive robots in construction environments. Addressing the requirements for **path smoothing**, **trajectory generation**, and **trajectory tracking**, the solution features:

* **Global Planner:** $C^2$ Continuous Cubic Spline Interpolation.
* **Controller:** Stanley-Style Precision Controller with Dynamic Braking.
* **Safety:** Reactive Tangential Vector Field for dynamic obstacle avoidance.
* **Simulation:** A procedural "Construction Site" spawner that generates a complex inspection circuit (Jersey barriers, barrels) without external assets.



## 2. Setup & Execution Instructions

### Prerequisites
* **OS:** Ubuntu 22.04
* **ROS Distro:** ROS 2 Humble
* **Dependencies:** `ros-humble-turtlebot3-gazebo`, `scipy`, `numpy`, `matplotlib`

### Installation

# 1. Create Workspace
mkdir -p ~/10x_ws/src
cd ~/10x_ws/src
git clone [https://github.com/ADARSH848/Robot-Navigation-with-Obstacle-Avoidance.git](https://github.com/ADARSH848/Robot-Navigation-with-Obstacle-Avoidance.git)

# 2. Build
cd ~/10x_ws
colcon build --packages-select navigation_assignment
source install/setup.bash

Running the DemoFollow these steps to replicate the "Inspection Circuit" results:Terminal 1: Launch SimulationBashexport TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
Terminal 2: Spawn Site & Run RobotBashcd ~/10x_ws
source install/setup.bash

# 1. Spawn Procedural Construction Site (Barriers & Barrels)
ros2 run navigation_assignment site_loader

# 2. Launch Navigation Stack
ros2 launch navigation_assignment navigation.launch.py
Terminal 3: Validation & AnalysisBashcd ~/10x_ws
source install/setup.bash

# Run Unit Tests (Before Sim)
python3 src/navigation_assignment/navigation_assignment/test_spline.py

# Run Plotter (After Sim - Press Ctrl+C when robot stops)
python3 src/navigation_assignment/navigation_assignment/plot_results.py



