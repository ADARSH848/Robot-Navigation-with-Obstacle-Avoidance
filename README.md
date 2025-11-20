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

Running the DemoFollow these steps to replicate the "Inspection Circuit" results:Terminal 1: Launch Simulation

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo empty_world.launch.py


# 1. Spawn Procedural Construction Site (Barriers & Barrels)

Terminal 2: Spawn Site & Run Robot

cd ~/10x_ws

source install/setup.bash

ros2 run navigation_assignment site_loader

# 2. Launch Navigation Stack
ros2 launch navigation_assignment navigation.launch.py

Terminal 3: Validation & Analysis

cd ~/10x_ws

source install/setup.bash

# Run Unit Tests (Before Sim)
python3 src/navigation_assignment/navigation_assignment/test_spline.py

# Run Plotter (After Sim - Press Ctrl+C when robot stops)
python3 src/navigation_assignment/navigation_assignment/plot_results.py

Output Images:

<img width="1847" height="1048" alt="Screenshot from 2025-11-20 20-23-19" src="https://github.com/user-attachments/assets/6147a16c-262d-45ae-bf9a-816ffb70b0b1" />

<img width="1847" height="1048" alt="Screenshot from 2025-11-20 20-25-35" src="https://github.com/user-attachments/assets/9475ae5a-ac4d-4ebd-aca2-095795863d9e" />

<img width="1000" height="600" alt="trajectory_comparison" src="https://github.com/user-attachments/assets/9799a24f-8b18-405e-ba9f-7ced60578f42" />

<img width="1000" height="400" alt="error_analysis" src="https://github.com/user-attachments/assets/f861e896-fe01-4254-83d4-cfb59a461c6b" />


