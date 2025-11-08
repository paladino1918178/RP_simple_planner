# Simple Planner
Simple Planner implementation ROS2
This project implement a simple planner using ROS 2 Jazzy.

---

## Prerequisites

- **ROS 2 Jazzy**
  
---

## Path Planner Algorithm

The **simple_planner** node receives the *map*, the *start*, and the *goal* positions, then computes the shortest path using A* on a grid map. Each cell’s cost depends on distance and proximity to walls, while occupied cells are excluded. The path goes through the center of open doors, and if all doors to the goal are closed making the route impossible, the node publishes an empty path to indicate no valid path exists. Customizable number of ports from terminal.

---

## Setup and Build Instructions

Open a terminal and follow these steps:

```bash
# Navigate to your workspace directory
cd ~/ws

# Source the ROS 2 Foxy environment
source /opt/ros/jazzy/setup.bash

# Clean previous build artifacts
rm -rf build/ install/ log/

# Build the workspace using colcon
colcon build --symlink-install

# Source the newly built workspace
source install/setup.bash

# Launch
ros2 launch simple_planner launch.py num_open_doors:=15
```

--- 

## Demo

[Link to the video demo](https://drive.google.com/drive/folders/1CLRh9fg6Q_rQKs-Xss-WJDV5EsIUgK13)
