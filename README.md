# Simple Planner
Simple Planner implementation ROS2
This project implement a simple planner using ROS 2 Jazzy.

---

## Prerequisites

- **ROS 2 Jazzy**
  
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
ros2 launch simple_planner launch.py
```

--- 

## Demo