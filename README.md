# armando_sim


# Armando Simulation Workspace

🤖 **Simulation workspace for Armando, a 4-DOF robotic arm in ROS 2 Humble.**


## 📚 Quick Index

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Docker Environment](#docker-environment)
- [ROS 2 Packages](#ros-2-packages)
- [Quick Start](#quick-start)


## Overview

This repository provides a complete simulation and development environment for **Armando**, a 4-DOF robotic arm, using ROS 2 Humble. The workspace is designed for both educational and research purposes, focusing on:

1. **Technical Reference**: Documenting the software architecture and runtime logic of the robotic arm in a ROS 2 Humble environment.
2. **Operational Guide**: Offering a practical workflow for developing, extending, and testing a generic robotic manipulator in simulation before deploying to real hardware.

The environment is fully containerized with Docker to ensure reproducibility, minimize host-side dependency issues, and provide a consistent setup for all developers.


## Repository Structure

The workspace is organized into two main directories:

- **`docker_scripts/`**: Scripts and configuration for building, running, and accessing the Docker-based development environment.
- **`src/`**: ROS 2 packages and simulation assets, including robot description, controllers, sensors, worlds, and launch files.

This structure keeps the host system clean and ensures every developer uses the same environment.

### 🗂️ Structure Overview

```
armando_sim/
├── docker_scripts/
│   ├── Dockerfile
│   ├── docker_build_image.sh
│   ├── docker_run_container.sh
│   └── docker_connect.sh
└── src/
  ├── armando_description/
  └── armando_controller/
```


## Docker Environment

The `docker_scripts/` directory contains scripts to manage the Docker container lifecycle:

| File | Description |
|------|-------------|
| **Dockerfile** | Defines the base image (Ubuntu 22.04 + ROS 2 Humble), installs Gazebo Harmonic, and prepares the workspace. |
| **docker_build_image.sh** | Builds the Docker image. Run this after cloning or when the Dockerfile changes. |
| **docker_run_container.sh** | Starts the container and configures X11 forwarding for GUI apps like Gazebo and RViz. |
| **docker_connect.sh** | Opens an extra shell in a running container (useful for monitoring or parallel commands). |

**Why Docker?**

1. **Isolation**: All developers use the same versions of ROS 2, Gazebo, and dependencies.
2. **Fast setup**: Environment provisioning is automated and quick.
3. **Portability**: Code developed in simulation is easily transferred to the real robot's onboard computer.


## ROS 2 Packages

The `src/` directory contains the main ROS 2 packages for the Armando simulation stack:

### armando_description

Defines the physical and visual representation of the robot, including:

- **`urdf/`**: Xacro files for the robot's structure (links, joints, sensors).
- **`meshes/`**: 3D geometry and simulation assets.
- **`worlds/`**: SDF worlds for simulation (e.g., workbench).
- **`launch/`**: Launch files for robot description, Gazebo, and RViz.
- **`config/`**: RViz and controller configuration files.
- **`CMakeLists.txt`** and **`package.xml`**: Build configuration and package metadata.

**Main launch files:**

- `armando_gazebo.launch.py`: Launches the Gazebo Harmonic simulation (robot, sensors, world).
  ```bash
  ros2 launch armando_description armando_gazebo.launch.py
  ```
- `armando_rviz.launch.py`: Launches RViz2 for visualization (TF, camera, etc.).
  ```bash
  ros2 launch armando_description armando_rviz.launch.py
  ```

### armando_controller

Implements the control logic for the Armando robot. Includes:

- **`src/`**: C++ source files for controller nodes.
- **`include/`**: C++ headers.
- **`config/`**: YAML configuration files for poses and controllers.
- **`CMakeLists.txt`** and **`package.xml`**: Build configuration and package metadata.


## Submodules

This repository may use Git submodules to organize ROS 2 packages. If so, initialize them with:

```bash
git submodule update --init --recursive
```


## Quick Start

1. Make all Docker scripts executable:
  ```bash
  chmod +x docker_scripts/*.sh
  ```
2. Build the Docker image:
  ```bash
  ./docker_scripts/docker_build_image.sh
  ```
3. Run the Docker container:
  ```bash
  ./docker_scripts/docker_run_container.sh
  ```
4. (Optional) Open an extra shell in the running container:
  ```bash
  ./docker_scripts/docker_connect.sh
  ```
5. Inside the container, launch RViz and Gazebo:
  ```bash
  ros2 launch armando_description armando_rviz.launch.py
  ros2 launch armando_description armando_gazebo.launch.py
  ```


## About

Simulation workspace for the Armando robotic arm, developed for ROS 2 Humble. Includes Gazebo/RViz integration, controller nodes, and a Docker-based workflow for a standardized, portable development experience.

### Topics

`docker` `robot` `robotics` `gazebo` `ros2` `rviz2` `robotics-education` `robotic-arm` `manipulation`




 ros2 run armando_controller armando_controller_node --ros-args -p pose:=pos0
ros2 topic pub --once /gripper/control std_msgs/msg/Bool "{data: true}"
