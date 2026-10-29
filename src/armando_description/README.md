# armando_description

ROS 2 package containing the physical and visual description of the Armando robotic arm, developed for educational and simulation purposes.


## 🚀 Launch Files

The `armando_description` package includes two main launch files:

### armando_gazebo.launch.py
Manages the physical simulation environment in Gazebo Harmonic. Simulates the dynamics of the robotic arm, sensors, and loads the virtual world (`.sdf`).

```bash
ros2 launch armando_description armando_gazebo.launch.py
```

### armando_rviz.launch.py
Starts the visualization interface in RViz2. It subscribes to runtime topics such as TF transforms and camera data, displaying them in a 3D view.

```bash
ros2 launch armando_description armando_rviz.launch.py
```




