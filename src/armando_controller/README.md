## Available Nodes and Control Modes

The `armando_controller` package offers three main modes to manage the robot:

### Predefined Pose Control
Allows sending the robot to specific joint positions saved in the `config/poses.yaml` file. It uses an **Action Client** to ensure smooth and synchronized movements.

* **Run:**
    ```bash
    ros2 run armando_controller armando_controller_node --ros-args -p pose:="pos0"
    ```
*(Replace `pos0` with the desired pose name or `home` to reset).*

### Complete Automation
Executes a timed, automated "Pick and Place" sequence. The robot moves between different poses and activates/deactivates Gazebo plugins to grab and release the cubes.

* **Run:**
    ```bash
    ros2 run armando_controller armando_pick_place_node
    ```
* **Manual Gripper Control:**
    You can force the attachment or detachment of the cubes (ID 0-3) via the following topics:
    ```bash
    # Attach cube A (ID 0)
    ros2 topic pub --once /gripper/attach_a std_msgs/msg/Empty "{}"

    # Detach cube A (ID 0)
    ros2 topic pub --once /gripper/detach_a std_msgs/msg/Empty "{}"
    ```
*Cube mapping: `a=ID 0`, `b=ID 1`, `c=ID 2`, `d=ID 3`.*

### Dynamic IK Control
This node uses analytical Inverse Kinematics (IK) to reach targets (ArUco Markers) detected by the camera.

* **Run:**
    ```bash
    ros2 launch armando_controller armando_ik.launch.py
    ```
* **Real-Time Target Change:**
    Change the ID of the tracked cube without restarting the node:
    ```bash
    ros2 param set /armando_ik_node target_id 2
    ```
*(Use the desired ID or `-1` to return the arm to the Home position).*

### KDL IK Control
The `armando_ik_kdl_node` executable is a separate KDL-based alternative to
the analytical IK node. Start the simulation and ArUco pipeline first, then
run this launch file in another terminal from the sourced workspace:

```bash
ros2 launch armando_controller armando_ik_kdl.launch.py
```

The `armando_ik_kdl_node` is the generic IK solver: it subscribes only to
`/target_pose` (`geometry_msgs/msg/PoseStamped`), transforms the pose to
`base_link`, and uses its XYZ position as the target. It builds its KDL chain
from `base_link` to `crawer_base`, uses the latest `/joint_states` as its
initial guess, and checks the resulting joint positions against the URDF
limits. End-effector orientation is not constrained.

To run the generic solver without ArUco, start only the IK node and publish a
pose:

```bash
ros2 run armando_controller armando_ik_kdl_node --ros-args \
  -p robot_description:="$(xacro $(ros2 pkg prefix armando_description)/share/armando_description/urdf/arm.urdf.xacro package_path:=$(ros2 pkg prefix armando_description)/share/armando_description)"

ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.2, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"
```

The pose frame must be connected to `base_link` through TF. The solver also
requires the robot description at startup; the command above supplies it.

The launch file enables ArUco by starting both the generic solver and a
separate adapter node. The adapter subscribes to
`/aruco_marker_publisher/markers`, selects `target_id` (default `0`), applies
the default `target_normal_offset` of `0.1` m along the marker's positive local
Z axis, and publishes the resulting `PoseStamped` on `/target_pose`. The solver
then treats it like any other pose input. Change the tracked marker with
`ros2 param set /armando_aruco_target_node target_id 2`. Launch arguments
`target_id`, `marker_topic`, `target_pose_topic`, `target_normal_offset`, and
`use_sim_time` can be set. Do not run this launch file and `armando_ik_node`
at the same time because both send goals to the same trajectory controller.

## 🔧 Useful Commands & Debugging

**Send the robot to the Home position (0,0,0,0)**

If you want to manually reset the arm pose via the Action Server:
* ```bash
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
            joint_names: ['j0', 'j1', 'j2', 'j3'],
            points: [{
                positions: [0.0, 0.0, 0.0, 0.0],
                velocities: [0.0, 0.0, 0.0, 0.0],
                time_from_start: {sec: 2, nanosec: 0}
            }]
        }
    }"

Alternatively, you can use the controller node with the preset parameter:

* ```bash
    ros2 run armando_controller armando_controller_node --ros-args -p pose:="home"
