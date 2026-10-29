# armando_controller

ROS 2 package with joint-space pose control, a timed motion sequence, and a
KDL-based inverse-kinematics node.

## Nodes

### `armando_controller_node` — predefined joint poses

Loads named joint configurations from `config/poses.yaml` and sends them to
the `FollowJointTrajectory` action
`/joint_trajectory_controller/follow_joint_trajectory`. The default pose is
`pos0`; choose another configured pose with the `pose` parameter. `home`
commands all four joints to zero.

```bash
ros2 run armando_controller armando_controller_node --ros-args -p pose:=home
ros2 run armando_controller armando_controller_node --ros-args -p pose:=pos0
```

Additional pose names can be inspected in
[`config/poses.yaml`](./config/poses.yaml).
The `poses_file` parameter can select another YAML file, and `controller_name`
can override the trajectory action name.

### `armando_pick_place_node` — timed joint and gripper sequence

Runs a fixed sequence of joint trajectories and publishes attach/detach
commands on `/gripper/attach_a` through `/gripper/attach_d` and the matching
`/gripper/detach_*` topics.

```bash
ros2 run armando_controller armando_pick_place_node
```

The current Gazebo world does not include graspable objects or matching
detachable-joint plugins. Consequently, this node can publish its motion and
gripper sequence, but its attach/detach commands will not pick up or release
objects unless compatible models and plugins are added to the simulation.

### `armando_ik_kdl_node` — generic Cartesian target

Subscribes to `/target_pose` (`geometry_msgs/msg/PoseStamped`), transforms the
pose into `base_link`, and solves for its XYZ position. It uses the latest
`/joint_states` as its IK initial guess, checks the URDF joint limits, and
sends a trajectory to
`/joint_trajectory_controller/follow_joint_trajectory`. The solver does not
constrain end-effector orientation.

Start Gazebo first, then launch the solver:

```bash
ros2 launch armando_controller armando_ik_kdl.launch.py
```

Send a target pose in `base_link`:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.2, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"
```

The chosen position must be reachable and within the robot's joint limits.
