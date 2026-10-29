# armando_controller

The package contains the provided KDL inverse-kinematics node and the starter
interfaces for the ArUco classwork.

## Provided KDL solver

`armando_ik_kdl_node` is independent of ArUco. It subscribes to
`/target_pose` (`geometry_msgs/msg/PoseStamped`), transforms the pose to
`base_link`, and solves for its XYZ position. It uses `/joint_states` as its
initial guess, checks the joint limits from the URDF, and sends a trajectory
to `/joint_trajectory_controller/follow_joint_trajectory`. End-effector
orientation is not constrained.

Run the solver after starting the simulation and controller stack:

```bash
ros2 launch armando_controller armando_ik_kdl.launch.py
```

For a manual smoke test, publish a reachable pose in the `base_link` frame:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.2, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"
```

## Classwork starter

The ArUco publisher has deliberately been removed from the Gazebo launch, the
world no longer contains ready-made marker textures, and
`armando_aruco_target_node.cpp` is an incomplete scaffold. Complete the tasks
in [`KDL_IK_CLASSWORK.md`](../../KDL_IK_CLASSWORK.md). The launch file
`armando_aruco_ik.launch.py` is also a scaffold for composing the detector and
pose adapter with the provided IK solver.
