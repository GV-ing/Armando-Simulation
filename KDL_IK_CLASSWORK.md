# KDL IK + ArUco classwork

## Objective

Complete the camera-to-robot target pipeline without changing the provided
inverse-kinematics solver. The finished system must detect a chosen ArUco tag,
estimate its pose, and publish a target pose that the KDL node can consume.

The supplied solver subscribes to:

- Topic: `/target_pose`
- Type: `geometry_msgs/msg/PoseStamped`
- Required frame: `base_link`, or a frame connected to `base_link` through TF
- Used target: XYZ position; the solver does not constrain end-effector
  orientation

The solver and its topic contract are provided. Marker design, detection,
transforms, pose adaptation, and launch integration are student work.

## Starting point

The starter workspace intentionally:

- Keeps the robot, camera, Gazebo world, ROS-Gazebo image bridge, and KDL IK
  solver.
- Keeps the obstacle boxes but removes their ready-made ArUco visuals and
  generated marker textures.
- Does not start an ArUco detector from `armando_gazebo.launch.py`.
- Provides an incomplete `armando_aruco_target_node.cpp` scaffold.
- Provides an `armando_aruco_ik.launch.py` scaffold to complete.

The Gazebo camera topics are expected to be `/camera/image_raw` and
`/camera/camera_info`. Confirm the actual topic names and message types in the
running system before configuring the detector.

## Tasks

### 1. Design and place markers

1. Select an ArUco dictionary documented by the
   [official OpenCV ArUco documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).
   Record the dictionary name and why it is suitable.
2. Select at least two distinct marker IDs from that dictionary and record the
   physical marker side length used for pose estimation.
3. Generate the marker images from the selected dictionary and IDs. Do not
   substitute hand-drawn patterns or images from a different dictionary.
4. Add the markers to the Gazebo scene. Use at least two placements with
   different orientations or supports (for example, on an object and flat on a
   surface facing upward). Ensure that the printed marker is visible to the
   robot camera and is not hidden by the support geometry.
5. Keep marker identity, image, model visual, physical size, and detector
   configuration consistent. Document the placement and ID mapping.

### 2. Configure camera detection and transforms

1. Add and configure an ArUco detector using the chosen dictionary and marker
   size.
2. Connect the detector to the camera image and camera calibration topics.
   Use the correct camera optical frame and simulation time.
3. Ensure detections provide a pose for each visible marker and the expected TF
   transform(s). Use a consistent frame convention and document the marker
   frame names.
4. Do not use Gazebo's ground-truth object-pose topic as a substitute for
   camera-based detection.
5. Add any required ArUco ROS packages to the package manifest and ensure the
   development environment installs them.

### 3. Complete the pose adapter

Complete `armando_aruco_target_node.cpp` so that it:

1. Subscribes to the detector's marker output.
2. Selects the marker ID requested by a ROS parameter.
3. Uses the detected marker pose (or its TF) rather than the known Gazebo
   model pose.
4. Produces a `geometry_msgs/msg/PoseStamped` on `/target_pose`, with a valid
   timestamp and a frame that the KDL solver can transform to `base_link`.
5. Handles missing markers safely: do not republish a stale or fabricated
   target when the selected marker is absent.
6. Optionally supports a configurable approach offset. If implemented, state
   which marker axis the offset follows and its sign.

### 4. Integrate and launch the pipeline

Complete `armando_aruco_ik.launch.py` to start the detector and pose adapter,
and connect their parameters and topics. It must also start or include the
provided KDL solver, which consumes `/target_pose`.

The classwork must be runnable after the simulation has started with one
documented command. Make the target marker ID configurable at launch; do not
hard-code the only usable ID in the adapter.

## Acceptance criteria

The work is complete when all of the following can be demonstrated:

1. Gazebo starts with camera image and camera calibration messages available.
2. The selected dictionary and marker IDs are documented and match the marker
   images in the world.
3. With a marker visible, the detector reports its ID and a plausible pose.
4. The marker transform can be inspected with TF tools and is consistent with
   the detector's pose and frame configuration.
5. The adapter publishes `/target_pose` only for the selected visible ID; its
   frame is correct and its position changes consistently when the marker
   moves.
6. The KDL solver receives that message and moves toward reachable target
   positions. An unreachable target is rejected by the solver rather than
   bypassing its checks.
7. Hiding or removing the selected marker stops new target publications; the
   node must not invent a target or use Gazebo ground truth.
8. The documented launch and test commands work from a sourced ROS 2
   workspace.

## Suggested verification commands

Run these after sourcing ROS 2 and the workspace:

```bash
ros2 topic list
ros2 topic info /camera/image_raw
ros2 topic info /camera/camera_info
ros2 topic echo /target_pose
ros2 run tf2_tools view_frames
```

Use the frame name reported by the detector when checking a transform:

```bash
ros2 run tf2_ros tf2_echo base_link <marker_frame>
```

Also test the manual IK path independently of ArUco by publishing a known
reachable pose:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.2, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"
```

## Deliverables

- Updated Gazebo world with generated markers and documented dictionary, IDs,
  side length, and placement.
- Detector configuration and verified camera/topic/frame connections.
- Completed pose-adapter node and any required build/package dependencies.
- Completed launch file that composes detector, adapter, and KDL solver.
- A short test record showing detection, TF, `/target_pose`, successful IK
  motion, and behavior when the selected marker is not visible.

## Suggested assessment rubric

| Area | Points |
| --- | ---: |
| Marker dictionary/ID selection, generated assets, and visible world placement | 20 |
| Camera detection, calibration inputs, and verified TF/frame setup | 25 |
| Correct configurable marker selection and `PoseStamped` adapter behavior | 25 |
| Launch integration and successful end-to-end IK demonstration | 20 |
| Documentation, reproducibility, and missing-marker test | 10 |
| **Total** | **100** |
