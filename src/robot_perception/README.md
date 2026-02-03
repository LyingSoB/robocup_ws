# Robot Perception

## RealSense bringup

Launch the Intel RealSense driver (via `realsense2_camera`):

```bash
ros2 launch robot_perception realsense_camera.launch.py
```

This publishes:
- `/camera/color/image_raw`
- `/camera/depth/image_raw`

## RViz2 visualization

Open RViz2 with the provided configuration:

```bash
rviz2 -d $(ros2 pkg prefix robot_perception)/share/robot_perception/rviz/perception.rviz
```

## Validation checklist

1. **Camera TF correctness**
   - Ensure `camera_link` is visible under the TF tree in RViz2.
   - Confirm the camera frame is rigidly attached to `base_link` and does not drift.

2. **Object pose accuracy**
   - Verify `/arm/object_pose_camera` and `/arm/object_pose_base` appear in RViz2.
   - The base-frame pose should move consistently with the camera when you move the robot.

3. **No arm movement yet**
   - Ensure `/arm/object_reachable` only changes the gate state and no arm commands are issued.

