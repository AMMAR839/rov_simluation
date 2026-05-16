# Attribution

This model is a workspace-local integration of small, useful assets from open
source underwater robotics projects.

- `meshes/bluerov2.dae`, T200 propeller model references: copied from
  `evan-palmer/blue`, MIT License.
- `meshes/grip_claw.stl` and `meshes/arm_link5.stl`: copied from
  `AlePuglisi/ROV-Ricketts-ros2`, MIT License, and retained as local reference
  assets. The active gripper visuals in this workspace are custom SDF geometry
  so the claw can align cleanly with the BlueROV2-style body.

The ROS nodes, Gazebo world integration, gripper attach logic, camera setup, and
KKI mission layout remain part of this `WS_ROV` workspace.
