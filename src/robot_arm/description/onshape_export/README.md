# Raw Onshape export (reference only)

`Robot_ArmNew_raw_export.urdf` is the unmodified onshape-to-robot output (robot name `Robot_ArmNew`,
joints `360`, `l1End`, `L2`, mesh paths `package://assets/...`).

Do NOT copy it over `../urdf/robot_arm.urdf`. The rest of the workspace (MoveIt, ros2_control, teleop, tests,
calibration) needs the cleaned URDF: frames `base_link` and `arm_tip`, joints `base_joint`, `shoulder_joint`,
`elbow_joint`, and mesh paths `package://robot_arm/description/meshes/...` (see `../URDF_CHANGES.md`).

Checked on 5 Nov 2026: this export has the same geometry as the cleaned URDF (link poses agree to 0.0003 mm
over the whole joint range, all 34 meshes identical). This folder is not installed by setup.py.
