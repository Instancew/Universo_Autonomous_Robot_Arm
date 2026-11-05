# Joint conventions

Two conventions exist in this workspace. Mixing them moves the arm to the wrong pose.

| | URDF convention | Export convention |
|---|---|---|
| Used by | `robot_arm.urdf`, MoveIt, ros2_control, `joy_teleop`, `joints.yaml`, calibration | `kinematics/fk.py`, `ik.py`, `jacobian.py` |
| Joint names | `base_joint`, `shoulder_joint`, `elbow_joint` | `360`, `l1End`, `L2` |
| Base | continuous, + = counter-clockwise seen from above | `360 = -base_joint`, wrapped to [-pi, pi) by `ik.py` |
| Shoulder | 0 = upper arm vertical; limits -1.419011 .. -0.197281 rad | `l1End = shoulder_joint + 0.197281`; limits -1.22173 .. 0 |
| Elbow | 0 = arm straight; limits 0.231847 .. 2.328057 rad | `L2 = elbow_joint - 2.328057`; limits -2.09621 .. 0 |

Conversion code: `robot_arm/autonomy/joint_convention.py` (`urdf_to_export`, `export_to_urdf`,
`export_to_urdf_dict`, `nearest_base_angle`). Tests: `test/test_joint_convention.py`.

Check done on 5 Nov 2026: `fk.py` with converted angles equals the URDF `arm_tip` to about 0.003 mm over
2000 random poses; fed with URDF angles unconverted, the error reaches 1.47 m.

Rule: IK output from `ik.py` must pass through `export_to_urdf` before it is sent to `arm_controller`.

## Frames
- `base_link`: on the base yaw axis at the top face of the 8 mm base plate; x forward, y left, z up.
- `arm_tip`: centre of the gripper mounting face; this is the IK target (position only).
