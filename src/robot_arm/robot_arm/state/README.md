# State package

`StateManager` is the in-process source of truth for system, arm, joint, gripper,
and command-authority state. The arm currently models the three independently
controlled joints using the URDF names `base_joint`, `shoulder_joint`, and
`elbow_joint`; the gripper remains a separate state object because it is
currently manually operated.

## Important semantics

- A joint's numeric `position` default is not evidence of a received encoder
  sample. Check `position_valid` before using it.
- A joint is not considered calibrated until its calibration component marks
  it calibrated. The arm is not considered homed until every configured joint
  has valid feedback and is marked calibrated, then `mark_arm_homed()` succeeds.
- `state_ready_for_autonomy` requires Jetson, STM32, CAN, and camera connection
  flags, valid position feedback and calibration for all arm joints, a homed
  arm, and no software error/E-stop latch. Connection flags must be updated by
  the relevant integration nodes; defaults are deliberately false.
- `ControlAuthority` is a thread-safe, in-process exclusive-owner primitive.
  It only arbitrates command sources that share the same instance. Separate
  ROS 2 processes need a ROS-level arbiter/service/action or equivalent shared
  authority mechanism before this can enforce ownership system-wide.
- The software E-stop latch does not stop motors electrically and is not a
  substitute for the physical E-stop/safety chain.

Use `StateManager` update methods for feedback writes. The contained state
objects remain exposed for compatibility, so consumers should avoid mutating
them directly when concurrent updates are possible.
