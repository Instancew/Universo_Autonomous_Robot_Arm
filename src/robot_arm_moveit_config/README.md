# robot_arm_moveit_config (MoveIt 2, ROS 2 Humble)

Planning group `arm` = chain `base_link` -> `arm_tip` (base_joint, shoulder_joint, elbow_joint).
Mock hardware (ros2_control GenericSystem) - simulation only.

## Install + run
```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher-gui
cd ~/final_autonomous_arm_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm robot_arm_moveit_config
source install/setup.bash
ros2 launch robot_arm_moveit_config demo.launch.py
```

## Key settings
- `kinematics.yaml`: KDL with `position_only_ik: true` (3 DOF can only reach a tip position, not a full pose).
Values below are for the 30 Oct CAD (URDF limits: shoulder -1.419011 .. -0.197281 rad, elbow 0.231847 .. 2.328057 rad).
- `initial_positions.yaml`: start at "home" (0, -0.24, 2.29); 0 is outside the shoulder and elbow limits.
- `joint_limits.yaml`: the URDF speeds are shoulder 0.0420 and elbow 0.0355 rad/s (carried over from the previous arm
  until the actuator mounting is confirmed) -> real motions take 10-40 s. A higher `max_velocity` here is for faster
  simulation only.
- SRDF: with the 30 Oct meshes only joined links touch inside the limits (3324-pose check; closest other pair:
  upper arm to base plate about 35 mm) -> all 6 link pairs disabled.
  Add the rover deck / ground as planning-scene objects to stop the arm hitting them.
- Group states: `home` (0, -0.24, 2.29) = upper arm 76.25 deg above horizontal, elbow inner angle 48.79 deg (folded);
  `ready` (0, -0.5236, 1.5708) = upper arm 60.00 deg, elbow inner angle 90.00 deg.
