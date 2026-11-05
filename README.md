# Universo_Autonomous_Robot_Arm

ROS 2 Humble workspace for the 3-DOF autonomous arm (base, shoulder, elbow) with a manual 3-DOF gripper.

## Packages (`src/`)
- `robot_arm`: URDF and meshes, calibration, kinematics, joystick teleop, simulation launch files, autonomy skeletons.
- `robot_arm_moveit_config`: MoveIt 2 configuration (mock hardware) and MoveIt Servo launch.
- `universo_bringup`: Xbox Cartesian teleop through MoveIt Servo.

## Build
```bash
cd ~/final_autonomous_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm robot_arm_moveit_config universo_bringup
source install/setup.bash
```

## Tests
```bash
cd ~/final_autonomous_arm_ws/src/robot_arm
python3 -m pytest test
```

## Run (simulation)
```bash
ros2 launch robot_arm simulation.launch.py
ros2 launch universo_bringup xbox_teleop.launch.py
```
More options: `src/robot_arm/docs/simulation.md`.

## Read first
- `src/robot_arm/description/URDF_CHANGES.md`: frames, joint conventions, geometry, limits.
- `src/robot_arm/docs/conventions.md`: URDF joint convention vs the convention used by `kinematics/`.
- `src/robot_arm/description/onshape_export/`: raw CAD export, reference only.
