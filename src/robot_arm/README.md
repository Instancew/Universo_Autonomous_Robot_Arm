# Autonomous Robot Arm — ROS 2 Humble Project Structure

Not a hardware-ready controller yet: nothing here commands the real motors.

Status (5 Nov 2026):
- Implemented and tested: URDF/meshes (`description/`), calibration (`calibration/`), kinematics (`kinematics/`), joystick teleop and simulation (`teleop/`, `launch/`), autonomy / mission / motion skeleton classes.
- Placeholders: `hardware/`, `perception/`, `safety/` (emergency stop, fault handler, safety manager), `logging_tools/`, `launch/bringup.launch.py`, `calibration/camera_calibration.py`.
- Joint conventions differ between `kinematics/` and the URDF: see `docs/conventions.md`.

## Target system
- ROS 2 Humble
- Jetson Orin Nano Super
- 3 arm DOFs: base rotation, shoulder rotation, elbow rotation
- Shoulder and elbow driven by linear actuators
- CAN-based hardware communication (exact topology/protocol to be confirmed)
- Orbbec Gemini 2 perception
- Manually operated gripper initially, with autonomous gripper components reserved for later

## Workspace layout
Place this package folder in `~/robot_arm_ws/src/`, then adapt package metadata and dependencies to the actual implementation. Build with `colcon build` only after the package metadata and dependencies are configured for your ROS 2 environment.

## Important safety notes
- Example configuration values are `null` or explicitly marked as placeholders.
- Do not connect placeholder code to real hardware.
- Confirm actuator geometry, stroke, joint limits, CAN IDs, motor-controller command formats, encoder scaling, and emergency-stop behavior before physical testing.
- A URDF represents the robot's joint kinematics; actuator linkage conversion is handled by a separate module unless the mechanism is explicitly modeled.
- Collision checking is only as accurate as the URDF/meshes and planning-scene configuration.
- Test first with unit tests and simulation, then with hardware disabled/unpowered, then under a controlled low-speed test procedure.

## Suggested development order
1. Finalize CAD measurements and actuator conversion tests.
2. Complete URDF/Xacro and verify joint axes, origins, limits, and collision meshes.
3. Configure MoveIt 2 planning group, IK, OMPL planner, and trajectory processing.
4. Establish CAN communication and validate feedback independently.
5. Implement the ROS 2 hardware interface and safe command/feedback mapping.
6. Test low-speed single-joint motion with hard limits and emergency stop verified.
7. Integrate Gemini 2 perception and base-frame target transforms.
8. Add autonomous gripper operation only after the arm motion pipeline is stable.

## Folder guide
- `robot_arm/`: Python package source.
- `description/`: URDF/Xacro, meshes, RViz config.
- `moveit_config/`: MoveIt configuration placeholders.
- `launch/`: launch files to be implemented.
- `config/`: hardware, geometry, safety, camera, and planning parameters.
- `test/`: unit-test starting points.
- `docs/`: architecture and integration notes.
