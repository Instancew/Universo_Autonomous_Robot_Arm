# Simulation + game controller

Drive the 3 arm joints (base, shoulder, elbow) with an Xbox / PlayStation / generic USB or Bluetooth
controller in **RViz**, **Gazebo Classic 11** or **Gazebo Ignition Fortress**. No hardware is needed.
The same `arm_controller` (joint trajectory controller) is used everywhere, so MoveIt and the joystick
can both move the arm.

## 1. Install (once)

```bash
sudo apt install ros-humble-joy ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-moveit                                              # RViz + MoveIt
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control      # Gazebo Classic
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control                  # Gazebo Ignition
```

## 2. Build

```bash
cd ~/final_autonomous_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm robot_arm_moveit_config
source install/setup.bash
```

## 3. Run (plug in the controller first; check it with `ls /dev/input/js*`)

```bash
ros2 launch robot_arm simulation.launch.py                        # RViz + MoveIt (mock motors)
ros2 launch robot_arm simulation.launch.py simulator:=gazebo      # Gazebo Classic 11
ros2 launch robot_arm simulation.launch.py simulator:=ignition    # Gazebo Ignition Fortress
```

Options: `joystick:=false` (no controller), `rviz:=false` (Gazebo without RViz),
`mount_height:=0.4` (pedestal height in Gazebo, metres; the 8 mm base plate sits on top, so base_link is 8 mm higher).

Single pieces, if you prefer separate terminals:

```bash
ros2 launch robot_arm_moveit_config demo.launch.py
ros2 launch robot_arm gazebo.launch.py rviz:=true
ros2 launch robot_arm ignition.launch.py rviz:=true
ros2 launch robot_arm teleop.launch.py                     # add use_sim_time:=true with Gazebo
```

## 4. Controls (defaults, change in `config/joystick.yaml`)

| Input | Xbox | PlayStation | Action |
|---|---|---|---|
| button 4 (hold) | LB | L1 | **Enable** (dead-man). Release = arm holds position |
| axis 0 | left stick left/right | left stick left/right | base (left = counter-clockwise) |
| axis 1 | left stick up/down | left stick up/down | shoulder (up = raise upper arm) |
| axis 4 | right stick up/down | right stick up/down | elbow (up = raise forearm) |
| button 0 (+ enable) | A | Cross | go to home pose |

Speeds are the actuator speeds from the URDF (shoulder 0.042 rad/s = 2.4 deg/s, elbow 0.0355 rad/s = 2.0 deg/s;
carried over from the previous arm until the actuator mounting is confirmed), so the arm moves slowly on purpose. For a quicker simulation set `speed_scale` in `config/joystick.yaml`.
Joint limits are read from the URDF; the teleop never commands outside them.

If a stick moves the wrong joint or direction: run `ros2 topic echo /joy`, move the stick, and set the
matching `axis_*` number or flip the `scale_*` sign in `config/joystick.yaml`.

## 5. What is and is not simulated

- Simulated: the 3 arm joints with their real limits and speeds, the arm geometry and collisions (CAD meshes
  of the 30 Oct export), a 0.40 m pedestal (stands in for the rover deck) and a table with two target cubes
  (Gazebo worlds).
- Not simulated: the 3 gripper DOFs (the gripper is one rigid part in the URDF) and the linear actuators
  themselves (the joints are moved directly, at the speed the actuators allow).

## Files

| File | Purpose |
|---|---|
| `robot_arm/teleop/joy_teleop.py` | ROS node: game controller -> `/arm_controller/joint_trajectory` |
| `robot_arm/teleop/jog_logic.py` | jogging maths (deadzone, speeds, limits), unit-tested |
| `config/joystick.yaml` | button / stick mapping, speeds, home pose |
| `config/sim_controllers.yaml` | ros2_control controllers for Gazebo |
| `description/urdf/robot_arm_sim.urdf.xacro` | robot_arm.urdf + pedestal + ros2_control for mock / Gazebo / Ignition |
| `worlds/arm_test.world`, `worlds/arm_test.sdf` | Gazebo Classic / Ignition test scenes |
| `launch/simulation.launch.py` | one command for all simulators |
| `launch/gazebo.launch.py`, `launch/ignition.launch.py`, `launch/teleop.launch.py` | the individual pieces |
