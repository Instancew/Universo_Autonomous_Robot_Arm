"""Drive the arm with a game controller (Xbox / PlayStation / generic) through ros2_control.

Works the same in RViz (mock hardware), Gazebo Classic, Gazebo Ignition and later the real arm,
because it only talks to the joint trajectory controller:
  in : /joy (sensor_msgs/Joy), /joint_states, /robot_description (joint limits + speeds)
  out: /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

Hold the ENABLE button (dead-man, default LB / L1) to move. Release it and the arm holds position.
Default sticks: left X = base, left Y = shoulder (up = raise arm), right Y = elbow (up = raise forearm).
ENABLE + HOME button (default A / Cross) = go to the home pose.
All mappings are parameters (config/joystick.yaml).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .jog_logic import JOINTS, apply_deadzone, clamp, move_duration, parse_urdf_limits, step_targets


def _duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')
        p = self.declare_parameter
        self.topic = p('controller_topic', '/arm_controller/joint_trajectory').value
        self.rate = float(p('rate_hz', 20.0).value)
        self.enable_button = int(p('enable_button', 4).value)
        self.home_button = int(p('home_button', 0).value)
        self.axes = {'base_joint': int(p('axis_base', 0).value),
                     'shoulder_joint': int(p('axis_shoulder', 1).value),
                     'elbow_joint': int(p('axis_elbow', 4).value)}
        self.scales = {'base_joint': float(p('scale_base', 1.0).value),
                       'shoulder_joint': float(p('scale_shoulder', 1.0).value),
                       'elbow_joint': float(p('scale_elbow', -1.0).value)}
        self.deadzone = float(p('deadzone', 0.1).value)
        self.speed_scale = float(p('speed_scale', 1.0).value)
        self.margin = float(p('limit_margin_rad', 0.01).value)
        self.base_cap = float(p('base_max_velocity', 0.5).value)
        self.home = dict(zip(JOINTS, [float(v) for v in p('home_position', [0.0, -0.24, 2.29]).value]))

        self.specs = None
        self.current = None
        self.targets = None
        self.joy = None
        self.was_enabled = False
        self.home_was_pressed = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/robot_description', self._on_description, latched)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.pub = self.create_publisher(JointTrajectory, self.topic, 10)
        self.create_timer(1.0 / self.rate, self._on_timer)
        self.get_logger().info(
            f'Joystick teleop ready -> {self.topic}. Hold button {self.enable_button} to move; '
            f'axes base/shoulder/elbow = {self.axes["base_joint"]}/{self.axes["shoulder_joint"]}/'
            f'{self.axes["elbow_joint"]}; button {self.home_button} = home.')

    # ------------------------------------------------------------------ inputs
    def _on_description(self, msg: String):
        try:
            self.specs = parse_urdf_limits(msg.data, JOINTS, {'base_joint': self.base_cap})
            self.get_logger().info('Joint limits loaded: ' + ', '.join(
                f'{s.name} [{s.lower}, {s.upper}] v={s.max_velocity:.4f}' for s in self.specs.values()))
        except Exception as exc:  # malformed description should not kill the node
            self.get_logger().error(f'Could not read joint limits from /robot_description: {exc}')

    def _on_joint_states(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.current = {j: pos[j] for j in JOINTS}

    def _on_joy(self, msg: Joy):
        self.joy = msg

    # ------------------------------------------------------------------ helpers
    def _button(self, index: int) -> bool:
        return 0 <= index < len(self.joy.buttons) and self.joy.buttons[index] == 1

    def _axis(self, index: int) -> float:
        return self.joy.axes[index] if 0 <= index < len(self.joy.axes) else 0.0

    def _send(self, positions: dict, seconds: float):
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(positions[j]) for j in JOINTS]
        point.time_from_start = _duration(seconds)
        traj.points = [point]
        self.pub.publish(traj)

    # ------------------------------------------------------------------ main loop
    def _on_timer(self):
        if self.specs is None or self.current is None or self.joy is None:
            missing = [n for n, v in (('/robot_description', self.specs), ('/joint_states', self.current),
                                      ('/joy', self.joy)) if v is None]
            self.get_logger().warn('Waiting for ' + ', '.join(missing), throttle_duration_sec=5.0)
            return

        enabled = self._button(self.enable_button)
        if not enabled:
            if self.was_enabled:                      # just released: hold where the arm is
                self._send(self.current, 2.0 / self.rate)
            self.was_enabled = False
            self.home_was_pressed = False
            return

        if not self.was_enabled:                      # just pressed: start from the measured pose
            self.targets = {j: clamp(self.current[j], self.specs[j], self.margin) for j in JOINTS}
            self.was_enabled = True

        home_pressed = self._button(self.home_button)
        if home_pressed and not self.home_was_pressed:
            goal = {j: clamp(self.home[j], self.specs[j], self.margin) for j in JOINTS}
            seconds = move_duration(self.current, goal, self.specs)
            self._send(goal, seconds)
            self.targets = goal
            self.get_logger().info(f'Going home ({seconds:.1f} s)')
        self.home_was_pressed = home_pressed
        if home_pressed:
            return

        commands = {j: self.scales[j] * apply_deadzone(self._axis(self.axes[j]), self.deadzone) for j in JOINTS}
        if all(c == 0.0 for c in commands.values()):
            return
        self.targets = step_targets(self.targets, commands, self.specs, 1.0 / self.rate,
                                    self.speed_scale, self.margin)
        self._send(self.targets, 3.0 / self.rate)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
