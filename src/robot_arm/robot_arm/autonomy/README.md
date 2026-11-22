# Autonomy package

The autonomy lifecycle is intentionally gated:

`IDLE -> INITIALIZING -> HOMING -> READY -> EXECUTING_MISSION`

Mission failures move to `RECOVERING`; failed recovery moves to `ERROR`.
`EMERGENCY_STOP` is latched and has no ordinary outgoing transition. Clearing
it requires an explicit `manual_reset_confirmed=True` acknowledgement.

## Startup integration contract

1. Transition `IDLE -> INITIALIZING`.
2. Verify required software components and communication status.
3. Transition `INITIALIZING -> HOMING`.
4. Run the actual homing procedure in the responsible calibration/homing node.
5. Update all joint feedback and calibration flags through `StateManager`.
6. Construct `MissionManager(state_machine=..., state_manager=...)` and call
   `complete_homing(success=True)` only after homing is physically verified.
   This method checks `StateManager`, confirms the state-machine homing gate,
   and transitions `HOMING -> READY` as one coordinated operation.
7. Mission start checks the full state-readiness predicate and uses that manager's
   shared in-process `ControlAuthority` instance.

`MissionManager.start_mission()` remains synchronous for compatibility. The
mission callback must return a truthy value only after successful completion.
The current mission callback interface is not preemptible; mission and motion
execution components must cooperate with cancellation/E-stop and stop issuing
commands when an E-stop or cancellation is received.

## Integration limitations

- `ControlAuthority` is in-process only. Teleop, MoveIt Servo and autonomy run
  in separate ROS 2 nodes/processes in the deployed architecture. A ROS-level
  command arbiter and routing of all command sources through it are required
  before command ownership is enforced system-wide.
- The state machine records that homing has been verified; it does not perform
  homing or choose a home reference. The incremental base encoder's physical
  home reference remains a hardware/calibration decision.
- The software E-stop is a logical latch only. Hardware safety circuitry and
  the motor/driver stop path must implement the physical stop.
- Recovery callbacks must be supplied by the integration layer. The manager
  bounds attempts and coordinates lifecycle/ownership; it does not invent a
  safe recovery trajectory.
