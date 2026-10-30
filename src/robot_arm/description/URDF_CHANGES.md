# robot_arm.urdf — what changed vs. the Onshape export (Robot_ArmNew, 30 Oct 2026, onshape-to-robot)

CAD geometry is untouched: the 4 CAD links, all part meshes, mesh origins and inertials are copied
verbatim. Every CAD link pose in this URDF equals the export's to 1e-11 m at the reference pose and to
0.005 mm over the whole joint range (the export's own axes are rounded to 6 digits).

| # | Change | Why |
|---|--------|-----|
| 1 | Mesh paths `package://assets/` → `package://robot_arm/description/meshes/` | `assets` is not a ROS 2 package |
| 2 | New `base_link` on the yaw axis at the base-plate part origin, z up, x = direction the arm reaches | the export root frame is tilted 0.84° (CAD alignment noise); the yaw axis is exactly normal to the base plate and passes through its origin |
| 3 | `360` (revolute ±180°) → `base_joint` (continuous) | slip ring: unlimited rotation. Sign: base_joint + = counter-clockwise from above = export `360` − |
| 4 | `l1End` → `shoulder_joint`, `L2` → `elbow_joint`, rebuilt in clean frames (`upper_arm_link`, `forearm_link`) | names used by MoveIt, servo and teleop; clean zero conventions (below) |
| 5 | New `arm_tip` frame = centre of the gripper mounting face (gripper base-plate origin) | IK target |
| 6 | Shoulder / elbow effort and velocity kept from the previous model | the export has placeholders (10 / 10) |

The gripper has no joints in this export (one rigid link `gripper__1` together with the forearm).

## Frames
- `base_link`: x forward, y left (along the shoulder axis), z up; origin on the yaw axis at the base-plate part origin,
  which is the **top face** of the 8 mm base plate. The plate's bottom face (rover mounting face) is at z = −8.00 mm.
- `arm_tip`: x along the forearm extrusion (outward), y left, z = x × y.

## Joint conventions
- `shoulder_joint`: 0 = upper arm (shoulder-axis → elbow-axis line) vertical, negative = leaning forward.
  Upper-arm elevation = 90° + q_shoulder. Export value: l1End = q_shoulder + 11.3033°.
- `elbow_joint`: 0 = arm straight (elbow → arm_tip line in line with the upper arm), positive = forearm folds down.
  Forearm (elbow → arm_tip line) elevation = 90° + q_shoulder − q_elbow. Elbow inner angle = 180° − q_elbow.
  Export value: L2 = q_elbow − 133.3878°.

## Geometry (base_link, mm)
| Item | Value |
|------|-------|
| Shoulder axis | x 103.1763, z 64.5001 |
| Shoulder axis → elbow axis (L1) | 549.6141 |
| Elbow axis → arm_tip (L2, in the arm plane) | 476.6496 |
| Arm centre plane (arm_tip y at base_joint = 0) | −1.9990 (2.00 mm to the right of the yaw axis) |
| Upper-arm extrusion vs. shoulder→elbow line | top face reads 91.3029° when q_shoulder = 0 |
| Forearm extrusion vs. elbow→arm_tip line | top face reads 84.7040° when both joints = 0 |
| Shoulder actuator base pin | x −134.3009, z 146.4861 |
| Upper-arm pins at q = 0 (near / far) | (66.6603, 328.5619) / (65.9781, 358.5541) |

Tip (exact vs. URDF to 1e-9 m):
x' = 103.1763 + 549.6141·cos(90° + q_s) + 476.6496·cos(90° + q_s − q_e), z = 64.5001 + (same with sin);
x = x'·cos(q_b) − y0·sin(q_b), y = x'·sin(q_b) + y0·cos(q_b), with y0 = −1.9990.

## Limits
| Joint | Lower | Upper | Export mate limit |
|-------|-------|-------|-------------------|
| shoulder_joint | −1.419011 rad (−81.303°) | −0.197281 rad (−11.303°) | l1End −1.22173 … 0 |
| elbow_joint | 0.231847 rad (13.284°) | 2.328057 rad (133.388°) | L2 −2.09621 … 0 |

Upper-arm elevation 8.70° … 78.70°; elbow inner angle 46.61° … 166.72°.
Shoulder actuator (near upper-arm pin): 313.24 mm at the upper limit, 493.02 mm at the lower limit
(full retraction 300 mm at q_shoulder −7.661°, full extension 500 mm at −86.768°).

## Checked with the meshes (34 STL files from the same Onshape export, in `description/meshes/`)
- Base plate: 340 × 190 × 8 mm; base_link is its top face, the bottom face is 8.00 mm lower.
  Thrust-bearing housing on top: Ø146 × 29.5 mm (base yaw bearing face 29.50 mm above base_link).
- arm_tip is the centre of the forearm extrusion end face (within 0.1 mm). Forearm 3030 extrusion: 560.0 mm long.
  Upper-arm 3030 extrusions: 2 × 500 mm.
- Shoulder axle (Ø12 × 140 mm) lies on the shoulder axis within 0.001 mm; elbow bearings on the elbow axis within 0.001 mm.
  One turntable bearing bore is 0.25 mm and one upper-arm bearing bore 0.35 mm off the axle line (CAD placement).
- Self-collision over the joint limits (3324 sampled poses): only joined links touch (base plate–turntable,
  turntable–upper arm, upper arm–forearm). Closest approach of the others: upper arm to base plate about 35 mm,
  forearm/gripper to base plate about 237 mm, forearm/gripper to turntable about 312 mm.

## Still open
- Elbow actuator: no pair of mounting holes found in the CAD gives a 300–500 mm actuator over the elbow range
  (forearm lever hole 96.05 mm from the elbow axis + upper-arm pin gives 191–349 mm). The CAD team must say
  which holes the elbow actuator uses.
