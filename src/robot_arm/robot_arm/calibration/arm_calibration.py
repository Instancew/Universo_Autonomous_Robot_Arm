"""Arm calibration: find each encoder's direction and zero offset so raw counts give URDF joint angles.

Reference = inclinometer (link angle above horizontal) - independent of the actuators' +/-3 mm tolerance.
  shoulder_joint = upper_arm_elevation - upper_arm_elevation_at_zero_deg
  elbow_joint    = shoulder_joint + forearm_elevation_at_zero_deg - forearm_elevation
  (both *_at_zero_deg values are in config/calibration.yaml; URDF of 30 Oct: 91.3029 / 84.7040 deg)
Measure along the top face of the extrusion, pointing OUTWARD from the joint (up = positive).
The extrusions are not exactly on the joint-to-joint lines; the *_at_zero_deg values absorb that offset.

Commands (see main()):
  status     which encoders are ready / what is still TBD
  joint      fit shoulder_joint or elbow_joint from >= 3 inclinometer poses
  base       fit base direction + zero from marked angles (home = 0 deg)
  base-cpr   measure base counts per turn (resolves OE-28 3 vs 7 PPR)
  endstop    cross-check encoder angle with the actuator fully retracted
  tip        compare FK (from encoder angles) with a tape-measured tip position
  simulate   run the whole procedure on simulated encoders (no hardware)
"""
import argparse
from dataclasses import dataclass, field
import datetime as dt
import math
from pathlib import Path
import sys
from typing import Dict, List, Sequence, Tuple

from ._config import config_dir, load_yaml, save_yaml
from .encoder_calibration import EncoderModel, load_encoders, model_from_entry, wrap_angle
from .encoder_sources import EncoderSource, ManualEntrySource, SimulatedSource, Stm32Source

TWO_PI = 2.0 * math.pi


# --------------------------------------------------------------------------- reference geometry
def reference_cfg(cfg: dict = None) -> dict:
    return (cfg or load_yaml('calibration.yaml'))


def shoulder_from_inclinometer(upper_elev_deg: float, ref: dict) -> float:
    """URDF shoulder_joint (rad) from the upper-arm inclinometer reading (deg above horizontal)."""
    return math.radians(upper_elev_deg - ref['reference']['upper_arm_elevation_at_zero_deg'])


def elbow_from_inclinometer(upper_elev_deg: float, fore_elev_deg: float, ref: dict) -> float:
    """URDF elbow_joint (rad) from upper-arm and forearm inclinometer readings (deg)."""
    r = ref['reference']
    return math.radians((upper_elev_deg - r['upper_arm_elevation_at_zero_deg'])
                        + r['forearm_elevation_at_zero_deg'] - fore_elev_deg)


def planar_tip(q_shoulder: float, q_elbow: float, ref: dict) -> Tuple[float, float]:
    """arm_tip [x, z] in the arm plane (m) - same model as the URDF.

    URDF convention: shoulder->elbow line elevation = 90 deg + q_shoulder,
                     elbow->arm_tip line elevation  = 90 deg + q_shoulder - q_elbow.
    """
    r = ref['reference']
    up = math.pi / 2 + q_shoulder
    fore = math.pi / 2 + q_shoulder - q_elbow
    sx, sz = r['shoulder_axis_xz']
    ex, ez = sx + r['upper_arm_length_m'] * math.cos(up), sz + r['upper_arm_length_m'] * math.sin(up)
    return ex + r['forearm_length_m'] * math.cos(fore), ez + r['forearm_length_m'] * math.sin(fore)


# --------------------------------------------------------------------------- fitting
@dataclass
class FitResult:
    joint: str
    direction: int
    zero_offset_counts: float
    residuals_deg: List[float]
    rms_deg: float
    max_deg: float
    spread_deg: float
    accepted: bool
    problems: List[str] = field(default_factory=list)

    def summary(self) -> str:
        s = (f'{self.joint}: direction {self.direction:+d}, zero_offset_counts {self.zero_offset_counts:.2f}, '
             f'RMS {self.rms_deg:.3f} deg, max {self.max_deg:.3f} deg over {len(self.residuals_deg)} poses '
             f'(spread {self.spread_deg:.1f} deg) -> {"ACCEPTED" if self.accepted else "REJECTED"}')
        return s + ''.join(f'\n  ! {p}' for p in self.problems)


def fit_zero_and_direction(model: EncoderModel, samples: Sequence[Tuple[float, float]],
                           min_poses: int = 3, min_spread_deg: float = 20.0,
                           max_rms_deg: float = 0.5) -> FitResult:
    """samples = [(raw_counts, reference_joint_angle_rad), ...]. Tries both directions, keeps the better."""
    if model.counts_per_rev is None:
        raise RuntimeError(f'{model.joint}: counts_per_rev unknown - set resolution_bits / pulses_per_rev first')
    cpr = model.counts_per_rev
    best = None
    for d in (+1, -1):
        zeros = [c - d * q * cpr / TWO_PI for c, q in samples]
        if model.kind == 'absolute':
            sx = sum(math.sin(z * TWO_PI / cpr) for z in zeros)
            cx = sum(math.cos(z * TWO_PI / cpr) for z in zeros)
            zero = (math.atan2(sx, cx) * cpr / TWO_PI) % cpr
        else:
            zero = sum(zeros) / len(zeros)
        trial = EncoderModel(model.joint, model.kind, cpr, d, zero)
        res = [math.degrees(wrap_angle(trial.counts_to_angle(c) - q)) for c, q in samples]
        rms = math.sqrt(sum(r * r for r in res) / len(res))
        if best is None or rms < best[2]:
            best = (d, zero, rms, res)
    d, zero, rms, res = best
    qs = [math.degrees(q) for _, q in samples]
    spread = max(qs) - min(qs) if qs else 0.0
    problems = []
    if len(samples) < min_poses:
        problems.append(f'only {len(samples)} poses, need >= {min_poses}')
    if spread < min_spread_deg:
        problems.append(f'poses span {spread:.1f} deg, need >= {min_spread_deg} deg (direction not reliable)')
    if rms > max_rms_deg:
        problems.append(f'RMS {rms:.3f} deg > {max_rms_deg} deg - re-measure (encoder slipping? wrong link measured?)')
    return FitResult(model.joint, d, zero, res, rms, max(abs(r) for r in res), spread, not problems, problems)


def measure_counts_per_rev(count_start: float, count_end: float, turns: float = 1.0) -> float:
    """Base: turn the base exactly `turns` full turns (to the same mark) and read counts before/after."""
    return abs(count_end - count_start) / turns


# --------------------------------------------------------------------------- saving / report
def apply_fit(fit: FitResult, enc_cfg: dict = None, save: bool = True) -> dict:
    enc_cfg = enc_cfg or load_yaml('encoders.yaml')
    e = enc_cfg['encoders'][fit.joint]
    e['direction'] = fit.direction
    e['zero_offset_counts'] = round(fit.zero_offset_counts, 2)
    e['calibrated'] = True
    e['calibrated_on'] = dt.date.today().isoformat()
    e['calibration_rms_deg'] = round(fit.rms_deg, 4)
    if save:
        save_yaml('encoders.yaml', enc_cfg)
    return enc_cfg


def write_report(lines: List[str], report_dir: Path = None) -> Path:
    if report_dir is None:
        ws = config_dir().resolve().parents[2]          # <ws>/src/robot_arm/config -> <ws>
        report_dir = ws / 'deliveries' / 'manual'
        if not report_dir.is_dir():
            report_dir = Path.cwd()
    report_dir.mkdir(parents=True, exist_ok=True)
    path = report_dir / f'calibration_report_{dt.datetime.now():%Y%m%d_%H%M%S}.md'
    path.write_text('# Encoder calibration report\n\n' + '\n'.join(lines) + '\n', encoding='utf-8')
    return path


# --------------------------------------------------------------------------- interactive procedures
def _ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print('  enter a number')


def _source(name: str) -> EncoderSource:
    return {'manual': ManualEntrySource, 'stm32': Stm32Source}[name]()


def run_joint(joint: str, source: EncoderSource, n_poses: int, ref: dict) -> FitResult:
    enc = load_encoders()[joint]
    p = ref['procedure']
    print(f'\nCalibrating {joint}: {n_poses} poses spread over >= {p["min_angle_spread_deg"]} deg.')
    print('Inclinometer on the top face of the extrusion, pointing outward from the joint; up = positive.')
    samples = []
    for i in range(n_poses):
        input(f'\nPose {i + 1}/{n_poses}: move the arm, let it settle, press Enter')
        up = _ask_float('  upper-arm inclinometer (deg): ')
        if joint == 'shoulder_joint':
            q = shoulder_from_inclinometer(up, ref)
        else:
            fore = _ask_float('  forearm inclinometer (deg): ')
            q = elbow_from_inclinometer(up, fore, ref)
        c = source.read_counts()[joint]
        samples.append((c, q))
        print(f'  -> reference {joint} = {math.degrees(q):.3f} deg, counts {c}')
    return fit_zero_and_direction(enc, samples, p['min_poses_per_joint'], p['min_angle_spread_deg'],
                                  p['max_rms_residual_deg'])


def run_base(source: EncoderSource, angles_deg: Sequence[float], ref: dict) -> FitResult:
    enc = load_encoders()['base_joint']
    print('\nCalibrating base_joint. Pose 1 = home (0 deg). Then turn the base CCW (seen from above) to each mark.')
    samples = []
    for a in angles_deg:
        input(f'\nTurn base to {a:.1f} deg, press Enter')
        samples.append((source.read_counts()['base_joint'], math.radians(a)))
    p = ref['procedure']
    return fit_zero_and_direction(enc, samples, p['min_poses_per_joint'], p['min_angle_spread_deg'],
                                  p['max_rms_residual_deg'])


def check_endstops(angles: Dict[str, float], ref: dict) -> List[str]:
    """angles = encoder-derived joint angles (rad) with the actuator(s) fully retracted."""
    r = ref['reference']
    exp = {'shoulder_joint': r.get('shoulder_at_full_retraction_deg'), 'elbow_joint': r.get('elbow_at_full_retraction_deg')}
    tol = {'shoulder_joint': r['end_stop_tolerance_deg']['shoulder'], 'elbow_joint': r['end_stop_tolerance_deg']['elbow']}
    out = []
    for j, a in angles.items():
        if exp[j] is None or tol[j] is None:
            out.append(f'{j}: encoder {math.degrees(a):.3f} deg - end stop not known yet (null in calibration.yaml)')
            continue
        diff = math.degrees(a) - exp[j]
        ok = abs(diff) <= tol[j]
        out.append(f'{j}: encoder {math.degrees(a):.3f} deg vs expected {exp[j]:.3f} deg at full retraction '
                   f'-> diff {diff:+.3f} deg ({"OK" if ok else "CHECK"}, tolerance +/-{tol[j]} deg)')
    return out


def check_tip(q_base: float, q_shoulder: float, q_elbow: float, measured_xyz_m: Sequence[float], ref: dict) -> str:
    x, z = planar_tip(q_shoulder, q_elbow, ref)
    y0 = ref['reference'].get('arm_plane_offset_y_m', 0.0)   # arm centre plane is beside the yaw axis
    fk = (x * math.cos(q_base) - y0 * math.sin(q_base), x * math.sin(q_base) + y0 * math.cos(q_base), z)
    err = math.dist(fk, measured_xyz_m) * 1000
    return (f'FK tip ({fk[0]*1000:.1f}, {fk[1]*1000:.1f}, {fk[2]*1000:.1f}) mm vs measured '
            f'({measured_xyz_m[0]*1000:.1f}, {measured_xyz_m[1]*1000:.1f}, {measured_xyz_m[2]*1000:.1f}) mm '
            f'-> error {err:.1f} mm')


# --------------------------------------------------------------------------- simulation (no hardware)
def simulate(seed: int = 3, inclinometer_noise_deg: float = 0.1, save: bool = False) -> List[FitResult]:
    """End-to-end dry run with hidden true offsets; proves the procedure recovers them."""
    import random
    rng = random.Random(seed)
    ref = reference_cfg()
    truth = {
        'shoulder_joint': dict(kind='absolute', counts_per_rev=16384.0, direction=-1, zero_offset_counts=11873.0),
        'elbow_joint': dict(kind='absolute', counts_per_rev=16384.0, direction=+1, zero_offset_counts=402.0),
        'base_joint': dict(kind='incremental', counts_per_rev=16380.0, direction=+1, zero_offset_counts=-5210.0),
    }
    src = SimulatedSource(truth, seed=seed)
    enc_cfg = {'encoders': {
        'shoulder_joint': {'type': 'absolute', 'resolution_bits': 14},
        'elbow_joint': {'type': 'absolute', 'resolution_bits': 14},
        'base_joint': {'type': 'incremental', 'pulses_per_rev': 7, 'quadrature_multiplier': 4, 'gear_ratio': 585}}}
    p = ref['procedure']
    results = []
    # (shoulder, elbow) poses in degrees, inside the URDF limits (shoulder -81.3..-11.3, elbow 13.3..133.4)
    for joint, poses in (('shoulder_joint', [(-15, 40), (-28, 60), (-42, 80), (-57, 100)]),
                         ('elbow_joint', [(-30, 20), (-30, 55), (-30, 90), (-30, 125)])):
        model = model_from_entry(joint, enc_cfg['encoders'][joint])
        samples = []
        for qs_deg, qe_deg in poses:
            qs, qe = math.radians(qs_deg), math.radians(qe_deg)
            src.set_angles(shoulder_joint=qs, elbow_joint=qe)
            r = ref['reference']
            up = r['upper_arm_elevation_at_zero_deg'] + qs_deg + rng.gauss(0, inclinometer_noise_deg)
            fore = r['forearm_elevation_at_zero_deg'] + qs_deg - qe_deg + rng.gauss(0, inclinometer_noise_deg)
            q_ref = shoulder_from_inclinometer(up, ref) if joint == 'shoulder_joint' else elbow_from_inclinometer(up, fore, ref)
            samples.append((src.read_counts()[joint], q_ref))
        results.append(fit_zero_and_direction(model, samples, p['min_poses_per_joint'],
                                              p['min_angle_spread_deg'], p['max_rms_residual_deg']))
    model = model_from_entry('base_joint', enc_cfg['encoders']['base_joint'])
    samples = []
    for a in (0, 90, 180, 270):
        src.set_angles(base_joint=math.radians(a))
        samples.append((src.read_counts()['base_joint'], math.radians(a + rng.gauss(0, 0.2))))
    results.append(fit_zero_and_direction(model, samples, p['min_poses_per_joint'], p['min_angle_spread_deg'],
                                          p['max_rms_residual_deg']))
    for f in results:
        t = truth[f.joint]
        print(f.summary())
        print(f'  true: direction {t["direction"]:+d}, zero {t["zero_offset_counts"]:.2f}')
    return results


# --------------------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description='Encoder calibration for robot_arm (Phase 1).')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status')
    pj = sub.add_parser('joint')
    pj.add_argument('joint', choices=['shoulder_joint', 'elbow_joint'])
    pj.add_argument('--poses', type=int, default=4)
    pj.add_argument('--source', choices=['manual', 'stm32'], default='manual')
    pb = sub.add_parser('base')
    pb.add_argument('--angles', type=float, nargs='+', default=[0, 90, 180, 270])
    pb.add_argument('--source', choices=['manual', 'stm32'], default='manual')
    pc = sub.add_parser('base-cpr')
    pc.add_argument('--turns', type=float, default=1.0)
    pc.add_argument('--source', choices=['manual', 'stm32'], default='manual')
    pe = sub.add_parser('endstop')
    pe.add_argument('--source', choices=['manual', 'stm32'], default='manual')
    pt = sub.add_parser('tip')
    pt.add_argument('xyz_mm', type=float, nargs=3, help='tape-measured arm_tip in base_link (mm)')
    pt.add_argument('--source', choices=['manual', 'stm32'], default='manual')
    sub.add_parser('simulate')
    a = ap.parse_args(argv)

    from .encoder_calibration import status_report
    if a.cmd == 'status':
        print(status_report())
        return 0
    if a.cmd == 'simulate':
        return 0 if all(f.accepted for f in simulate()) else 1

    ref = reference_cfg()
    src = _source(a.source)
    if a.cmd in ('joint', 'base'):
        fit = run_joint(a.joint, src, a.poses, ref) if a.cmd == 'joint' else run_base(src, a.angles, ref)
        print('\n' + fit.summary())
        if fit.accepted and input('Save to config/encoders.yaml? [y/N] ').strip().lower() == 'y':
            apply_fit(fit)
            print('saved; report:', write_report([fit.summary()]))
        return 0 if fit.accepted else 1
    if a.cmd == 'base-cpr':
        input('Put the base on its mark, press Enter'); c0 = src.read_counts()['base_joint']
        input(f'Turn the base exactly {a.turns} turn(s) CCW back to the mark, press Enter'); c1 = src.read_counts()['base_joint']
        cpr = measure_counts_per_rev(c0, c1, a.turns)
        print(f'measured {cpr:.1f} counts per base turn  (7 PPR x4 x585 = 16380 ; 3 PPR x4 x585 = 7020)')
        return 0
    enc = load_encoders()
    if a.cmd == 'endstop':
        input('Fully retract BOTH actuators (until they stop), press Enter')
        c = src.read_counts()
        angles = {j: enc[j].counts_to_angle(c[j]) for j in ('shoulder_joint', 'elbow_joint')}
        for line in check_endstops(angles, ref):
            print(line)
        return 0
    if a.cmd == 'tip':
        c = src.read_counts()
        q = {j: enc[j].counts_to_angle(c[j]) for j in ('base_joint', 'shoulder_joint', 'elbow_joint')}
        print(check_tip(q['base_joint'], q['shoulder_joint'], q['elbow_joint'], [v / 1000 for v in a.xyz_mm], ref))
        return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())
