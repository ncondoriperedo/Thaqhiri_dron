"""Create a compact, shareable report from completed SIL run directories."""

import argparse
import colorsys
import json
from pathlib import Path
import shutil

import numpy as np
from thaiquiri_webots.sil_metrics import trajectory_error


def polyline(points, color):
    coordinates = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
    return f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="2"/>'


def plot_trial(run, output):
    truth = np.loadtxt(run/'truth.csv', delimiter=',', skiprows=1, ndmin=2)
    odom = np.loadtxt(run/'odometry.csv', delimiter=',', skiprows=1, ndmin=2)
    flight = np.loadtxt(run/'flight.csv', delimiter=',', skiprows=1, ndmin=2)
    if not len(truth) or not len(odom) or not len(flight):
        return
    _, alignment = trajectory_error(truth, odom)
    if alignment is None:
        return
    rotation, translation = alignment
    estimated = rotation.apply(odom[:, 1:4]) + translation
    lower = np.minimum(truth[:, 1:3].min(axis=0), estimated[:, :2].min(axis=0))-.3
    upper = np.maximum(truth[:, 1:3].max(axis=0), estimated[:, :2].max(axis=0))+.3
    scale = min(340/(upper[0]-lower[0]), 380/(upper[1]-lower[1]))
    def xy(points):
        values = (points-lower)*scale
        return np.column_stack([40+values[:, 0], 440-values[:, 1]])
    tmax = max(flight[:, 0].max(), 1)
    finite = flight[np.isfinite(flight[:, 1])]
    zmin = min(finite[:, 1].min(), .4)-.05
    zmax = max(finite[:, 1].max(), 1.2)+.05
    def tz(points):
        return np.column_stack([450+points[:, 0]/tmax*390,
                                440-(points[:, 1]-zmin)/(zmax-zmin)*370])
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="500" viewBox="0 0 900 500">',
           '<rect width="900" height="500" fill="white"/>',
           '<g font-family="sans-serif" font-size="14" fill="#222">',
           '<text x="30" y="28">Truth (blue) vs ICP odometry (red), first-pose alignment</text>',
           '<text x="460" y="52">Altimeter (blue) and commanded target (black)</text>',
           f'<text x="30" y="480">XY extent: {upper[0]-lower[0]:.2f} × {upper[1]-lower[1]:.2f} m; north/+Y up</text>',
           f'<text x="450" y="480">Time: 0–{tmax:.0f} s; height: {zmin:.2f}–{zmax:.2f} m</text>',
           '</g>', polyline(xy(truth[:, 1:3]), '#1565c0'),
           polyline(xy(estimated[:, :2]), '#d32f2f'),
           polyline(tz(finite[:, [0, 2]]), '#222'), polyline(tz(finite[:, :2]), '#1565c0'), '</svg>']
    (output/'trajectory.svg').write_text('\n'.join(svg))


def plot_map(run, output):
    """Show metric top/side projections of the evaluated map, colored by height."""
    if not (run/'map_world.npz').exists():
        return
    points = np.load(run/'map_world.npz')['points']
    if not len(points):
        return
    selected = points[np.linspace(0, len(points)-1, min(len(points), 8000), dtype=int)]
    low, high = points.min(axis=0), points.max(axis=0)
    span = np.maximum(high-low, .1)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="500" viewBox="0 0 900 500">',
           '<rect width="900" height="500" fill="#fafafa"/>',
           '<g font-family="sans-serif" font-size="14" fill="#222">',
           '<text x="30" y="28">Evaluated map: XY top view</text>',
           '<text x="470" y="28">YZ side view (Z up)</text>',
           f'<text x="30" y="475">{len(points):,} evaluated points; up to 8,000 shown; blue→yellow = low→high Z</text>',
           f'<text x="30" y="495">World bounds: X [{low[0]:.1f}, {high[0]:.1f}], Y [{low[1]:.1f}, {high[1]:.1f}], Z [{low[2]:.1f}, {high[2]:.1f}] m</text>',
           '</g>']
    for point in selected:
        rgb = colorsys.hsv_to_rgb(.65-.5*(point[2]-low[2])/span[2], .8, .8)
        color = '#'+''.join(f'{int(v*255):02x}' for v in rgb)
        for left, axes in ((30, (0, 1)), (470, (1, 2))):
            scale = min(390/span[axes[0]], 390/span[axes[1]])
            x = left+(point[axes[0]]-low[axes[0]])*scale
            y = 440-(point[axes[1]]-low[axes[1]])*scale
            svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r=".8" fill="{color}" opacity=".65"/>')
    svg.append('</svg>')
    (output/'map.svg').write_text('\n'.join(svg))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, default=Path('validation/cerro_rico'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for run in args.runs:
        result = json.loads((run/'result.json').read_text())
        name = run.name
        dest = args.output/name
        dest.mkdir(exist_ok=True)
        for filename in ('result.json', 'report.md', 'camera.png', 'truth.csv',
                         'odometry.csv', 'flight.csv', 'images.csv', 'lidar.csv',
                         'commands.csv', 'odometry_status.csv',
                         'imu_raw.csv', 'imu_filtered.csv',
                         'map_transforms.csv', 'map_world.npz', 'map_raw.npz'):
            if (run/filename).exists():
                shutil.copy2(run/filename, dest/filename)
        plot_trial(run, dest)
        plot_map(run, dest)
        results.append((name, result))
    lines = ['# Cerro Rico SIL evidence', '',
             'These are measured results for the entrance-corridor trial. Failed checks remain failures.', '',
             '| Trial | Scenario | Overall | Altitude RMSE (m) | Odometry RMSE (m) | Drift at last valid pose (m) | Surface P95 (m) | Map points |',
             '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for name, result in results:
        m = result['metrics']
        values = [m['altitude_error_m']['rmse'], m['trajectory'].get('position_rmse_m'),
                  m['trajectory'].get('end_drift_m'), m['map_surface'].get('p95_m')]
        text = ['n/a' if v is None else f'{v:.4f}' for v in values]
        verdict = 'PASS' if result['passed'] else 'FAIL'
        if result['metadata'].get('diagnostic_only', False):
            verdict = 'DIAGNOSTIC '+verdict
        lines.append(f"| [{name}]({name}/report.md) | {result['metadata']['scenario']} | "
                     f"{verdict} | {' | '.join(text)} | {m['mapped_points']} |")
    for name, result in results:
        odometry = np.loadtxt(args.output/name/'odometry.csv', delimiter=',', skiprows=1, ndmin=2)
        last_valid = float(odometry[-1, 0]) if len(odometry) else None
        lines += ['', f'## {name}', '', f'![Trajectory and altitude]({name}/trajectory.svg)', '',
                  f'![Evaluated map projections]({name}/map.svg)', '',
                  f'![Stereo camera sample]({name}/camera.png)', '',
                  'Failed checks: '+(', '.join(k for k,v in result['checks'].items() if not v['pass']) or 'none')+'.',
                  f'Last retained valid odometry stamp: **{last_valid} s** of a {result["metrics"]["sim_duration_s"]} s trial.',
                  f"Raw run directory: `experiment_data/sil/{name}/` (ignored by Git)."]
        mode = result['metadata'].get('control_mode', 'odometry')
        lines += [f'Control feedback: **{mode}**.']
        if mode == 'ground_truth':
            lines += ['Diagnostic only: ground truth drives control; this run cannot validate autonomous navigation.']
        tracking = result['metrics'].get('odometry_tracking')
        if tracking is not None:
            lines += [f"Valid-pose rate over the evaluation window: **{tracking['valid_rate_hz']:.3f} Hz**; "
                      f"longest gap: **{tracking['max_gap_s']:.3f} s**."]
    lines += ['', '## Tracking availability correction', '',
              'Version 1 results count all published odometry messages, including invalid/null estimates, in the rate check. Version 2 checks valid poses over the full evaluation window and also checks the longest tracking gap.',
              'Position RMSE and the JSON field `end_drift_m` cover retained valid estimates only; the latter is drift at the last valid pose, not necessarily the end of the 130-second trial.',
              'The fixture stops horizontal commands once the last valid estimate is 0.5 s old. A tracking outage can therefore also cause route/return failures.',
              'Per-check JSON results are retained as measured under their recorded criteria version.']
    lines += ['', '## Scope', '',
              'Ground truth is evaluation-only in odometry/hover modes. Explicit ground_truth diagnostics feed it to the route fixture and are labeled separately.',
              'Map residuals use a sampled, simplified simulation collision surface; they do not establish surveyed accuracy or full-mine coverage.',
              'These results do not validate ArduPilot firmware, hardware, dust, communications faults, gas detection, battery endurance, or every branch of the mine.']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
