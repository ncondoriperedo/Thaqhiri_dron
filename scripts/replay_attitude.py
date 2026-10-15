"""Check attitude tuning using reconstructed IMU signals from saved SIL poses."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from thaiquiri_webots.imu_validation import reconstruction_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = {}
    for run in args.runs:
        path = run/'truth.csv'
        truth = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
        results[run.name] = {
            'truth_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'tuning': reconstruction_errors(truth),
        }
    report = {
        'method': 'IMU reconstructed from sampled truth, not recorded raw IMU or closed-loop flight. Runtime does not use this truth.',
        'scoring_window_s': [30, 100], 'results': results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
