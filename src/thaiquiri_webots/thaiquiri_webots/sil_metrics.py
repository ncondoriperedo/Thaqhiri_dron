"""Numerical SIL metrics independent of ROS and Webots execution."""

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def horizontal_speed_summary(truth, start=20.):
    """Measure actual XY speed from successive world poses, ignoring invalid dt."""
    truth = np.asarray(truth, dtype=float)
    if len(truth) < 2:
        return summarize_samples([])
    dt = np.diff(truth[:, 0])
    delta = np.diff(truth[:, 1:3], axis=0)
    valid = ((dt > 0) & np.isfinite(dt)
             & np.isfinite(delta).all(axis=1) & (truth[:-1, 0] >= start))
    return summarize_samples(np.linalg.norm(delta[valid] / dt[valid, None], axis=1))


def odometry_tracking(valid_stamps, start, end):
    """Score valid poses over the entire evaluation window, including dropouts."""
    if not np.isfinite([start, end]).all() or end <= start:
        raise ValueError('Tracking window must have finite, increasing bounds')
    stamps = np.asarray(valid_stamps, dtype=float)
    stamps = np.unique(stamps[np.isfinite(stamps) & (stamps >= start) & (stamps <= end)])
    gaps = np.diff(np.concatenate(([start], stamps, [end])))
    return {
        'valid_samples': len(stamps),
        'valid_rate_hz': float(len(stamps) / (end - start)),
        'max_gap_s': float(gaps.max()),
        'last_valid_stamp_s': float(stamps[-1]) if len(stamps) else None,
        'final_age_s': float(end - stamps[-1]) if len(stamps) else None,
    }


def trajectory_error(truth, estimate):
    """Align origins once using first matched poses; never fit scale or trajectory.

    Rows contain time, XYZ position, and XYZW orientation. Samples outside the
    truth time interval or over 0.15 s from truth are excluded and counted.
    """
    truth, estimate = np.asarray(truth), np.asarray(estimate)
    if len(truth) < 2 or len(estimate) < 2:
        return {'available': False}, None
    indices = np.searchsorted(truth[:, 0], estimate[:, 0])
    valid = (indices > 0) & (indices < len(truth))
    indices = np.clip(indices, 1, len(truth)-1)
    gaps = np.minimum(abs(truth[indices, 0]-estimate[:, 0]),
                      abs(truth[indices-1, 0]-estimate[:, 0]))
    valid &= gaps <= 0.15
    est = estimate[valid]
    if len(est) < 2:
        return {'available': False}, None
    positions = np.column_stack([np.interp(est[:, 0], truth[:, 0], truth[:, i])
                                 for i in (1, 2, 3)])
    first = np.argmin(abs(truth[:, 0]-est[0, 0]))
    rotation = Rotation.from_quat(truth[first, 4:8]) * Rotation.from_quat(est[0, 4:8]).inv()
    translation = positions[0] - rotation.apply(est[0, 1:4])
    aligned = rotation.apply(est[:, 1:4]) + translation
    errors = np.linalg.norm(aligned - positions, axis=1)
    distance = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
    return {'available': True, 'matched_samples': len(est),
            'excluded_samples': int(len(estimate)-len(est)),
            'position_rmse_m': float(np.sqrt(np.mean(errors**2))),
            'position_p95_m': float(np.percentile(errors, 95)),
            'end_drift_m': float(errors[-1]),
            'end_drift_stamp_s': float(est[-1, 0]),
            'end_drift_scope': 'last matched valid estimate',
            'truth_path_length_m': distance,
            'end_drift_per_m': float(errors[-1]/distance) if distance > 0.1 else None,
            'alignment': 'first matched pose SE(3), no fitted scale/trajectory'}, (rotation, translation)


def summarize_samples(values):
    """Summarize finite scalar observations without silently accepting missing data."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {'count': 0, 'mean': None, 'rmse': None, 'p95_abs': None, 'max_abs': None}
    return {'count': len(values), 'mean': float(np.mean(values)),
            'rmse': float(np.sqrt(np.mean(values**2))),
            'p95_abs': float(np.percentile(abs(values), 95)),
            'max_abs': float(np.max(abs(values)))}


def surface_error(points, reference):
    """Score map points against sampled geometry; empty maps are unavailable."""
    points, reference = np.asarray(points), np.asarray(reference)
    if not points.size or not reference.size:
        return {'available': False}
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return {'available': False}
    distances = cKDTree(reference).query(points, workers=-1)[0]
    return {'available': True, 'median_m': float(np.median(distances)),
            'p95_m': float(np.percentile(distances, 95)),
            'method': 'nearest sampled collision surface; first-pose alignment; includes launch-pad points'}
