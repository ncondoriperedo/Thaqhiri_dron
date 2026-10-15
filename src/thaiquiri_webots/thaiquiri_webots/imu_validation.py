"""Offline attitude checks; never feeds reconstructed truth into the flight stack.

The small independent quaternion implementation below checks Madgwick's
accelerometer-correction behavior. Runtime estimation remains imu_tools.
"""

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp


def imu_step(quaternion, angular_velocity, acceleration, dt, gain):
    """Integrate gyro and a normalized gravity residual, using XYZW quaternions."""
    q = np.asarray(quaternion, dtype=float)
    omega = np.asarray(angular_velocity, dtype=float)
    acceleration = np.asarray(acceleration, dtype=float)
    if (q.shape != (4,) or omega.shape != (3,) or acceleration.shape != (3,)
            or not np.isfinite(np.concatenate((q, omega, acceleration))).all()
            or not np.isfinite([dt, gain]).all() or dt <= 0 or not 0 <= gain <= 1
            or np.linalg.norm(q) < 1e-12):
        raise ValueError('Finite IMU values, a quaternion, positive dt and gain in [0,1] required')
    q = q/np.linalg.norm(q)
    x, y, z, w = q
    gx, gy, gz = omega
    derivative = .5*np.array([w*gx+y*gz-z*gy, w*gy+z*gx-x*gz,
                              w*gz+x*gy-y*gx, -x*gx-y*gy-z*gz])
    norm = np.linalg.norm(acceleration)
    if norm > 1e-12 and gain:
        direction = acceleration/norm
        predicted = np.array([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)])
        jacobian = np.array([[2*z, -2*w, 2*x, -2*y],
                             [2*w, 2*z, 2*y, 2*x], [-4*x, -4*y, 0, 0]])
        gradient = jacobian.T@(predicted-direction)
        gradient_norm = np.linalg.norm(gradient)
        if gradient_norm > 1e-12:
            derivative -= gain*gradient/gradient_norm
    result = q+derivative*dt
    return result/np.linalg.norm(result)


def reconstruct_sensors(truth, start=2., end=129.9, dt=.008):
    """Approximate the Cube IMU from sampled poses, including its lever arm.

    This reconstructs motion and specific force; it is not a raw-sensor replay
    or closed-loop simulator. Sampling/interpolation omit high-frequency motion.
    """
    truth = np.asarray(truth, dtype=float)
    if (len(truth) < 4 or truth.shape[1] != 8 or not np.isfinite(truth).all()
            or not np.all(np.diff(truth[:, 0]) > 0)
            or not np.isfinite([start, end, dt]).all() or dt <= 0
            or not truth[0, 0] <= start < end <= truth[-1, 0]):
        raise ValueError('Ordered finite poses and an in-range sampling window required')
    times = np.arange(start, end, dt)
    if len(times) < 2:
        raise ValueError('At least two reconstructed samples required')
    poses = Rotation.from_quat(truth[:, 4:8])
    rotations = Slerp(truth[:, 0], poses)(times)
    # Cube native coordinates (left, up, forward) rotated into body FLU.
    sensor_world = truth[:, 1:4]+poses.apply([.042, .093, .325])
    accel_world = CubicSpline(truth[:, 0], sensor_world)(times, 2)
    acceleration = rotations.inv().apply(accel_world+[0, 0, 9.81])
    omega = (rotations[:-1].inv()*rotations[1:]).as_rotvec()/dt
    return times, rotations, acceleration, omega


def reconstruction_errors(truth, gains=(.1, .01), start=30., end=100.):
    """Compare tunings on the same reconstructed input; truth is scoring-only."""
    dt = .008
    times, rotations, acceleration, omega = reconstruct_sensors(truth)
    actual = rotations.as_euler('xyz')
    mask = (times >= start) & (times < end)
    if not mask.any():
        raise ValueError('No samples in scoring interval')
    results = {}
    for gain in gains:
        quaternions = [rotations[0].as_quat()]
        for i in range(len(times)-1):
            quaternions.append(imu_step(quaternions[-1], omega[i], acceleration[i+1], dt, gain))
        angles = Rotation.from_quat(quaternions).as_euler('xyz')
        error = np.arctan2(np.sin(angles-actual), np.cos(angles-actual))
        results[str(gain)] = {
            'roll_pitch_rmse_deg': float(np.sqrt(np.mean(np.degrees(error[mask, :2])**2))),
            'roll_pitch_max_abs_deg': float(np.max(np.abs(np.degrees(error[mask, :2])))),
            'estimated_std_deg': np.std(np.degrees(angles[mask, :2]), axis=0).tolist(),
            'true_std_deg': np.std(np.degrees(actual[mask, :2]), axis=0).tolist(),
        }
    return results
