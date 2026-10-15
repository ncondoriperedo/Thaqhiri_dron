"""Reference checks for dynamic attitude error, gyro integration and correction."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from thaiquiri_webots.imu_validation import imu_step, reconstruct_sensors


def test_gyro_integration_without_accelerometer():
    q = [0., 0., 0., 1.]
    for _ in range(1000):
        q = imu_step(q, [0, 0, .2], [0, 0, 0], .008, .01)
    assert (Rotation.from_quat(q).inv()*Rotation.from_rotvec([0, 0, 1.6])).magnitude() < 1e-6


def test_moving_thrust_must_not_erase_real_tilt():
    times = np.arange(0, 12, .008)
    roll = np.radians(6)*np.sin(2*np.pi*.45*times)
    omega = np.diff(roll)/.008
    rmse = []
    for gain in (.1, .01):
        quaternions = [[0., 0., 0., 1.]]
        for w in omega:
            # Specific force along body thrust during accelerating flight.
            quaternions.append(imu_step(quaternions[-1], [w, 0, 0], [0, 0, 9.81], .008, gain))
        sensed = Rotation.from_quat(quaternions).as_euler('xyz')[:, 0]
        rmse.append(np.sqrt(np.mean(np.degrees(sensed-roll)**2)))
    assert rmse[0] > 2
    assert rmse[1] < .6


def test_lower_gain_still_corrects_stationary_tilt_and_gyro_bias():
    q = Rotation.from_euler('x', np.radians(5)).as_quat()
    for _ in range(1500):
        q = imu_step(q, [.005, 0, 0], [0, 0, 9.81], .008, .01)
    assert abs(Rotation.from_quat(q).as_euler('xyz')[0]) < np.radians(.1)


def test_reconstructed_static_sensor_has_gravity_and_no_rotation():
    times = np.linspace(0, 10, 101)
    r = Rotation.from_euler('xyz', [.2, -.1, 1.])
    truth = np.column_stack([times, np.zeros((101, 3)), np.tile(r.as_quat(), (101, 1))])
    _, _, acceleration, omega = reconstruct_sensors(truth, start=2, end=9)
    assert np.max(abs(omega)) < 1e-12
    assert acceleration == pytest.approx(np.tile(r.inv().apply([0, 0, 9.81]), (len(acceleration), 1)))


@pytest.mark.parametrize('gain,dt', [(-1, .008), (np.nan, .008), (.01, 0)])
def test_invalid_integration_settings(gain, dt):
    with pytest.raises(ValueError):
        imu_step([0, 0, 0, 1], [0, 0, 0], [0, 0, 9.81], dt, gain)
