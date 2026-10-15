"""Regression checks for attitude conventions, motor mixing, and IMU startup."""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    from _ros_stubs import install
    install()

from builtin_interfaces.msg import Time

from geometry_msgs.msg import Quaternion, Twist

import pytest

from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Imu

from thaiquiri_webots.drone_driver import (
    DroneDriver, mix_motor_speeds, quaternion_to_roll_pitch)
from thaiquiri_webots.imu_merger_node import ImuMergerNode


@pytest.mark.parametrize('angles', [(0, 0, 0), (0.3, -0.4, 1.2), (-0.7, 0.2, -2)])
def test_attitude_is_euler_not_quaternion_components(angles):
    """Recover roll/pitch independently of yaw and quaternion scale/sign."""
    values = Rotation.from_euler('xyz', angles).as_quat() * -2
    q = Quaternion(**dict(zip(('x', 'y', 'z', 'w'), values)))
    assert quaternion_to_roll_pitch(q) == pytest.approx(angles[:2])


@pytest.mark.parametrize('q', [Quaternion(w=0.0), Quaternion(x=math.nan)])
def test_invalid_attitude(q):
    """Reject missing or non-finite orientation rather than driving motors."""
    assert quaternion_to_roll_pitch(q) is None


def test_mixer_corrects_roll_pitch_and_never_reverses_thrust():
    """Positive roll reduces left thrust; positive pitch increases front thrust."""
    limits = [930.0] * 4
    roll = list(map(abs, mix_motor_speeds(470, 10, 0, 0, limits)))
    assert roll[0] < roll[1] and roll[2] < roll[3]
    pitch = list(map(abs, mix_motor_speeds(470, 0, 10, 0, limits)))
    assert pitch[0] > pitch[2] and pitch[1] > pitch[3]
    saturated = mix_motor_speeds(470, 2000, -2000, 2000, limits)
    assert all(0 <= sign * speed <= 930
               for speed, sign in zip(saturated, (-1, 1, 1, -1)))


def test_startup_requires_received_imu_and_altitude_command_is_increment():
    """Jazzy's default Imu quaternion is valid but is not a received measurement."""
    robot = Mock()
    robot.getBasicTimeStep.return_value = 8
    motors = [Mock() for _ in range(4)]
    for motor in motors:
        motor.getMaxVelocity.return_value = 930.0
    altimeter = Mock()
    altimeter.getValue.return_value = 0.5
    robot.getDevice.side_effect = [altimeter, *motors]
    with patch('thaiquiri_webots.drone_driver.rclpy') as ros:
        clock = ros.create_node.return_value.get_clock.return_value
        clock.now.return_value.to_msg.return_value = Time()
        driver = DroneDriver()
        driver.init(SimpleNamespace(robot=robot), {})
        driver.step()
        for motor in motors:
            motor.setVelocity.assert_called_with(0.0)
        callbacks = {c.args[1]: c.args[2]
                     for c in ros.create_node.return_value.create_subscription.call_args_list}
        imu = Imu()
        imu.header.frame_id = 'imu_link'
        callbacks['/imu/data'](imu)
        command = Twist()
        command.linear.z = 0.05
        callbacks['cmd_pos'](command)
        driver.step()
        driver.step()
        assert driver.target_altitude == pytest.approx(0.55)
        assert all(abs(m.setVelocity.call_args.args[0]) > 0 for m in motors)


def test_imu_rotation_covariance_and_source_header():
    """Rotate Cube's native axes/covariances into FLU without mutating input."""
    merger = SimpleNamespace(_initialized=True, _last_stamp_ns=None, imu_pub=Mock())
    accel, gyro = Imu(), Imu()
    accel.header.frame_id = 'accelerometer_cube'
    accel.header.stamp.sec = 7
    gyro.header.stamp.sec = 7
    accel.linear_acceleration.y = 9.81
    gyro.angular_velocity.z = 0.2
    accel.linear_acceleration_covariance = [1., 0., 0., 0., 2., 0., 0., 0., 3.]
    ImuMergerNode.sync_callback(merger, accel, gyro)
    result = merger.imu_pub.publish.call_args.args[0]
    assert result.header.frame_id == 'imu_link'
    assert result.header.stamp.sec == 7
    assert accel.header.frame_id == 'accelerometer_cube'
    assert result.linear_acceleration.z == pytest.approx(9.81)
    assert result.angular_velocity.x == pytest.approx(0.2)
    assert list(result.linear_acceleration_covariance)[::4] == [3., 1., 2.]
    assert result.orientation_covariance[0] == -1


def test_imu_waits_for_settled_initialization():
    """Do not initialize the filter from free fall or a contact impulse."""
    merger = SimpleNamespace(_initialized=False, _settled_since=None,
                             _last_stamp_ns=None, imu_pub=Mock())
    accel, gyro = Imu(), Imu()
    ImuMergerNode.sync_callback(merger, accel, gyro)
    assert not merger.imu_pub.publish.called
    accel.linear_acceleration.y = 9.81
    for stamp in (1.0, 1.1, 1.3):
        accel.header.stamp.sec = int(stamp)
        accel.header.stamp.nanosec = int((stamp % 1) * 1e9)
        gyro.header.stamp.sec = accel.header.stamp.sec
        gyro.header.stamp.nanosec = accel.header.stamp.nanosec
        ImuMergerNode.sync_callback(merger, accel, gyro)
    assert merger._initialized
    merger.imu_pub.publish.assert_called_once()


def test_imu_does_not_mis_stamp_old_gyro_or_duplicate_samples():
    merger = SimpleNamespace(_initialized=True, _last_stamp_ns=None, imu_pub=Mock())
    accel, gyro = Imu(), Imu()
    accel.header.stamp.sec = 4
    gyro.header.stamp.sec = 4
    gyro.header.stamp.nanosec = 8000000
    ImuMergerNode.sync_callback(merger, accel, gyro)
    assert not merger.imu_pub.publish.called
    gyro.header.stamp.nanosec = 0
    ImuMergerNode.sync_callback(merger, accel, gyro)
    ImuMergerNode.sync_callback(merger, accel, gyro)
    merger.imu_pub.publish.assert_called_once()


def test_imu_drops_nonfinite_measurements_without_poisoning_filter():
    merger = SimpleNamespace(_initialized=True, _last_stamp_ns=None, imu_pub=Mock())
    accel, gyro = Imu(), Imu()
    accel.linear_acceleration.x = math.nan
    ImuMergerNode.sync_callback(merger, accel, gyro)
    assert not merger.imu_pub.publish.called


def test_sensor_frames_match_model_geometry():
    """Keep IMU rotation and rectified stereo assumptions aligned with assets."""
    package = Path(__file__).resolve().parents[1]
    urdf = ET.parse(package / 'resource/thaiquiri.urdf').getroot()
    origins = {j.get('name'): j.find('origin') for j in urdf.findall('joint')}
    body = Rotation.from_euler('xyz', list(map(
        float, origins['base_footprint_to_base_link'].get('rpy').split())))
    imu = Rotation.from_euler('xyz', list(map(
        float, origins['cube_orange_imu_joint'].get('rpy').split())))
    assert (body * imu).magnitude() < 1e-5

    proto = (package / 'webots_world/protos/thaiquiri.proto').read_text()
    positions = []
    for side in ('left', 'right'):
        values = re.search(r'DEF camera_' + side + r' Camera \{\s+translation ([^\n]+)', proto)
        actual = list(map(float, values.group(1).split()))
        expected = list(map(float, origins[
            f'ZED_X_Mini_camera_camera_{side}_joint'].get('xyz').split()))
        assert actual == pytest.approx(expected)
        positions.append(actual)
    assert [right-left for left, right in zip(*positions)] == pytest.approx([0.05, 0, 0])
