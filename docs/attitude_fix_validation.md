# Moving-flight attitude correction

## Problem and implemented change

The saved ground-truth-route trial still circled around its goal and reached
0.669 m/s despite a requested 0.2 m/s per axis. In that trial, real roll RMS was
4.23 degrees, while the filtered roll consumed by the inner controller had RMS
0.60 degrees. True pitch RMS was 3.73 degrees versus 0.55 degrees estimated.
Changing navigation feedback to ground truth did not change the inner attitude
source, which remained the Madgwick-filtered Cube IMU.

The normal simulation launch and SIL runner now both use Madgwick gain **0.01**,
instead of the previous 0.1. This reduces accelerometer correction during
translation while retaining gyro propagation and a nonzero gravity correction.
`imu_filter_gain:=0.1` (launch) or `--imu-filter-gain 0.1` (runner) restores the
previous tuning. No special flags are needed to use the correction.

The Cube IMU merger now matches equal simulation timestamps, rejects duplicate,
out-of-order, mismatched and nonfinite samples, and keeps its existing startup
settling gate. Both Webots devices request 125 Hz on the same simulation clock.
The former approximate synchronizer allowed 50 ms of separation. That timing
allowance was an avoidable integration risk; the available older recordings do
not prove that it caused the measured attitude error.

Raw and filtered inertial samples are now saved separately by the SIL runner.
No ground-truth attitude or position is injected into the production controller
or odometry. Automatic odometry resets remain disabled and the ICP correction
gate remains 0.2 m; tracking continuity is not obtained by merely loosening that gate.

## Offline evidence

`scripts/replay_attitude.py` reconstructs angular rate and specific force from
the two saved world-pose trajectories. It includes the Cube sensor lever arm,
resamples at 8 ms, and evaluates the two gains against identical reconstructed
sensor input. Attitude error below is roll/pitch RMSE over 30–100 s:

| Recorded motion | Gain 0.1 | Gain 0.01 |
| --- | ---: | ---: |
| Odometry-driven route | 4.590 degrees | 0.470 degrees |
| Ground-truth-driven route | 3.353 degrees | 0.348 degrees |

The high-gain reconstruction also reproduces the strong attenuation of tilt
seen in the recorded filter output. This supports the correction mechanism.
It does **not** constitute a closed-loop re-flight or exact replay of raw IMU:
the original recordings lack raw inertial measurements, and interpolation of
20.8 Hz truth cannot recreate every 125 Hz sensor effect. Gyro bias and noise
are not identified by these reconstructions. This is a simulation-specific
tuning change, not a calibration for a real aircraft.

The independent quaternion reference used for validation is in
`thaiquiri_webots/imu_validation.py`; runtime still uses the installed
`imu_filter_madgwick` implementation. Tests cover dynamic translational
specific force, retained static tilt/bias correction, gyro-only propagation,
sensor geometry, invalid inputs, and the actual merger callback guards.
When ROS is absent, driver unit tests use message-shaped doubles, not a
replacement ROS executor, transport, or synchronizer.

Reproduce the offline comparison from a sourced package environment:

```bash
python scripts/replay_attitude.py /path/to/nominal /path/to/gt --output experiment_data/attitude_reconstruction.json
python -m pytest -q src/thaiquiri_webots/test/test_control.py src/thaiquiri_webots/test/test_imu_validation.py src/thaiquiri_webots/test/test_sil_route.py src/thaiquiri_webots/test/test_sil_metrics.py src/thaiquiri_webots/test/test_cerro_assets.py
```

## Acceptance and remaining validation

Criteria version 3 now checks actual horizontal peak speed and final speed
RMSE, and tightens attitude-estimation RMSE to 1 degree. A small final position
error can no longer hide a rapid orbit around the waypoint. The previous
ground-truth trial would fail these new control criteria even though 20 of its
21 original checks passed. Archived results have not been rewritten.

The complete ROS Jazzy/Webots/RTAB-Map pipeline is unavailable on the Windows
validation host. A full simulator rerun is still required to establish actual
closed-loop speed, return settling, ICP tracking continuity and camera quality
after this correction. The reduced gain alone is not a localization recovery
or independent stereo fallback. Camera contrast remains a separate known issue.

Reference implementations inspected:

- [imu_tools IMU-only update](https://github.com/CCNYRoboticsLab/imu_tools/blob/jazzy/imu_filter_madgwick/src/imu_filter.cpp)
- [Webots accelerometer: gravity and motion acceleration](https://github.com/cyberbotics/webots/blob/R2025a/src/webots/nodes/WbAccelerometer.cpp)
- [Webots ROS IMU publishing and timestamps](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_driver/src/plugins/dynamic/Ros2IMU.cpp)
