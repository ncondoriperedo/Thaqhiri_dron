# Thaqhiri Dron

This repository is a **ROS 2 Jazzy simulation workspace for the Thaiquiri quadrotor**, designed for testing autonomous aerial robotics in mine-like environments using **Webots**. The main objective is to develop and validate flight control, inertial sensing, stereo vision, LiDAR-based localization, and 3D mapping before moving toward integration with a real or simulated ArduPilot flight stack.

The repository directory is named `thaqhiri`, while the ROS package and robot assets use the name `thaiquiri`. The main ROS 2 package is therefore `thaiquiri_webots`.

## Main Features

The simulation provides a complete sensing and control pipeline for the quadrotor, including:

- Quadrotor dynamics and motor control in Webots.
- Keyboard-based manual flight commands.
- Altitude and attitude stabilization.
- Cube accelerometer and gyroscope simulation.
- IMU synchronization, axis conversion, and Madgwick orientation filtering.
- Stereo cameras with image conversion, calibration correction, and stereo processing.
- A 40-layer simulated LiDAR.
- RTAB-Map integration for LiDAR/stereo 3D mapping.
- ICP-based odometry.
- ROS 2 bag recording for experimental data.
- Automated Software-in-the-Loop (SIL) validation tools.
- Experimental components for future ArduPilot integration.

Two simulation environments are available: a controlled mine-like test arena and an imported model of the **Cerro Rico mine**.

## Control Architecture

The default configuration does not use ArduPilot. Instead, a Python Webots controller directly controls the four propeller motors.

The main control flow is:

```text
Keyboard commands
       |
       v
    /cmd_pos
       |
       v
 DroneDriver
       |
       v
Motor mixing -> Webots motors -> Webots physics
       ^
       |
Altitude + filtered IMU attitude
```

The controller automatically waits for a valid filtered IMU orientation before starting the motors. It stabilizes:

- altitude,
- roll,
- pitch,
- yaw rate.

Horizontal position is not actively controlled, so some lateral drift is expected.

The nominal hover motor speed is approximately **469 rad/s**, with PID-based altitude and attitude control and additional motor trims to compensate for the simulated payload imbalance.

## IMU Processing

The simulated Cube accelerometer and gyroscope are synchronized before being merged into a ROS IMU message.

The IMU pipeline:

```text
Cube accelerometer + gyroscope
        |
        v
IMU merger
        |
        v
Axis conversion to ROS FLU frame
        |
        v
Madgwick filter
        |
        v
/imu/data
        |
        v
Flight controller
```

The Cube native coordinate system is converted to the ROS `forward-left-up` convention. The merger also waits for the initial spawn and contact motion to settle before orientation initialization.

The current Madgwick gain is **0.01**. Previous tests showed that the earlier gain of `0.1` significantly attenuated real vehicle tilt during flight reconstruction.

## Stereo Vision and LiDAR

The robot contains two 640 × 480 cameras with a **5 cm stereo baseline**. Webots images are converted from BGRA to BGR, and the camera calibration information is corrected before being sent to `stereo_image_proc`.

The stereo processing produces rectified images and disparity data.

The simulated LiDAR provides:

- 360° horizontal coverage,
- approximately 59° vertical field of view,
- 40 vertical layers,
- 1000 horizontal samples,
- 0.1–40 m range.

Although it is labelled as a Livox sensor, it behaves as a conventional rotating Webots LiDAR and does not reproduce Livox's real non-repetitive scanning pattern.

## RTAB-Map Mapping

RTAB-Map combines the stereo cameras and LiDAR to build a 3D map of the environment.

The current configuration uses:

- **ICP odometry**
- LiDAR point clouds
- rectified stereo images
- filtered IMU data
- `map -> odom -> base_footprint` TF structure

Visual odometry is disabled.

ICP uses a local point-cloud map of up to 15,000 points, 0.10 m sampling, and a 0.50 m correspondence search distance. These parameters were introduced to improve tracking with the dense simulated LiDAR.

Mapping is primarily intended as an integration experiment. A nonempty RTAB-Map map should not yet be interpreted as proof of accurate localization.

## Cerro Rico Simulation

The repository also includes an imported model of the **Cerro Rico mine**, stored as:

```text
cerro_rico.wbt
```

It can be launched using:

```bash
ros2 launch thaiquiri_webots drone_launch.py world:=cerro_rico.wbt
```

An automated SIL experiment is provided to test stabilization, mapping, trajectory behavior, and environmental interaction inside the mine.

Current SIL results demonstrate functional stabilization and mapping, but several issues remain, including:

- lateral drift,
- return-path tracking error,
- poor camera contrast in some mine regions,
- incomplete traversal of the intended trajectory.

Therefore, the complete mine-navigation acceptance criteria have not yet been achieved.

## Gas Sensor Payload

The quadrotor model visually includes a gas sensing payload containing components labelled for gases such as:

- CO,
- H₂S,
- NO₂,
- O₂.

However, the gas sensors currently exist **only as 3D geometry**.

The repository does not yet simulate:

- gas dispersion,
- gas concentration fields,
- sensor response,
- environmental interference,
- gas ROS topics,
- gas measurement processing.

Implementing the gas sensing system is therefore an important future extension of the simulation.

## Main ROS Package

The `thaiquiri_webots` package contains the simulation and processing components.

Important modules include:

- `drone_driver.py` — active quadrotor controller.
- `teleop_drone.py` — keyboard command publisher.
- `imu_merger_node.py` — Cube IMU synchronization and coordinate transformation.
- `bgra_to_bgr.py` — camera image conversion.
- `fix_camera_info.py` — stereo calibration correction.
- `sil_observer.py` — ground-truth observer used only for SIL evaluation.
- `sil_metrics.py` — trajectory and map error calculations.
- `odom_to_ardupilot.py` — experimental external-navigation bridge.
- `gps_pose_publisher.py` — incomplete experimental ArduPilot GPS helper.
- `drone_driver_3.py` — inactive experimental ArduPilot/Webots controller.

## ArduPilot Integration

The repository contains experimental components for future ArduPilot integration, but ArduPilot is **not part of the default simulation**.

The current repository does not provide:

- a complete ArduPilot source workspace,
- an active SITL container,
- a Micro-XRCE-DDS Agent configuration,
- Thaiquiri ArduPilot parameters,
- a fully validated Webots–SITL connection.

The `ardupilot_msgs` package only defines ROS messages and services.

Additional development is required to correctly connect motor commands, sensor data, coordinate systems, networking, ROS DDS communication, and external navigation to ArduPilot.
