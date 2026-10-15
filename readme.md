# Thaqhiri / Thaiquiri Webots experiments

This repository is a ROS 2 Jazzy workspace for simulating the **Thaiquiri quadrotor in a mine-like environment**, controlling it from the keyboard, and experimenting with stereo vision, LiDAR, inertial sensing, and RTAB-Map mapping.

The directory uses the spelling `thaqhiri`, but the ROS package, Python modules, and robot assets use **`thaiquiri`**. Use `thaiquiri_webots` in ROS commands. The Docker service retains the older name **`padoru_webots_jazzy`**.

**Cerro Rico mine:** the supplied `02_Cerro_Rico_Messy.blend` is integrated as
`cerro_rico.wbt`. See [mine import and SIL validation](docs/cerro_rico_sil.md) for
asset provenance, the automated flight/mapping trial, acceptance criteria, and
interpretation limits. Launch it with
`ros2 launch thaiquiri_webots drone_launch.py world:=cerro_rico.wbt`.
The [measured SIL report](validation/cerro_rico/report.md) records working
stabilization/mapping and remaining failures in drift, return tracking, and
camera contrast; the complete traversal criteria are not yet met.

## 1. The bigger picture

There are two packages:

- **`thaiquiri_webots`** supplies the robot model, environment, launch file, flight controller, keyboard controls, and sensor-processing nodes.
- **`ardupilot_msgs`** defines ROS messages and services for an experimental ArduPilot connection. Building it generates interfaces; it does not start an autopilot or service servers.

The default experiment uses a Python controller directly inside Webots:

```text
Keyboard teleop -> /cmd_pos -> DroneDriver -> four Webots propeller motors
                                   ^                    |
                                   |                    v
                         altitude + gyro          Webots physics

Webots stereo cameras -> BGRA-to-BGR conversion + camera-info correction
                       -> stereo_image_proc -> rectified images / disparity
Webots LiDAR -----------------------------------------> RTAB-Map
Rectified stereo images ------------------------------> RTAB-Map
                                                        |
                                                 ICP odometry + map

Webots Cube accelerometer + gyro -> IMU merger -> Madgwick -> /imu/data
```

The controller consumes `/imu/data` in the ROS forward-left-up `imu_link` frame and converts its quaternion to roll/pitch. The IMU merger rotates the Cube's native axes and waits for the initial spawn/contact motion to settle before initializing Madgwick. The controller waits for an actual IMU message before starting the motors.

The moving-flight IMU tuning uses Madgwick gain **0.01** and exactly synchronized
Cube accelerometer/gyro samples. The previous gain 0.1 strongly attenuated the
real tilt in offline reconstructions of the saved flights. See
[correction and validation limits](docs/attitude_fix_validation.md). The normal
launch includes the correction; `imu_filter_gain:=0.1` restores the old tuning
for comparison. Sensor reconstruction is validation-only and never supplies
ground truth to the flight controller.

With the Linux ROS/Webots dependencies already installed, `bash run_sil.sh`
builds both packages and starts the corrected 130-second nominal trial. It
uses a new timestamped output directory by default. Stop other Webots/teleop
sessions first. Existing `WEBOTS_HOME` and `.venv` setups are reused.

The larger intended workflow is to develop sensing and mapping in simulation, then connect estimated motion to ArduPilot. The repository contains pieces of that integration, but the default launch uses neither ArduPilot SITL nor the odometry/GPS bridge nodes.

Two worlds are supplied: the original arena and the imported Cerro Rico mine.
`scripts/run_sil.py` automates a bounded entrance-corridor flight/mapping trial;
the procedures below also support manual experiments. The gas-sensor payload is
represented visually; no gas dispersion, concentration measurements, or gas ROS
topics are implemented.

## 2. Setup and build

### Native setup with uv (recommended; no Docker)

Use **Ubuntu 24.04 x86-64, ROS 2 Jazzy, Python 3.12**, an accessible graphical desktop (`DISPLAY`), and `uv`. ROS must already be installed under `/opt/ros/jazzy`. Run these commands from this repository's root.

`uv` manages the Python dependencies; ROS's compiled libraries, OpenCV/cv_bridge, and the Webots simulator are native dependencies, not PyPI packages. The virtual environment uses the **system Python** and `--system-site-packages` so it can load ROS's Python extensions. Do not substitute a uv-managed Python version.

#### 1. Install native ROS/build dependencies

```bash
sudo apt-get update
# Keep existing Jazzy libraries and newly installed packages compatible.
sudo apt-get install --only-upgrade $(dpkg-query -W -f='${binary:Package}\n' 'ros-jazzy-*')
sudo apt-get install -y \
  build-essential cmake curl bzip2 \
  python3-colcon-common-extensions python3-rosdep python3-opencv \
  ros-jazzy-webots-ros2 ros-jazzy-rtabmap-ros \
  ros-jazzy-stereo-image-proc ros-jazzy-image-proc \
  ros-jazzy-imu-filter-madgwick ros-jazzy-cv-bridge \
  ros-jazzy-message-filters ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros ros-jazzy-rosidl-default-generators \
  ros-jazzy-fastcdr ros-jazzy-fastrtps ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-rosbag2 ros-jazzy-rviz2
```

#### 2. Install Webots R2025a in your home directory

This release matches the world/PROTO headers. If it is already installed, set `WEBOTS_HOME` to that installation instead. The following extracts into `~/.local/opt/webots`; use an empty destination when installing.

```bash
mkdir -p "$HOME/.local/opt" "$HOME/.cache/webots-downloads"
curl -fL --retry 3 \
  https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a-x86-64.tar.bz2 \
  -o "$HOME/.cache/webots-downloads/webots-R2025a-x86-64.tar.bz2"
tar -xjf "$HOME/.cache/webots-downloads/webots-R2025a-x86-64.tar.bz2" \
  -C "$HOME/.local/opt"
export WEBOTS_HOME="$HOME/.local/opt/webots"
"$WEBOTS_HOME/webots" --version
```

Webots downloads externally referenced assets on first launch, so internet access is needed.

#### 3. Create the uv environment and build

```bash
cd /path/to/webots_thaqhiri
source /opt/ros/jazzy/setup.bash
uv venv --python /usr/bin/python3 --system-site-packages .venv
uv pip install --python .venv/bin/python -r requirements.txt
source .venv/bin/activate

python -c 'from colcon_core.command import main; raise SystemExit(main())' build --symlink-install \
  --packages-select ardupilot_msgs thaiquiri_webots \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
source install/setup.bash
```

Invoking `colcon_core.command.main` with the active Python interpreter generates node entry points using `.venv/bin/python`. The CMake options also select this interpreter for message generation. Build artifacts from a different interpreter/workspace should be moved aside before rebuilding.

`requirements.txt` pins the Python dependencies resolved with uv; `requirements.in` records the direct requirements. NumPy stays below 2 for compatibility with Ubuntu's OpenCV/cv_bridge binaries. To intentionally refresh the pins, run `uv pip compile requirements.in -o requirements.txt`, then reinstall and rebuild. OpenCV is supplied by apt; do not overlay it with `opencv-python`.

#### 4. Launch (and prepare every additional terminal)

```bash
cd /path/to/webots_thaqhiri
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export WEBOTS_HOME="$HOME/.local/opt/webots"

# Simulation and sensor pipeline:
ros2 launch thaiquiri_webots drone_launch.py rtabmap:=false
```

For the default mapping pipeline, stop that launch with `Ctrl-C`, then run:

```bash
ros2 launch thaiquiri_webots drone_launch.py
```

In another terminal, repeat the environment setup above, then run `ros2 run thaiquiri_webots teleop_drone` or the topic checks in section 3. Use Webots' Play control if paused. Stop with `Ctrl-C` in the launch terminal. No Docker service or `xhost` change is needed for this native setup.

### Docker setup (alternative)

#### Host prerequisites

The supplied Docker configuration expects:

- Linux on x86-64/amd64, Docker Engine, and Docker Compose with support for `gpus: all` (Compose 2.30 or newer).
- An X11 display, or XWayland with a usable `DISPLAY` and `/tmp/.X11-unix` socket.
- NVIDIA drivers and NVIDIA Container Toolkit, because Compose explicitly requests NVIDIA GPU resources.
- `/dev/dri` for graphics-device access.
- Internet access for Docker/apt downloads and Webots' externally referenced Cyberbotics assets.

The image uses `osrf/ros:jazzy-desktop-full` and installs Webots from Cyberbotics' apt repository. **Webots is not version-pinned in the Dockerfile**; the world and PROTO files declare **R2025a**. Prefer a compatible Webots release and record the installed version for repeatable experiments.

For a non-NVIDIA machine, adapt `docker-compose.yml`: remove both `gpus: all` and the NVIDIA reservation under `deploy.resources.reservations.devices`, remove NVIDIA-specific environment settings, and configure rendering for that machine. `/dev/dri` must also exist if it is mounted.

#### Start the container — host terminal

Run from this repository's root: Compose mounts `$PWD` as the ROS workspace.

```bash
cd /path/to/webots_thaqhiri
echo "$DISPLAY"
xhost +local:docker
docker compose config --quiet
docker compose up -d --build padoru_webots_jazzy
docker compose exec padoru_webots_jazzy bash
```

The X11 command grants local access for the session; revoke it with `xhost -local:docker` when finished. If the display still rejects the container, use the Xauthority configuration appropriate to your display session.

The container runs as `ubuntu`, with working directory `/home/ubuntu/ros_ws`. The entrypoint sources ROS, but **does not build the workspace or launch the simulation**.

#### Install runtime dependencies — inside the container

Install the ROS/Python runtime dependencies and build tools (also declared in the package manifest where applicable):

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros_ws
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-opencv \
  ros-jazzy-webots-ros2 ros-jazzy-rtabmap-ros \
  ros-jazzy-stereo-image-proc ros-jazzy-image-proc \
  ros-jazzy-imu-filter-madgwick ros-jazzy-cv-bridge \
  ros-jazzy-message-filters ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosbag2 ros-jazzy-rviz2

# Initialize rosdep only if this image has not already initialized it.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

Packages installed interactively survive container restarts, but not container recreation. For a persistent development image, add the missing dependencies to the Dockerfile's apt installation list.

#### Build both packages — inside the container

```bash
colcon build --symlink-install --packages-select ardupilot_msgs thaiquiri_webots
source install/setup.bash
ros2 pkg prefix thaiquiri_webots
ros2 pkg executables thaiquiri_webots
```

Repeat the build after changing launch files, URDF/PROTO/world resources, package metadata, or entry points, then restart the launch. `--symlink-install` helps with Python edits, but already-running nodes still need restarting.

In **every additional terminal**, enter the container and source the overlay:

```bash
docker compose exec padoru_webots_jazzy bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ros_ws/install/setup.bash
cd /home/ubuntu/ros_ws
```

For native execution, use the uv setup above.

## 3. Run the Thaqhiri experiments

### Experiment A — simulation and sensor smoke test

In a sourced native terminal (or container terminal A):

```bash
ros2 launch thaiquiri_webots drone_launch.py rtabmap:=false
```

This opens `mine_thaiquiri.wbt` and starts the robot controller, robot-state publisher, Webots clock supervisor, camera adapters, stereo processing, IMU merger, and Madgwick filter. Processing uses simulation time. Use Webots' Play control if paused. Add `gui:=false` to minimize Webots and forward its console output (a display is still required).

`rtabmap:=false` skips RTAB-Map package lookup and startup. In this mode there is no global pose/odometry TF; visualize the robot using `base_footprint` as the fixed frame. With mapping enabled, the dynamic tree is `map -> odom -> base_footprint`; a static world-to-moving-body transform is not published.

In a second sourced terminal, inspect the running graph:

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /drone/altimeter_cube --once
ros2 topic echo /drone/velocity --once
ros2 topic hz /drone/cube/gyro
# Ctrl-C stops the rate monitor; run the next checks separately.
ros2 topic hz /imu/data
ros2 topic hz /drone/left/image_raw
ros2 topic hz /drone/lidar_livox/point_cloud
```

The camera/LiDAR names above are the names expected by this launch. These devices rely on Webots ROS 2's automatic device handling; confirm actual publishers with `ros2 topic list -t` if a stream is absent. Cube IMUs request 125 Hz for control and Livox IMUs request 10 Hz; achieved rates depend on simulator/bridge scheduling.

After IMU initialization, the controller automatically takes off toward an **altimeter reading of 0.5 m** (sensor elevation, not base-link height). Altitude PID, attitude damping/integral feedback, and payload trim replace the previous offset/cubic controller. There is no separate arming step or horizontal position hold; some lateral drift is expected.

### Experiment B — keyboard flight and controller observation

Use Experiment A's launch. In another sourced interactive terminal:

```bash
ros2 run thaiquiri_webots teleop_drone
```

Keep focus in the teleop terminal. Controls publish `geometry_msgs/msg/Twist` on `/cmd_pos`:

| Key | Command as implemented |
| --- | --- |
| `i` / `,` | Pitch disturbance `linear.x = -1.5 / +1.5`. |
| `j` / `l` | Roll disturbance `linear.y = +1 / -1`. |
| `u`, `o`, `m`, `.` | Combined pitch/roll commands for diagonal motion. |
| `t` / `b` | Increase/decrease the altitude target by `0.05` per received command. |
| `y` / `n` | Yaw disturbance `angular.z = +1.3 / -1.3`. |
| `k`, space, or another unbound key | Publish zero disturbance; retain the altitude target. |
| `Ctrl-C` | Publish zero disturbance and exit teleop. |

Despite the `Twist` message type, these are **controller disturbances and altitude increments**, not a standard velocity interface in m/s. Pitch/roll commands persist until replaced; releasing a key does not send a stop. Yaw decays by a factor of 0.7 per controller step. A zero command does not disarm the motors or land the aircraft.

For a command-line altitude increment instead of keyboard input:

```bash
ros2 topic pub --once /cmd_pos geometry_msgs/msg/Twist \
  '{linear: {z: 0.05}}'
```

Use one command source at a time. Observe `/drone/altimeter_cube` and `/drone/velocity` while applying small commands. Reset/reload the Webots world and restart the launch between comparable trials so the vehicle state and controller setpoint start consistently.

### Experiment C — stereo/LiDAR mapping with RTAB-Map

Stop the previous launch, then run:

```bash
ros2 launch thaiquiri_webots drone_launch.py rtabmap:=true
```

RTAB-Map is also enabled when the `rtabmap` argument is omitted. Its launch waits for the Webots controller connection and is configured for:

- Stereo input from `/drone/left/image_rect_color` and `/drone/right/image_rect` with corrected left/right camera information.
- Point-cloud input from `/drone/lidar_livox/point_cloud`.
- **ICP odometry enabled**, visual odometry disabled.
- ICP waits for `/imu/data` and uses a 15,000-point local map, 0.10 m
  sampling, and a 0.50 m correspondence search. Override `icp_odom_args` to
  tune odometry separately from SLAM. These settings address tracking loss
  with the dense 40-layer LiDAR; they still require a new flight trial.
- `icp_max_translation` exposes the separate correction acceptance limit
  (default 0.2 m) for controlled comparisons.
- SLAM consumes the odometry topic so tracking-loss covariance is preserved;
  ICP continues to publish the `odom -> base_footprint` transform.
- `base_footprint`, `odom`, and `map` frame names, approximate synchronization, and queue size 20.
- RViz disabled in the included launch.
- **`--delete_db_on_start`**, so the database at RTAB-Map's configured path is reset on startup. Export or copy results before restarting; remove that argument if persistence is wanted.

Start teleop in a second terminal and move through the environment while checking the rectified images, LiDAR stream, and RTAB-Map logs. Optional visualization:

```bash
rviz2
```

In RViz, choose `map` as Fixed Frame once available, and add TF, PointCloud2, Image, and map displays using the topics that actually appear in `ros2 topic list -t`. RTAB-Map explicitly uses namespace `/drone`, publishing `/drone/odom` and `/drone/mapData`. Stereo and mapping includes have separate scopes so their launch arguments do not leak into one another.

This is a mapping integration experiment. Short hover and altitude-step checks pass, but stereo correspondence quality and long-flight behavior still require evaluation before quantitative map or trajectory comparisons.

### Record a trial

From a separate sourced terminal at the workspace root, record the configured sensor/control pipeline:

```bash
mkdir -p experiment_data
ros2 bag record \
  -o "experiment_data/thaiquiri_$(date +%Y%m%d_%H%M%S)" \
  /cmd_pos /clock /tf /tf_static \
  /drone/altimeter_cube /drone/velocity \
  /drone/cube/acc /drone/cube/gyro /imu/data_raw /imu/data \
  /drone/left/image_raw /drone/right/image_raw \
  /drone/left/camera_info /drone/right/camera_info \
  /drone/left/image_rect_color /drone/right/image_rect \
  /drone/disparity /drone/lidar_livox/point_cloud
```

Add the actual RTAB-Map odometry/map topics from your running graph when recording Experiment C. Stop recording with `Ctrl-C`, then inspect a bag with `ros2 bag info /path/to/bag`. Data under this workspace is persisted through the host bind mount; `experiment_data/` is not currently ignored by Git.

For repeatable comparisons, record the Git revision and local changes, Webots/ROS package versions, world starting state, controller gains, launch settings, sensor settings, and command sequence. No ground-truth trajectory recorder or accuracy-scoring script is supplied.

### Stop an experiment

Stop the recorder and teleop with `Ctrl-C`, then stop the launch. Closing Webots also triggers launch shutdown. For Docker only, run on the host:

```bash
docker compose down
xhost -local:docker
```

## 4. Files and their purpose

### Repository and container files

| File | Purpose and execution |
| --- | --- |
| `readme.md` | This architecture, file, and experiment guide. |
| `requirements.in` / `requirements.txt` | Native uv Python requirements and resolved pins; install into the system-Python virtual environment described above. |
| `Dockerfile` | Builds the Jazzy desktop/Webots development image, installs RTAB-Map and Python libraries, configures the existing `ubuntu` user, and installs the entrypoint. Used by `docker compose build`. Torch/torchvision are installed but unused by the supplied nodes. |
| `docker-compose.yml` | Defines the active `padoru_webots_jazzy` service, GPU/X11 access, workspace mount, `webots_config` volume, and `drone_net` bridge. The ArduPilot service is commented out. |
| `entrypoint.sh` | Sources `/opt/ros/jazzy/setup.bash`, prints startup information, and executes the supplied command or Bash. Runs on container startup. |
| `.gitignore` | Ignores colcon `build/`, `install/`, `log/`, selected external directories, PDFs, Graphviz files, and VS Code settings. |
| `ardupilot_dev_docker/Dockerfile_dev-ros` | Optional multi-stage ArduPilot development image: Jazzy ROS base, build/Python tools, Micro-XRCE-DDS code generator, and STM32 cross-compiler. It does not clone/build ArduPilot itself or launch SITL. |

`.git/` contains Git's repository metadata rather than experiment code. `build/`, `install/`, and `log/` are generated by colcon; launch resolves assets from the installed package share directory.

### `src/thaiquiri_webots/` — package and launch

All paths in this table are relative to `src/thaiquiri_webots/`.

| File | Purpose and execution |
| --- | --- |
| `package.xml` | ROS package identity, Apache-2.0 declaration, runtime dependencies, and test dependencies. Read by ROS/colcon/rosdep. |
| `setup.py` | Installs the Python package, launch file, URDF, both worlds, PROTOs, meshes/materials/textures, and seven console-script entries including `sil_observer`. Does not install `disparity_node.yaml`. |
| `setup.cfg` | Places Python executables in `lib/thaiquiri_webots` so `ros2 run` can discover them. |
| `resource/thaiquiri_webots` | Empty ament resource-index marker used for package discovery. |
| `resource/thaiquiri.urdf` | ROS link/joint tree, simplified visual/collision geometry, Webots plugin selection, four IMU plugins, and altimeter configuration. Selects `thaiquiri_webots.drone_driver.DroneDriver`; read by the launch and Webots controller. |
| `launch/drone_launch.py` | The main entry point: `ros2 launch thaiquiri_webots drone_launch.py`. Starts the clock, simulation, processing, and optional mapping pipeline; shuts it down when Webots exits. ArduPilot helpers are separate executables, not part of this launch. |
| `disparity_node.yaml` | Saved stereo/disparity tuning values, including a 61-pixel correlation window, disparity range 384, and minimum disparity -350. Currently not loaded by the launch; it is a flat parameter list, not a ready-to-use node-scoped ROS parameter file. |
| `.gitignore` | Ignores a local `/drone_project` directory. |
| `LICENSE` | Apache License 2.0 text for this package. |

### `src/thaiquiri_webots/thaiquiri_webots/` — Python modules

| File | Purpose and execution |
| --- | --- |
| `__init__.py` | Empty Python package marker. |
| `drone_driver.py` | **Active Webots plugin.** Reads altitude and filtered body-frame IMU data; runs altitude/attitude feedback and bounded motor mixing. Publishes altitude and commanded motor-speed telemetry (not measured rotor speed). Loaded by the URDF via `init()`/`step()`; no standalone executable. |
| `teleop_drone.py` | Interactive keyboard publisher on `/cmd_pos`; run with `ros2 run thaiquiri_webots teleop_drone`. Uses Linux terminal APIs; its declared `stamped`/`frame_id` parameters do not change the published `Twist` message. |
| `bgra_to_bgr.py` | Removes the alpha channel from both Webots camera images using OpenCV/cv_bridge, preserving headers. Outputs `/drone/left/image_raw` and `/drone/right/image_raw`. Launched automatically; executable `bgra_to_bgr`. |
| `fix_camera_info.py` | Republishes left camera calibration and sets right projection matrix `P[3] = -fx * 0.05`, assuming a 5 cm stereo baseline. Launched automatically; executable `fix_camera_info`. |
| `imu_merger_node.py` | Synchronizes Cube acceleration/gyro (queue 10, tolerance 0.05 s), rotates vectors and covariance from native `(left, up, forward)` to ROS `(forward, left, up)`, and sets `imu_link`. Waits for 0.2 s of settled readings at startup; marks orientation unavailable for Madgwick. |
| `sil_observer.py` | Independent, spawned Webots supervisor publishing true body pose and contact counts for SIL scoring. Does not feed flight control, SLAM, or TF. |
| `sil_metrics.py` | First-pose trajectory alignment, scalar error summaries, and sampled-surface map error calculations used by the SIL runner. |
| `drone_driver_3.py` | **Inactive alternate plugin** wrapping `WebotsArduVehicle` plus altitude telemetry. Uses a hard-coded SITL address `172.18.0.3`, no GPS/InertialUnit device, and no reversed motors. Has no standalone executable. |
| `webots_vehicle.py` | Imported ArduPilot/Webots transport implementation. Exchanges binary UDP motor/sensor packets, applies motor scaling, and optionally streams camera/rangefinder data over TCP. Its background thread calls `robot.step()`. No command-line entry point. |
| `odom_to_ardupilot.py` | Optional `Odometry` to `/ap/tf` adapter. Validates input frame names, adjusts orientation using the URDF rotation and a 180-degree X rotation, copies position, and publishes hard-coded `odom -> base_link` frames using the node's current time. Executable `odom_to_ardupilot`; disabled in the main launch. |
| `gps_pose_publisher.py` | Optional 5 Hz publisher intended for `/ap/cmd_gps_pose`. Currently contains a field-name typo and publishes zero coordinates with all position fields ignored; it is not a working EKF-origin initializer. Executable `gps_pose_publisher`; disabled in the main launch. |

Avoid starting duplicate copies of sensor adapters while the main launch is running. For isolated debugging, their executables can be run with `ros2 run thaiquiri_webots <executable>`.

### `src/thaiquiri_webots/webots_world/` — environment and robot

| File | Purpose |
| --- | --- |
| `worlds/mine_thaiquiri.wbt` | R2025a world: 8 ms physics timestep, damping, 10 × 10 m arena with 5 m walls, soil-textured boxes, rocks, pipe passages, uneven terrain, and one `drone` using an external controller. Initial translation is approximately `(0, -3.8, 0)`. |
| `worlds/.mine_thaiquiri.wbproj` | Saved Webots editor layout, perspectives, and device-view settings; not simulation logic. |
| `worlds/.mine_thaiquiri.jpg` | Preview image of the quadrotor in the mine world. |
| `protos/thaiquiri.proto` | Full Webots robot: physical bodies/masses, collision shapes, meshes, four thrust-generating propellers, Cube sensors, stereo cameras, and Livox-labelled LiDAR/IMU devices. |
| `worlds/cerro_rico.wbt` | Imported full mine with a repeatable level launch fixture at the source spawn marker. |
| `protos/CerroRico.proto`, `protos/cerro_rico/` | Generated mine visual/collision meshes, packed textures, practical lights, geometry provenance, and a deterministic surface reference for SIL map scoring. |

The Webots PROTO defines physics; the URDF defines ROS frames and plugin configuration. Keep both consistent when changing geometry or sensor placement.

Key model settings:

- Stereo cameras: 640 × 480, field of view `1.91986` rad, near/far planes 0.1/20 m, purely lateral 5 cm separation. The former 3 mm right-camera depth offset was removed in both PROTO and URDF to match the projection model. URDF camera links describe optical axes; Webots Camera nodes use a different axis convention.
- LiDAR: rotating scan, horizontal resolution 1000, 40 layers, approximately 360° horizontal and 59° vertical field of view, 0.1–40 m range. This is a configured Webots LiDAR, not a reproduced Livox non-repetitive scanning pattern.
- Motors: maximum velocity 930 rad/s; propeller thrust constants have magnitude `4.43e-05` with alternating signs. In Webots R2025a, propeller kinematic spin-up uses `min(acceleration, maxTorque)`. Both limits are now 2000, allowing roughly 0.24 s spin-up instead of the former minute-long ramp. This numerical motor model is not a calibrated physical motor-torque specification.
- Active controller: nominal hover speed 469 rad/s; altitude P/D/I gains 60/35/10; roll/pitch P/D/I gains 180/40/30; yaw-rate damping 10. Roll/pitch speed trims -1/+11 compensate the modeled payload imbalance. Integral terms and motor magnitudes are bounded. Values are in `drone_driver.py` and require retuning if mass/geometry changes.

#### Mesh and material inventory

Every row below represents **two files** in `webots_world/protos/meshes/`: `<name>.obj` contains vertices/faces/material assignments, and `<name>.mtl` contains the corresponding material properties. Webots loads these as assets; they are not executable programs.

| File stem (`.obj` and `.mtl`) | Representation / current use |
| --- | --- |
| `frame` | Main frame, decks, booms, landing gear, and fittings. |
| `battery` | Battery pack, retaining straps, and markings. |
| `camera` | Stereo camera housing, lenses, and mounting bracket. |
| `lidar` | LiDAR housing, optical surfaces, and heat-sink details. |
| `cube_flight_controller` | Cube Orange, carrier board, isolation feet, and connectors. |
| `cube_power_module` | Cube power-module board. |
| `DC_regulator` | DC/DC regulator board and components. |
| `esc mosfet` | Power-distribution board and ESC MOSFETs; filename contains a space. |
| `gas_sensor` | Gas instrumentation pod and CO/H2S/NO2/O2-labelled components; visual only. |
| `gas_amplifier` | Gas instrumentation amplifier/ADC board; visual only. |
| `jetson` | Companion-computer enclosure/board and ports; visual only. |
| `stm_interface` | STM32 interface board; visual only. |
| `telemetry_whip` | Telemetry radio/antenna representation. |
| `wires` | Power, signal, camera, LiDAR, and telemetry wiring harnesses. |
| `motor` | Motor mesh reused at all four motor positions. |
| `motor_2` | Alternate motor export, currently not referenced by the PROTO. |
| `propeller` | Alternate propeller export, currently not referenced by the PROTO. |
| `propeller_2` | Propeller mesh used for all four fast/slow helix representations. |

### `src/ardupilot_msgs/` — interface definitions

| File | Purpose |
| --- | --- |
| `package.xml` | Declares the ament CMake interface package, dependencies, and GPL-3.0 license. |
| `CMakeLists.txt` | Uses `rosidl_generate_interfaces` to generate the four messages and three services; configures interface lint checks. |
| `msg/Airspeed.msg` | Stamped true-airspeed vector in ROS axes and equivalent-to-true airspeed conversion factor. |
| `msg/GlobalPosition.msg` | Global position/velocity/acceleration goal fields, coordinate-frame constants, ignore flags, and yaw. |
| `msg/Rc.msg` | RC connection state, RSSI, up to 32 channel values, and override flags. |
| `msg/Status.msg` | Vehicle type, arm/mode/flying/external-control state, and active failsafes. |
| `srv/ArmMotors.srv` | Boolean arm/disarm request and success result. |
| `srv/ModeSwitch.srv` | Numeric mode request; returns status and current mode. |
| `srv/Takeoff.srv` | Requested takeoff altitude and status response. |

Inspect generated interfaces after building, for example:

```bash
ros2 interface show ardupilot_msgs/msg/GlobalPosition
ros2 interface show ardupilot_msgs/srv/ArmMotors
```

### Tests

The files under `src/thaiquiri_webots/test/` include regression and lint checks:

- `test_control.py`: quaternion conversion, invalid attitudes, motor correction signs/limits, startup waiting for a received IMU, one-shot altitude increments, IMU axis/covariance conversion, and settled initialization.

- `test_flake8.py`: Python style checking through `ament_flake8`.
- `test_pep257.py`: docstring conventions through `ament_pep257`.
- `test_copyright.py`: copyright-header check, currently explicitly skipped.

Run the existing checks inside the built/sourced workspace:

```bash
python -m colcon test --packages-select ardupilot_msgs thaiquiri_webots
python -m colcon test-result --all --verbose
```

These checks do not verify flight stability, sensor calibration, mapping accuracy, or SITL integration.

Run the focused control regression checks in the sourced uv environment:

```bash
python -m pytest -q src/thaiquiri_webots/test/test_control.py
```

## 5. ArduPilot integration: current state and execution path

**ArduPilot experiments require additional integration work.** The active Compose file starts only Webots. There is no supplied ArduPilot source tree, SITL ROS launch package, Micro-XRCE-DDS Agent runtime setup, or Thaiquiri flight-parameter file. `ardupilot_msgs` is only the message/service schema.

The available pieces can be connected as follows:

1. Build the optional development image if needed:

   ```bash
   docker build -f ardupilot_dev_docker/Dockerfile_dev-ros \
     -t ardupilot_sitl ardupilot_dev_docker
   ```

2. Supply an external, compatible ArduPilot workspace and its build/dependency setup. The commented Compose service expects a host directory `../ardu_ws` mounted at `/home/ubuntu/ros_ws`. Uncomment/configure that service and build/start it only after providing the workspace. The Dockerfile supplies tools, not the workspace or a running autopilot.
3. To select the alternate controller, change the URDF plugin from `thaiquiri_webots.drone_driver.DroneDriver` to `thaiquiri_webots.drone_driver_3.DroneDriver`, then rebuild. There is no launch argument that switches drivers.
4. Replace `drone_driver_3.py`'s fixed `172.18.0.3` address with the reachable SITL address/service name. The Compose `SITL_UDP_IP` variable is currently unused by the Python code. Configure SITL's Webots backend to send motor packets to the Webots container.
5. For instance 0, the bridge binds UDP **9002** for controls and sends flight-dynamics packets to the SITL address on **9003**. Instance `n` adds `10*n` to both ports. Keep both containers on the same reachable network. No host MAVLink ports are published by the current Compose file.
6. Validate motor channel ordering/reversal, axes, and sensor inputs before flight. The alternate driver currently passes `reversed_motors=None` despite the model's alternating thrust signs, and supplies no orientation/GPS devices, causing orientation, position, and velocity fields from those devices to be zero. The transport assumes propeller linearization compatible with `MOT_THST_EXPO=0`.
7. Resolve simulation-step ownership: `webots_vehicle.py` steps the robot in its background transport thread, while the ROS Webots controller also owns a stepping loop. This integration needs verification rather than assuming the alternate plugin is drop-in compatible.
8. If using ROS/DDS external navigation, configure the external DDS-capable SITL and Agent, use compatible interface definitions, and repair/configure the optional bridge nodes described below. Use the same `ROS_DOMAIN_ID=0` and compatible ROS discovery settings across participants.

The odometry adapter can be exercised independently against a real odometry publisher, without a running autopilot:

```bash
# Replace /your/odometry with the discovered topic, and set the input frame
# parameters to the frame IDs carried by its messages.
ros2 run thaiquiri_webots odom_to_ardupilot --ros-args \
  -p odom_topic:=/your/odometry \
  -p parent_frame:=odom -p child_frame:=base_footprint
```

It publishes `/ap/tf`, but that alone does not make ArduPilot consume it. The node applies orientation corrections and hard-codes output frames despite the introductory docstring saying it only copies the pose. Validate those transformations against the external navigation convention you use.

When adding the optional bridge nodes to a launch, select the appropriate clock explicitly: `/clock` for Webots or a verified `/ap/clock` publisher for an ArduPilot-driven simulation.

## 6. Current limitations and required corrections

These observations describe the checked-in implementation and explain common failures during the experiments.

| Symptom / issue | Cause and next step |
| --- | --- |
| Missing launch package or Python module | Install section 2's dependencies, rebuild, and source the ROS/uv/workspace environment. |
| ICP odometry exits with a Fast CDR `undefined symbol` error, or node creation reports `This member is not been selected` | Mixing new RTAB-Map/middleware binaries with older ROS message/type-support libraries caused this on the native host. Update **all installed Jazzy packages together** using the `--only-upgrade` command in section 2; updating only Fast CDR/Fast DDS is insufficient. Stop ROS processes, run `ros2 daemon stop`, rebuild with `--cmake-clean-cache`, and restart. This is a native ROS dependency issue, not a uv package. |
| Motors remain stopped at startup | Check `/clock`, Cube IMU streams, then `/imu/data`. The filter waits for settled acceleration/gyro readings; the driver waits for a received valid orientation. |
| Drone drifts sideways | The controller stabilizes attitude and altitude, not horizontal position. Payload changes also require trim/gain tuning. |
| Stereo correspondence warnings | Geometry now matches the 5 cm projection model, but broad-FOV scene texture, disparity range, and calibration still need quantitative evaluation. Do not treat a nonempty map as an accuracy test. |
| No global TF with `rtabmap:=false` | Expected: the previous incorrect static `world -> base_footprint` was removed. Use `base_footprint` for sensor-only visualization or enable mapping. |
| Editing `disparity_node.yaml` has no effect | The file is neither installed nor loaded. Wire desired parameters into the stereo launch/component configuration, or create a node-scoped `ros__parameters` file and explicitly load it. |
| `ros2 run thaiquiri_webots drone_driver` fails | It is a Webots plugin; the invalid console entry was removed. Use `drone_launch.py`. |
| GPS helper crashes or fails to establish an origin | `msg.coordiante_frame` is misspelled; the actual interface field is `coordinate_frame`. Even after fixing it, all coordinates are zero and position ignore flags are set. Verify the correct ArduPilot interface for setting an EKF origin instead of assuming this goal publisher initializes one. |
| SITL does not connect | Service is disabled, source workspace is absent, driver is inactive, and the alternate driver uses a fixed IP. Complete section 5 first. `SITL_UDP_IP` alone does not configure the bridge. |
| Webots environment variable looks wrong | Compose sets `WBOTS_HOME` (typo), but the Dockerfile sets the correct `WEBOTS_HOME`. Use `WEBOTS_HOME` when adapting the setup. |
| No graphics / GPU device-selection error | Check host GPU driver/toolkit, `/dev/dri`, X11 socket, and `DISPLAY`; adapt Compose for the actual GPU. |
| Changes appear to be ignored | Launch loads resources from the installed package share directory. Rebuild, source `install/setup.bash` in the launching terminal, and restart the launch. |

Useful diagnostics:

```bash
ros2 topic info -v /imu/data
ros2 topic echo /clock --once
ros2 topic info -v /drone/lidar_livox/point_cloud
ros2 run tf2_ros tf2_echo base_footprint camera_left
```

On the host, use `docker compose ps` and `docker compose logs padoru_webots_jazzy` for container startup diagnostics. ROS processes started via `docker compose exec` write primarily to their own terminals and ROS logs.

### Validation scope

**Implementation debugging — 2026-10-13:**

- Ten focused regression tests pass (control math/mixing, startup gating, one-shot altitude commands, IMU conversion/initialization, and sensor geometry). Focused flake8 checks pass for the driver, IMU merger, launch file, and regression tests.
- Native sensor-only takeoff/hover and a 60-second mapping run were exercised. A +0.1 m altitude command changed the target from 0.5 to 0.6 m; after settling, measured sensor altitude ranged from **0.5997 to 0.6022 m**. This is a short altitude/attitude test, not a horizontal-position-hold or long-flight validation.
- The mapping run received **469 odometry messages, 48 map updates, 4,050 filtered IMU messages, 5,912 rectified left images, and 492 LiDAR clouds**. Empty-IMU-frame and excessive stereo-correspondence warnings were absent after fixing the IMU frames and stereo baseline geometry.
- Runtime parameter queries confirmed simulation time on robot-state publishing, image/IMU processing, stereo disparity, ICP odometry, and RTAB-Map. Only the Webots clock supervisor uses wall time, to avoid waiting on the clock it produces.
- Python adapter shutdown now handles interrupted/already-shut-down ROS contexts, and Webots exit no longer requests a second launch shutdown. The installed upstream Webots supervisor can still exit with signal code -2 on Ctrl-C; an upstream component-container teardown failure was also seen in earlier mapping runs. These shutdown reports are distinct from the validated runtime data flow.
- Map accuracy, all keyboard maneuvers, and SITL remain unvalidated. Logs from the latest run are in `/tmp/opencode/final-validation.log` and `/tmp/opencode/trial-launch.log` on the test host (temporary files).

**Initial installation smoke test (before the implementation fixes):**

Native execution was smoke-tested on **2026-10-12** with Ubuntu 24.04, system Python 3.12.3, uv 0.8.3, ROS 2 Jazzy (updated installed packages), Webots R2025a, and the pinned Python requirements:

- Both packages built successfully using the uv environment; Python ROS/OpenCV imports and generated ArduPilot interfaces loaded successfully. `uv pip check` passed.
- `ros2 launch thaiquiri_webots drone_launch.py rtabmap:=false` connected the controller and delivered altitude, motor telemetry, filtered IMU, raw/rectified camera images, and LiDAR messages during a 20-second subscription check.
- The default `ros2 launch thaiquiri_webots drone_launch.py` also ran. A 30-second check received **190 `/drone/odom` messages, 27 `/drone/mapData` updates, 227 filtered IMU messages, 2,710 rectified left images, and 189 LiDAR clouds**. These are observed message counts, not guaranteed performance rates.
- That initial run emitted empty-source-frame transform warnings and validated startup/data flow only. The later debugging checks above supersede its implementation findings. Keyboard flight and SITL were not validated.

During native dependency installation, apt also reported an NVIDIA DKMS failure for the host's pending `7.0.0-38-generic` kernel. The required ROS packages nevertheless installed, and the currently running `6.17.0-20-generic` kernel/NVIDIA driver supported these tests. That separate host package-configuration issue was not repaired by this setup.

The earlier documentation-only check `docker compose config --quiet` passed; Docker execution was not used for the native validation above.
