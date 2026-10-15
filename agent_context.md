# Agent context: Thaiquiri / Thaqhiri Webots UAV

> Handoff snapshot: 2026-10-14. Read this first, then verify the current Git status, source files, and trial metadata before changing code. This file describes **this checkout**, not every related working copy.

## Mission and boundaries

The repository is a ROS 2 Jazzy workspace for exploring a Thaiquiri quadrotor in a Webots mine, including manual teleoperation, stereo imaging, a rotating LiDAR, IMU processing, ICP odometry, and RTAB-Map mapping. The long-term idea is a GNSS-denied mine-survey UAV with possible ArduPilot integration. The **active** simulation uses a Python Webots flight plugin, **not ArduPilot SITL**. `ardupilot_msgs` supplies interfaces; the alternate ArduPilot bridge/controller is incomplete and not exercised by the standard launch or SIL trial. The gas sensor is a visual payload only: there is no gas-concentration physics, reading, or ROS topic. Do not describe these unimplemented capabilities as validated.

Names matter: the repository is `webots_thaqhiri`, but its ROS package, modules, and robot assets are `thaiquiri_webots`/`thaiquiri`. The old Docker service is named `padoru_webots_jazzy`.

## Repository map and data flow

| Path | Role |
| --- | --- |
| `src/thaiquiri_webots/launch/drone_launch.py` | Main launch; chooses `world:=mine_thaiquiri.wbt` (default) or `world:=cerro_rico.wbt`; optional `rtabmap:=false`, `gui:=false`, `rtabmap_viz:=false`, `database_path:=...`. ROS uses `/clock` from the Webots supervisor; the supervisor itself must use wall time. |
| `src/thaiquiri_webots/resource/thaiquiri.urdf` | ROS frames, Cube sensor plugins, and active Python Webots plugin selection. |
| `src/thaiquiri_webots/webots_world/protos/thaiquiri.proto` | Physical drone, propellers, stereo cameras, LiDAR, payload meshes, sensors, and collisions. URDF and PROTO must agree on extrinsics. |
| `src/thaiquiri_webots/thaiquiri_webots/drone_driver.py` | Active Webots flight controller; reads `/imu/data` and Webots altimeter, subscribes to `/cmd_pos`, drives four motors, publishes `/drone/altimeter_cube` and `/drone/velocity`. The latter is **commanded**, not measured, rotor velocity. |
| `imu_merger_node.py`, `bgra_to_bgr.py`, `fix_camera_info.py` in the same module directory | Native Cube axes to ROS FLU IMU and camera adapters. Madgwick outputs `/imu/data`; camera adapters produce `/drone/left` and `/drone/right` inputs for stereo processing. |
| `src/thaiquiri_webots/webots_world/worlds/mine_thaiquiri.wbt` | Original small mine-like arena. |
| `02_Cerro_Rico_Messy.blend`, `scripts/export_cerro_rico.py`, `src/thaiquiri_webots/webots_world/worlds/cerro_rico.wbt`, `.../protos/CerroRico.proto`, `.../protos/cerro_rico/` | Authoring scene, reproducible export, full mine world, visual/physics assets, texture files, manifest, sampled collision-surface reference. |
| `scripts/run_sil.py`, `config/sil_criteria.json`, `src/thaiquiri_webots/thaiquiri_webots/sil_observer.py`, `sil_metrics.py` | Entrance-corridor test fixture, thresholds, independent simulator truth/contact observer, and scoring. Read the **runner version associated with a result**, not merely the version in this checkout. |
| `src/ardupilot_msgs/` | Generated ArduPilot ROS messages/services only; does not launch firmware or an autopilot. |
| `docs/cerro_rico_sil.md`, `validation/cerro_rico/report.md`, `readme.md` | Mine/SIL protocol, older checked-in test evidence, and general setup/reference. |
| `experiment_data/sil/<run>/` | Ignored per-run outputs: `result.json`, `report.md`, CSVs, camera sample, map samples, `rtabmap.db`, and logs. These are **not** Git-tracked by default. |

Control/data flow: keyboard or SIL test fixture publishes `geometry_msgs/Twist` to `/cmd_pos`; these values are controller disturbances and **altitude increments**, not standard velocity setpoints. Webots Cube gyro/acceleration -> IMU merger (`/imu/data_raw`, FLU `imu_link`) -> Madgwick (`/imu/data`) -> flight controller. Cameras -> BGRA adapter and camera-info correction -> rectification/stereo sync; LiDAR -> ICP odometry and RTAB-Map. The stereo pair is packaged as `RGBDImage` to synchronize it with the LiDAR for mapping. The independent `/sil/ground_truth` and `/sil/contact_count` are for **evaluation only** in the normal odometry-feedback trial; the runner's `ground_truth` control mode is diagnostic and cannot validate autonomous localization.

## Host setup and commands

Validated host family: Ubuntu 24.04 x86-64, ROS 2 Jazzy, system Python 3.12, Webots **R2025a**, graphical display, `uv`. ROS binaries, Webots, and OpenCV/cv_bridge are native packages; `uv` does **not** install them. Full apt/uv installation instructions are in `readme.md`. Python lives in a **system-Python `--system-site-packages` venv** with NumPy < 2 for Ubuntu OpenCV binary compatibility; `requirements.in` records direct requirements, `requirements.txt` the resolved pins. Avoid a different Python interpreter when rebuilding ROS interfaces or installing entry points.

From the repository root after native prerequisites are installed:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python -m colcon build --symlink-install --packages-select ardupilot_msgs thaiquiri_webots \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
source install/setup.bash
export WEBOTS_HOME="$HOME/.local/opt/webots"
ros2 launch thaiquiri_webots drone_launch.py world:=cerro_rico.wbt
```

Use `python -m colcon` from the venv for consistent generated node shebangs. Build after editing launch/URDF/PROTO/world/install metadata. Source ROS, the venv, and `install/setup.bash` in **every** additional terminal. To run sensor-only, add `rtabmap:=false`. Mapping launches use `--delete_db_on_start`; use a dedicated database path or preserve your results before relaunching. Do not run concurrent Webots instances on port 1234 when using the SIL runner.

The checked-in mine assets can be regenerated without modifying the source `.blend` using Blender 4.2.0:

```bash
~/.local/opt/blender-4.2.0-linux-x64/blender -b 02_Cerro_Rico_Messy.blend \
  --python scripts/export_cerro_rico.py
python -m colcon build --symlink-install --packages-select thaiquiri_webots
```

The scene is metre-scale and approximately 100 × 94 × 7.7 m. The artist supplied a simplified static **SIM COLLISION** mesh; small visual debris not in it does not collide. Practical lights are relative, **not calibrated lux**. A level launch pad was added at the authoring spawn marker `(0, 3)` for reproducible IMU initialization. The Webots altimeter measures world elevation, not height over the changing mine floor.

## Revision history (distinguish commits from experimental runs)

The current `thaiquiri` branch in **this checkout** is at `734a8ef` (also `origin/thaiquiri` when this file was written). At the time of writing `git status --short` was empty. Relevant actual commits, oldest to newest:

- `3697bef` initial repository; `c3b1873` adds Thaiquiri model; `91d9de3`–`3203e23` progressively add quadrotor control, sensors, URDF/launch/stereo/RTAB-Map, teleoperation, and ArduPilot message interfaces. Inspect `git log` for exact file-level provenance.
- `92344ad` adds an earlier Thaiquiri README.
- `e474ed0` commits the native `uv` setup and extensive debugging: source/package cleanup; IMU topic, frame, axis, and quaternion fixes; startup gating; ROS simulation clock/TF corrections; controller/motor tuning; stereo baseline correction; regression tests. During the initial native setup, newly installed RTAB-Map exposed a mismatch with older ROS middleware packages: the installed Jazzy packages had to be upgraded together. An unrelated NVIDIA DKMS failure on a pending kernel was observed; it was not part of the project fix.
- `734a8ef` commits the Blender mine import, exported Webots assets, SIL runner/observer/criteria, tests, and the **older** checked-in nominal/low-light evidence in `validation/cerro_rico/`. Those October 13 pilot runs failed overall, notably because valid ICP tracking ended around 48.6 s / 44.3 s in 130 s trials. Their apparent odometry-rate success initially counted invalid/null messages; this interpretation is corrected in the validation docs. Old reports are historical evidence, **not** the latest `experiment_data/sil/nominal` run.

There is a **separate working copy** at `/home/daguill/3thaqhiri/thaiquiri-drone-fixed`. It contains later experimental SIL revisions (for example `sil_route.py` and `imu_validation.py`), a v3 criteria file, and tunable launch options such as `icp_max_translation` and `imu_filter_gain`. **Those revisions are not present in this checkout's `scripts/run_sil.py`/`config/sil_criteria.json`.** Do not silently mix, overwrite, or claim they were merged. Compare hashes, diffs, setup/build paths, and Git state if asked to import them. The later run's metadata stores implementation SHA-256 values but `git_revision`/`git_dirty` are null because that copy was an archive without its own repository root.

## SIL method and findings

The local pilot protocol (`docs/cerro_rico_sil.md`) is a 130 simulated-second entrance-corridor trial: raise the altimeter target at 5 s and 15 s, command a ~3 m outbound route from 20 s and return after 70 s. The odometry-driven test fixture is **not a production planner/obstacle avoider**. A separate Webots supervisor measures true pose and contact counts without feeding the flight controller or RTAB-Map. Scoring compares ICP pose to truth by an initial SE(3) alignment **without fitting scale**; map-surface P95 is distance to sampled **simulation collision** geometry, not surveyed map accuracy or coverage. `low_light` disables practical PointLights but emissive glass remains: it is a stress case, not calibrated darkness. Thresholds are engineering targets, not certification limits. For this checkout's runner, use a **new** output directory (the runner refuses to overwrite an existing directory):

```bash
python scripts/run_sil.py --scenario nominal --output experiment_data/sil/<new-nominal-id>
python scripts/run_sil.py --scenario low_light --output experiment_data/sil/<new-low-light-id>
python -m pytest -q src/thaiquiri_webots/test/test_control.py \
  src/thaiquiri_webots/test/test_sil_metrics.py \
  src/thaiquiri_webots/test/test_cerro_assets.py
```

Historical checked-in evidence: `validation/cerro_rico/report.md` (Oct 13, `734a8ef`) shows **FAIL** for both nominal and low light. Nominal altitude RMSE ~0.0091 m, odometry RMSE ~0.2518 m **over available estimates only**, and last-valid odometry stamp 48.568 s. Low light last-valid stamp 44.304 s. Return failed in both because odometry stopped before the scheduled return; the route fixture stopped horizontal commands when feedback became stale. ICP logs showed correspondence/registration failures. Camera contrast was below threshold. Do not treat `end_drift_m` from those runs as a 130 s endpoint error.

**Newer unmerged result reviewed 2026-10-14:** `experiment_data/sil/nominal/` in this workspace was produced by the **other** `/home/daguill/3thaqhiri/thaiquiri-drone-fixed` code, not by this checkout. Its `result.json` and `report.md` are the authoritative records for that run; the source for its scoring is the other copy's `scripts/run_sil.py` (hash `0b5ba03f...` in result metadata). It used `control_mode=odometry`, criteria v3, `imu_filter_gain=0.01`, ICP maximum correction 0.2 m, and a 0.2 m/s **requested** speed limit (not a physical speed cap). Valid odometry was retained to 129.896 s of a 130 s run (1115 matched poses, 0.312 s max gap); the outbound/return route executed. Results: altitude RMSE **0.00109 m**, attitude-estimation RMSE **0.426°**, odometry position RMSE **0.0283 m**, final matched-pose drift **0.0193 m**, route progress **3.06 m**, return error **0.063 m**, map-to-sampled-surface P95 **0.162 m**, ~21,732 evaluated map points. **Overall FAIL** (21/23 checks passed): settled horizontal-speed RMSE **0.0787 m/s** exceeds 0.05, and median grayscale contrast **5.11** is below 8. `navigation_validated=false`. Post-takeoff contact and motor-saturation checks passed; the source logs contain forced shutdown/component-container teardown errors after data collection. This is a much better single run but neither production navigation qualification nor a statistical repeatability study. No equivalent later low-light result was examined in that chat.

For any trial, first read `metadata` (`execution_error`, source and criteria hashes, control mode, version), inspect `odometry_status.csv`/last valid pose and gaps, truth/odometry/commands CSVs, and check camera/launch/observer logs. A publisher may emit invalid odometry while appearing to have a healthy message rate. Contact sampling starts after initial takeoff, so pad contact before 10 s can be expected. Compare frame and alignment assumptions before treating map-surface residuals as absolute map quality.

## Open issues and priorities

1. **Reproducibility:** reconcile the later `thaiquiri-drone-fixed` experiments with this committed branch deliberately; maintain a clean run provenance and update docs/evidence only after rerunning with matching sources. Do not alter or delete the user's other checkout or old experiments.
2. **Flight/navigation:** repeat nominal and low-light trials with identical code; characterize settled horizontal residual velocity, actual speed versus requested limit, and tracking-loss behavior. Add valid-estimate availability/recovery criteria if a runner lacks them. Test position control against simulator truth only as an explicitly marked diagnostic, never as evidence of odometry-driven autonomy.
3. **Sensing/mapping:** improve illumination/exposure and camera contrast, evaluate loop closures, measure map completeness as well as surface proximity, and challenge ICP in other corridors, obstacles, and lighting. Beware of the small 5 cm stereo baseline and simulation-only camera calibration.
4. **Scope:** full-mine autonomy, collisions on small decorative rubble, dust/gas, battery endurance, fault tolerance, communications loss, real hardware, and ArduPilot firmware-in-loop are **not validated**. Webots default physics/motors and idealized sensing require hardware identification before transferring performance claims to a real UAV.

For more detail: `readme.md` (installation/architecture), `docs/cerro_rico_sil.md` (pilot trial protocol/limits), `validation/cerro_rico/report.md` (checked-in older evidence), and each run's `result.json`/`report.md`/CSV/`launch.log` (specific measured evidence). Treat this context as a map, not a substitute for checking the live code and artifacts.
