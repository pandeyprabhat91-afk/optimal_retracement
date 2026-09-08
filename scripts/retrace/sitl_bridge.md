# SITL visualisation runbook (Docker, Windows 11 + WSL2)

Source: `riotu-lab/gps_denied_navigation_sim` + `riotu-lab/gps_denied_navigation_docker`
READMEs (verified 2026-09-07). No code here — copy-paste commands only.

## 0. Prerequisites

- Docker 29+ with WSL2 backend, `git`, SSH key on GitHub.
- RViz/Gazebo render via WSLg (no native Windows Gazebo).

## 1. Build image

```bash
git clone https://github.com/riotu-lab/gps_denied_navigation_docker.git
cd gps_denied_navigation_docker/docker
make px4-simulation-cuda12.2.0-ubuntu22   # skip CUDA variant if no NVIDIA GPU
```

## 2. Enter container

```bash
cd gps_denied_navigation_docker
./docker_run.sh        # native Linux
./docker_run_wsl.sh    # from WSL2 on Windows
# container name: gpsdnav, user: user
```

## 3. Workspace bootstrap (inside container)

```bash
mkdir -p ~/shared_volume/ros2_ws/src && cd ~/shared_volume/ros2_ws/src
git clone git@github.com:riotu-lab/gps_denied_navigation_sim.git
cd gps_denied_navigation_sim && ./install.sh
# install.sh builds PX4-Autopilot (navsat_callback branch), clones mavros,
# mavlink, yolov8_ros, tercom_nav, tercom_rviz_plugins, rosdep build
```

## 4. Run default TERCOM pipeline (taif_test4)

| Terminal | Command      | Purpose                           |
| -------- | ------------ | --------------------------------- |
| 1        | `zenoh`      | Zenoh RMW daemon                  |
| 2        | `mono_taif4` | PX4 SITL + Gazebo + MAVROS + RViz |
| 3        | `tercom`     | TERCOM + ESKF + DEM server + diag |

Other world/UAV aliases: `mono_tug mono_taif mono_taif1 stereo_* twin_*`.

## 5. Retrace overlay wiring

- Ground truth: `/target/gt_path` (from MAVROS, every launch file).
- Default estimate: `/tercom/eskf_node/odom`.
- Retrace estimate: publish replay/`run_replay` output as `nav_msgs/Path`
  on `/retrace/est_path` (one node, reads `output/retrace/results.csv`
  trail or live estimator).
- RViz: add `/retrace/est_path` to pre-baked TERCOM layout for
  truth-vs-retrace visual.

## 6. Scoring (reuse their tools)

```bash
# online Euclidean/angular/velocity error -> CSV
ros2 run gps_denied_navigation_sim run_path_error_analysis.py
# 15 publication figures from diagnostics CSV (tercom_nav)
python3 analyze_tercom_log.py <diagnostics.csv>
```

Compare `/target/gt_path` vs `/retrace/est_path` per combo C1-C6,
horizons 10/30/60 s, same gates as `scripts/retrace/combos.yaml`.

## 7. GPS-loss simulation in SITL

- PX4 v1.13+: `param set EKF2_AID_MASK 24` + `EKF2_HGT_MODE 3` (vision pose),
  or remove GPS plugin from `iris.sdf`.
- ArduPilot SITL alt: `GPS_TYPE=0` / Lua `copter-deadreckon-home.lua`
  (`Guided_NoGPS` lean-home) for Exp1 closed-loop confirm.
