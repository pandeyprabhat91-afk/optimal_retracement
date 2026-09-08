# SITL Handover — GPS-loss retrace live validation

Date: 2026-09-08. Branch `optimal-retracement` (repo `E:\kp_vio`). SITL root `E:\gpsdnav` (image + container + shared_volume, NOT in git).

## Fixes landed 2026-09-08 (uncommitted: E:\gpsdnav; plan: E:\kp_vio\docs\superpowers\plans\2026-09-08-sitl-arm-and-retrace-validation.md)

1. Root cause of arm denial: COM_ARM_EKF_POS/VEL/HGT/YAW were -1.0 (set by shared_volume/verify_params.py). estimatorCheck.cpp uses `test_ratio > param` → -1 always fails. Set 10.0 live + `param save` + patched script to 10.0 (dropped BAT_MIN -1). EKF itself healthy (ratios 0.00-0.06, gps_check 0).
2. MAVROS TX fixed: stock fcu_url `udp://:14540@127.0.0.1:14557` sends to dead port (px4-rc.mavlink: onboard listens 14580). Relaunched MAVROS with `fcu_url:=udp://:14540@127.0.0.1:14580` → TX proven (instance #1 rx 280 B/s), services usable. Handover rule "MAVROS services unusable → pymavlink direct" is STALE.
3. Commander rewritten MAVROS-native (retrace_nav/commander.py): setpoints via /target/mavros/setpoint_position/local, OFFBOARD+arm+LAND via set_mode/arming services, verify via /target/mavros/state. Raw pymavlink TX dead (GCS 18570 ignores us; 14550 RX-only broadcast, instance later died entirely). Keep spinning ROS while waiting (verify loops starved the 10 Hz setpoint stream → OFFBOARD dropped).
4. EKF GPS-drift degrades on ground (~10 min): pre_flt innov + HDRIFT/VDRIFT fail. Widened EKF2_REQ_HDRIFT/VDRIFT 0.1/0.2→5.0 + saved. If stale, reboot PX4 (keep gz), arm+fly fast.
5. retrace_node frame fix: MAVROS FLU/ENU → estimator FRD/NED at boundary (body_flu_to_frd, world_enu_to_ned, odom_quat_to_ned_yaw, baro sign, ENU republish) + hover guard (>1000). Without it live C4 diverged 1097 m on a 100.9 m flight.
6. Replay hygiene (learned hard, 2 wasted runs): ONE publisher on /retrace/est_path (kill stale nodes first); play WITHOUT recorded est/gt topics (`--topics` filter — bag replays old live est); NO use_sim_time/--clock (frozen/jumping sim clock skips cal → zero bias + 1.1 m/s² tilt leak); wall clock + `-r 1`; STOP MAVROS during replay (live static IMU dilutes playback maneuvers).
7. SITL mag fine (PX4 mag device SIM present, strength matches ref) despite no gz mag topic. No SDF fix needed.
8. GCS mavlink instance (18570→14550) observed DEAD while onboard link healthy. All tooling must use MAVROS topics/services, never raw 14550. arm_test.py/hb_test.py/ob_test.py (host-side pymavlink) are obsolete.

## Flight evidence

- flight_sitl5 (bags/flight_sitl5): square 20 m @5 m, path 100.9 m, z_max 5.08 m, home miss 0.26 m (PX4 GPS flight excellent). Scored via shared_volume/score_sitl.py + score_aligned.py (bag-t interpolation; naive index align is WRONG: 10 Hz est vs 30 Hz truth).
- Static SITL IMU carries ~1.1 m/s² FRD-y tilt (6° rest tilt) — absorbed by cal bias; do NOT "fix".
- Replay C1–C6 on flight_sitl5 via shared_volume/replay_all.sh → shared_volume/replay/est_Cx → score_aligned.py. PENDING at write time.

## Final results 2026-09-08 (flight_sitl6, gentle ramp climb, 114.5 m, PX4 home 0.36 m)

Replay C1-C4 (wall clock -r1, MAVROS dead, single publisher, topic-filtered play).
C5/C6 skipped deliberately (battery nil offline; square has no hover for ZUPT).
Scores = time-aligned ATE vs odom truth (output/retrace/sitl_results.csv):

- C1: ATE 2690 m (horiz), C2: 2677, C3: 2665 + vert 24→0.3 m, C4: 423 m (6.3× cut)
- Hierarchy reproduces paper exactly (compass nil, baro vertical-only, drag dominates); scale ~50× real logs.
- Root causes of scale, all measured: MAVROS IMU 49 Hz vibration-rectification bias (+0.46 m/s² cruise world-frame residual), aliasing spikes (1137 m/s² / 37 rad/s), instant-takeoff sensor storm (92% gyro saturation 10 s, fixed by ramped climb in commander.py), rest-tilt 4-6° (fixed by full-attitude odom init, mat_to_quat), truth-z EKF garbage (v1.15 height bug — report horizontal).
- Paper updated: §4.5 SITL table + §6 note in build_paper_{docx,pdf}.py; docs rebuilt.
- Autonomous loop: E:\gpsdnav\RUN_SITL_LOOP.ps1 → scripts/retrace/sitl_loop.py (params→fly→replay→score, gates abort with reason; score stage validated end-to-end). Container scripts: fly_square.sh, replay_all.sh [bag], fix_params.sh, preflight_check.sh, score_hv.py, score_aligned.py.
- Windows quoting lesson: cmd.exe strips single quotes and splits pipes/&/redirects even inside double quotes — loop sends only simple double-quoted commands; all pipe logic lives in container .sh files.
- Open threads: GCS mavlink instance (18570→14550) died silently once — all tooling MAVROS-native now; EKF GPS-drift re-fails preflight ~10 min after boot (reboot PX4, fly fast); verify_params.py patched to 10.0 (never -1).

## Done (committed in E:\kp_vio)

- Offline matrix C1–C6 x 10/30/60s x Flight A/B x Exp1/Exp2: `scripts/retrace/run_matrix.py` → `output/retrace/results.csv` (72 rows).
- Analytic optimal world F1–F6: `scripts/retrace/optimal_world.py` → `output/retrace/optimal_results.csv`.
- Bugs fixed: ATT degrees-vs-radians (57x), gyro quaternion frame order, circular yaw RMSE, tautological Exp2 scorer.
- Paper: `docs/GPS_LOSS_RETRACE_PAPER.docx/.pdf` + builders `scripts/retrace/build_paper_{docx,pdf}.py` + `gen_figures.py`. Has §2 logged-terms glossary. NO SITL column yet.
- Results: C4 thrust-drag best (A30s 119→8.8m, B30s C6 0.6m PASS); gyro+accel sufficient in perfect world (0.21m/801m); compass conditional (±); Exp2 return <2m always.

## SITL state (E:\gpsdnav, uncommitted)

- Image `mzahana/px4-simulation-cuda12.2.0-ubuntu22` (39GB, self-built; prebuilt pull 404). Container `gpsdnav` up, GPU ok, shared_volume ↔ `E:\gpsdnav\shared_volume`.
- PX4 v1.15.0-alpha1 (`navsat_callback` branch) built; gz Garden headless `taif_test4`; MAVROS; zenoh; `retrace_nav` pkg (retrace_node C1–C6 + commander) built.
- Fixes applied: `.gitattributes` LF for docker scripts; `retrace_node` uses ROS clock (MAVROS IMU stamps are 0 — timesync broken); commander sends offboard setpoints direct via MAVLink with live boot_ms.
- Airframe `4022_gz_x500_mono_cam_3d_lidar` += COM_ARM_WO_GPS=1, COM_RCL_EXCEPT=7, EKF2_REQ_GPS_H/EPH/EPV=50.

## Blockers (PX4 will not arm)

1. Preflight `GPS Horizontal/Vertical Pos Drift too high` + `position/velocity estimate error`; later `Yaw/height estimate error`. EKF GPS fusion flaky in this world.
2. v1.15 has NO `ARMING_CHECK` param (`unknown param` error). Do NOT use it.
3. NEVER set `COM_ARM_EKF_*=-1` (always-fails). Lenient highs only (10.0), or leave default.
4. TWO heartbeat sources on 14550: PX4 comp=1 autopilot=12 (truth) vs MAVROS mirror comp=191 autopilot=8 (stale, custom=0). Filter `autopilot==12`.
5. MAVROS→PX4 TX dead: fcu_url sent to 14557 (nothing listens). PX4 onboard listens 14580. Telemetry RX ok (14540). Timesync broken (RTT ~1.8e10 ms). MAVROS services unusable → use pymavlink direct on 14550 for commands.
6. OFFBOARD custom_mode=5 (NOT 6=ACRO/STAB). Commander now uses 5. OFFBOARD engages (327680 observed) but vehicle never took off (bag z≈0.1) — setpoints ignored while disarmed.
7. Duplicate `gz sim` / `px4` processes + zombies seen repeatedly. Always `pkill -9` both and verify single instance before start.
8. Gazebo publishes NO magnetometer topic (`gz_frame_id` SDF warnings) → compass missing → yaw preflight error. Baro/IMU/navsat publish ok.
9. NSH force-arm prepared but unused: `px4_start.sh` now redirects PX4 stdin from fifo `~/shared_volume/px4_stdin` (holder via `tail -f /dev/null`). Send `commander arm -f` with `echo ... > fifo`.

## Next session tasks

1. Clean boot: kill gz+px4, start ONE `gz sim -r -s taif_test4.sdf`, start PX4 via `px4_start.sh`, wait `Ready for takeoff` in `px4.log`.
2. `echo "commander arm -f" > ~/shared_volume/px4_stdin` (via `docker exec gpsdnav bash -c '...'`), verify PX4-only heartbeat armed.
3. Record bag `flight_sitl` (imu/mag/alt/odom/rc) + run commander square (`install/retrace_nav/bin/commander`). Verify bag path >50m, z≈5.
4. Replay bag per combo C1–C6 through `retrace_node` (`ros2 bag play` + node with `-p combo:=Cx`), record `/retrace/est_path` vs odom truth, score ATE/home-miss (reuse `metrics.py`; see `docs/.../ALGORITHM_ANALYSIS.md` path_error_calculator as alt).
5. Live C4 run during flight for Table I SITL reference (analytic 0.21m vs live SITL value).
6. Add SITL column to `build_paper_docx.py` + `build_paper_pdf.py` tables (Table II C-combos; Table I note live-C4 reference), rebuild docs, commit on `optimal-retracement`.

## Key commands (PowerShell, workdir E:\gpsdnav)

- `docker exec gpsdnav bash -c 'ps aux | grep -E "[g]z sim|[b]in/px4" | grep -v grep'`
- Start gz: `docker exec -d gpsdnav bash -c 'export GZ_SIM_RESOURCE_PATH=.../PX4-Autopilot/Tools/simulation/gz/models:.../worlds && cd .../PX4-Autopilot && gz sim -r -s Tools/simulation/gz/worlds/taif_test4.sdf > ~/shared_volume/gz.log 2>&1'`
- Start PX4: `docker exec -d gpsdnav bash -c '/home/user/shared_volume/px4_start.sh > /home/user/shared_volume/px4.log 2>&1'`
- PX4-only heartbeat check: python pymavlink script filtering `autopilot==12` (see `shared_volume/verify_params.py`, `ob_test.py`, `arm_test.py`, `hb_test.py`).
- MAVROS (telemetry only): fcu_url `udp://:14540@127.0.0.1:14580`, `ROS_DOMAIN_ID=18`, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- Venv (offline): `E:\kp_vio\kp_vio_py\.venv\Scripts\python.exe -X utf8 -u`.
