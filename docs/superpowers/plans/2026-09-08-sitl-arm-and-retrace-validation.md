# SITL Arm + Retrace Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Arm PX4 SITL in taif_test4, fly square via commander, replay bag through retrace C1–C6, score SITL results into paper.

**Architecture:** Fix poisoned arm thresholds live via NSH (no reboot), correct MAVROS TX port, reuse riotu record→replay→score workflow with retrace topics, wrap all in one autonomous loop script.

**Tech Stack:** PX4 v1.15.0-alpha1 navsat_callback, gz Garden taif_test4, MAVROS Humble, pymavlink direct, ROS 2 bags, Python venv E:\kp_vio\kp_vio_py\.venv.

## Global Constraints

- NEVER set COM_ARM_EKF_*=-1 (always-fails: estimatorCheck.cpp uses `test_ratio > param`, any ratio > -1 trips fail).
- NEVER use ARMING_CHECK param (absent in v1.15, unknown-param error).
- Filter heartbeats autopilot==12 (MAV_TYPE PX4); ignore comp=191 mirror.
- MAVROS→PX4 TX dead on stock fcu_url; commands go pymavlink direct on 14550.
- OFFBOARD custom_mode=5 (not 6).
- Single gz + single px4 instance; pkill zombies before start.
- Venv only: E:\kp_vio\kp_vio_py\.venv\Scripts\python.exe -X utf8 -u.

---

## Diagnosis evidence (2026-09-08, live container gpsdnav)

- `estimator_status`: hgt 0.004, vel 0.035, pos 0.008, mag 0.0; gps_check_fail 0; pre_flt_fail all False; mag_device SIM present, strength matches ref. EKF healthy.
- `param show`: COM_ARM_EKF_POS/VEL/HGT/YAW all -1.0 (poisoned by shared_volume/verify_params.py). Root cause of all four estimate-error preflight fails.
- gz topic list: imu + air_pressure + navsat publish; magnetometer topic absent in Garden, but PX4 mag device present via SITL sim path — no SDF fix needed.
- px4-rc.mavlink: offboard local 14580, remote 14540; GCS 18570. Stock dem.launch.py fcu_url `udp://:14540@127.0.0.1:14557` sends TX to dead port; must be `udp://:14540@127.0.0.1:14580`.
- Refs: riotu ALGORITHM_ANALYSIS.md §1d/§3 (record-once bag, replay per algorithm, path_error_calculator topics /target/gt_path vs estimate); TERCOM.md (tercom alias, taif_test4 origin 21.26505N 40.35415E); SIMULATION_ENVIRONMENT.md (taif_test4 heightmap pos 71.26,-70.86 size 2708x2692x285.9); bash.sh aliases (mono_taif4, tercom, zenoh, ROS_DOMAIN_ID=18, zenoh RMW); PX4 preflight docs (COM_ARM_WO_GPS=1 allows arm with GPS warnings; COM_ARM_EKF_* thresholds); estimatorCheck.cpp:205-273 (`>` compare); issues #23852 (v1.15 height wobble, non-blocker here), #24882 (arm-denied checklist).

### Task 1: Unpoison arm thresholds live + persist

**Files:**

- Modify: `E:\gpsdnav\shared_volume\verify_params.py` (replace -1.0 with 10.0, drop BAT_MIN -1)
- Modify: PX4 live params via fifo `~/shared_volume/px4_stdin` + `param save`

**Interfaces:**

- Consumes: live fifo at /home/user/shared_volume/px4_stdin
- Produces: COM_ARM_EKF_POS/VEL/HGT/YAW=10.0, arming possible

- [ ] **Step 1: Set lenient thresholds live (no reboot)**

```bash
docker exec gpsdnav bash -c 'for p in COM_ARM_EKF_POS COM_ARM_EKF_VEL COM_ARM_EKF_HGT COM_ARM_EKF_YAW; do echo "param set $p 10.0" > /home/user/shared_volume/px4_stdin; sleep 1; done; echo "param save" > /home/user/shared_volume/px4_stdin'
```

- [ ] **Step 2: Verify values + preflight clears**

```bash
docker exec gpsdnav bash -c 'for p in COM_ARM_EKF_POS COM_ARM_EKF_VEL COM_ARM_EKF_HGT COM_ARM_EKF_YAW; do echo "param show $p" > /home/user/shared_volume/px4_stdin; sleep 1; done; sleep 2; tail -n 15 /home/user/shared_volume/px4.log'
```

Expected: values 10.0; no new `Preflight Fail` lines after set.

- [ ] **Step 3: Patch verify_params.py so reruns never repoison**

```python
# old tuple entries (b"COM_ARM_EKF_POS", -1.0, ...), (b"COM_ARM_EKF_VEL", -1.0, ...),
# (b"COM_ARM_EKF_HGT", -1.0, ...), (b"COM_ARM_EKF_YAW", -1.0, ...), (b"COM_ARM_BAT_MIN", -1.0, ...)
# new: 10.0 for all four EKF, drop BAT_MIN line entirely
for name, val, pt in (
    (b"COM_ARM_EKF_POS", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_EKF_VEL", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_EKF_HGT", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_EKF_YAW", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_IMU_ACC", 99.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_IMU_GYR", 99.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
):
```

- [ ] **Step 4: Confirm no other -1 setters remain**

```bash
grep -rn "COM_ARM_EKF" E:\gpsdnav\shared_volume\*.py E:\gpsdnav\shared_volume\ros2_ws\src\retrace_nav\ -r
```

Expected: only 10.0 values.

### Task 2: Force-arm smoke test (proves checks green)

**Files:**

- Use: `E:\gpsdnav\shared_volume\arm_test.py` (reads COM_ARM_WO_GPS then force-arms 21196), heartbeat filter autopilot==12

**Interfaces:**

- Consumes: pymavlink udp:127.0.0.1:14550
- Produces: armed=True on PX4 heartbeat

- [ ] **Step 1: Run force-arm**

```bash
E:\kp_vio\kp_vio_py\.venv\Scripts\python.exe -X utf8 -u E:\gpsdnav\shared_volume\arm_test.py
```

Expected: `COMMAND_ACK result=0`, armed heartbeat. If denied, read px4.log STATUSTEXT reason, do not retry blindly.

- [ ] **Step 2: Disarm after test**

```bash
docker exec gpsdnav bash -c 'echo "commander disarm" > /home/user/shared_volume/px4_stdin'
```

### Task 3: Fix MAVROS TX port (telemetry Michealides, services usable)

**Files:**

- Modify: launch invocation fcu_url to `udp://:14540@127.0.0.1:14580`
- Note: stock `mavros.launch.py` default + `dem.launch.py` hardcode 14557 (dead port per px4-rc.mavlink)

**Interfaces:**

- Consumes: PX4 offboard listener 14580
- Produces: MAVROS TX works, mode/arm services usable

- [ ] **Step 1: Start MAVROS with corrected fcu_url (telemetry only for now)**

```bash
docker exec -d gpsdnav bash -c 'source /opt/ros/humble/setup.bash && source ~/shared_volume/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=18 && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && ros2 launch gps_denied_navigation_sim mavros.launch.py mavros_namespace:=target/mavros tgt_system:=1 fcu_url:=udp://:14540@127.0.0.1:14580 base_link_frame:=target/base_link odom_frame:=target/odom map_frame:=map > ~/shared_volume/mavros.log 2>&1'
```

- [ ] **Step 2: Verify RX topics flow**

```bash
docker exec gpsdnav bash -c 'source /opt/ros/humble/setup.bash && timeout 15 ros2 topic echo /target/mavros/imu/data --once 2>&1 | head -n 15; timeout 15 ros2 topic echo /target/mavros/local_position/odom --once 2>&1 | head -n 15'
```

Expected: both print one message. Commander still uses pymavlink direct regardless.

### Task 4: Fly square + record bag (riotu §1d pattern)

**Files:**

- Use: `install/retrace_nav/bin/commander` (streams setpoints, OFFBOARD mode 5, arm x3, square 20m @5m, LAND)
- Bag: `~/shared_volume/bags/flight_sitl2/` (fresh; keep flight_sitl/flight1 untouched)

**Interfaces:**

- Consumes: armed PX4, MAVROS odom/imu topics
- Produces: bag with imu/mag/alt/odom/rc + gt_path, path length >50m, z~=5

- [ ] **Step 1: Start zenoh + bag record before flight**

```bash
docker exec -d gpsdnav bash -c 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=18 && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && ros2 run rmw_zenoh_cpp rmw_zenohd > ~/shared_volume/zenoh.log 2>&1'
docker exec -d gpsdnav bash -c 'source /opt/ros/humble/setup.bash && source ~/shared_volume/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=18 && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && ros2 bag record -o ~/shared_volume/bags/flight_sitl2 /tf /tf_static /clock /target/mavros/imu/data /target/mavros/imu/mag /target/mavros/altitude /target/mavros/local_position/odom /target/mavros/rc/out /target/gt_path /retrace/est_path > ~/shared_volume/bag.log 2>&1'
```

Bag topics mirror riotu §1d plus /retrace/est_path for live-C4 capture.

- [ ] **Step 2: Run commander square**

```bash
docker exec gpsdnav bash -c 'source /opt/ros/humble/setup.bash && source ~/shared_volume/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=18 && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && timeout 300 /home/user/shared_volume/ros2_ws/install/retrace_nav/bin/commander 2>&1 | tee ~/shared_volume/commander.log'
```

Expected: OFFBOARD ack=0, ARM ack=0, "armed, takeoff", pattern, LAND. If arm denied, stop: read commander.log + px4.log, fix params, rerun Task 1.

- [ ] **Step 3: Verify flight (path >50m, z~=5)**

```bash
E:\kp_vio\kp_vio_py\.venv\Scripts\python.exe -X utf8 -u E:\kp_vio\scripts\retrace\check_bag.py E:\gpsdnav\shared_volume\bags\flight_sitl2
```

(check_bag may not exist; fallback: ros2 bag info + quick odom range check. Write minimal checker inline if missing — do not block loop on it.)

### Task 5: Replay bag per combo C1–C6 + score (riotu §3 pattern)

**Files:**

- Use: `install/retrace_nav/bin/retrace_node` with `-p combo:=Cx`, `ros2 bag play --clock`
- Score: `E:\kp_vio\scripts\retrace\metrics.py` compute_ate + home_miss on /retrace/est_path vs odom truth
- Alt: riotu `path_error_calculator` est_path_topic:=/retrace/est_path

**Interfaces:**

- Consumes: bag flight_sitl2
- Produces: output/retrace/sitl_results.csv rows C1-C6 (ate, home_miss, path_len)

- [ ] **Step 1: Replay once per combo, capture est_path**

```bash
docker exec gpsdnav bash -c 'source /opt/ros/humble/setup.bash && source ~/shared_volume/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=18 && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && ros2 bag play --clock ~/shared_volume/bags/flight_sitl2 & ros2 run retrace_nav retrace_node --ros-args -p combo:=C4 > ~/shared_volume/retrace.log 2>&1'
```

Repeat Cx=C1..C6. Record /retrace/est_path per run (bag or topic echo to CSV).

- [ ] **Step 2: Score offline in venv**

```python
from metrics import compute_ate, home_miss
ate = compute_ate(est_xyz, truth_xyz)
hm = home_miss(est_end, home)
```

Expected: CSV with 6 rows; C4 thrust-drag best (offline analog A30s 119→8.8m).

### Task 6: Live C4 reference + paper SITL column

**Files:**

- Modify: `E:\kp_vio\scripts\retrace\build_paper_docx.py`, `build_paper_pdf.py` (Table II C-combos SITL column; Table I live-C4 note)
- Rebuild: docs/GPS_LOSS_RETRACE_PAPER.docx/.pdf, commit on optimal-retracement

**Interfaces:**

- Consumes: output/retrace/sitl_results.csv
- Produces: paper with SITL column, committed

- [ ] **Step 1: Run live C4 during flight (retrace_node combo C4 alongside commander)**

Bag already captures /retrace/est_path when node runs pre-flight. If missing, repeat Task 4 with node started first.

- [ ] **Step 2: Add SITL column, rebuild, commit**

```bash
E:\kp_vio\kp_vio_py\.venv\Scripts\python.exe -X utf8 -u E:\kp_vio\scripts\retrace\build_paper_docx.py
E:\kp_vio\kp_vio_py\.venv\Scripts\python.exe -X utf8 -u E:\kp_vio\scripts\retrace\build_paper_pdf.py
git -C E:\kp_vio add docs/GPS_LOSS_RETRACE_PAPER.docx docs/GPS_LOSS_RETRACE_PAPER.pdf output/retrace/sitl_results.csv
git -C E:\kp_vio commit -m "feat(retrace): SITL validation column C1-C6 + live C4 reference"
```

### Task 7: Autonomous loop script

**Files:**

- Create: `E:\kp_vio\scripts\retrace\sitl_loop.py` (clean boot → params → arm → record → fly → replay C1-C6 → score → CSV; idempotent, logs each stage, aborts with reason on arm/flight fail)
- Loop driver: `E:\gpsdnav\RUN_SITL_LOOP.ps1` (calls venv python + docker, single entry)

**Interfaces:**

- Consumes: all tasks above
- Produces: unattended end-to-end run, resumable per stage flag --from {params,arm,fly,replay,score}

- [ ] **Step 1: Write sitl_loop.py stages with checks (single gz/px4 verify, -1 guard, heartbeat autopilot==12 filter, path>50m gate)**

- [ ] **Step 2: Dry-run --check-only, then full loop, attach output/retrace/sitl_results.csv**
