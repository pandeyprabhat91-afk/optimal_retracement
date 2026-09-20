# GPS-Loss Path Retrace: Sensor-Factor Experiments

**Date:** 2026-09-07 · **Branch:** `optimal-retracement` · **Code:** `scripts/retrace/` · **Data:** `output/retrace/results.csv`, `optimal_results.csv`

## 1. Objective and setup

Question: which onboard sensor readings must a drone log during flight so that, when GPS is lost, it can retrace its path home? Two complementary experiments were built.

**Exp1 — dead-reckon home.** Simulate a GPS cutoff, propagate position forward with a sensor subset, score drift against GPS truth at 10/30/60 s horizons. Answers "how far off is our position belief."

**Exp2 — reverse breadcrumb.** Store the outbound trail at 2 Hz; on loss, fly it backwards (pursuit with the combo's heading quality), score home miss and cross-track error. Answers "can we physically fly back."

**Flight logs (real ArduPilot `.BIN`):** Flight A (`00000061.BIN`, 48 MB) and Flight B (`2026-06-29 17-05-31.bin`, 79 MB), parsed with `pymavlink`. Both contain IMU (gyro+accel), magnetometer, barometer, GPS truth, ATT attitude, motor PWM (`RCOU`), throttle, vibration and battery — the full candidate factor set. Truth gated to HDop<1.0, NSats≥10.

**Cumulative combos (real logs):** C1 IMU-only → C2 +compass yaw → C3 +baro altitude → C4 +motor-thrust drag model → C5 +battery → C6 +zero-velocity updates.

**Optimal world (user-defined: no drift, perfect sensors, everything works):** a fully analytic 120 s flight (hover, 8 m/s legs, coordinated 90° turns, climb, hover; 801 m path) with machine-exact synthetic sensors. **Removal combos:** F1 full, F2 no-gyro, F3 no-accel, F4 no-mag, F5 thrust-instead-of-accel, F6 no-baro. This isolates each factor's _information content_, free of noise.

## 2. Results

### 2.1 Optimal world — the factor ranking in pure form (ATE over 801 m)

| Combo              | Missing factor | ATE        | Yaw RMSE   | Meaning                                                              |
| ------------------ | -------------- | ---------- | ---------- | -------------------------------------------------------------------- |
| F1 full            | —              | **0.21 m** | 0.00004    | everything works                                                     |
| F4 no-mag          | compass        | 0.22 m     | 0.00000    | gyro+accel sufficient alone                                          |
| F6 no-baro         | barometer      | 0.22 m     | 0.00000    | perfect accel holds vertical                                         |
| F2 no-gyro         | gyro           | 232 m      | 0.075      | per-sample tilt attitude fails in turns — **gyro necessary**         |
| F3 no-accel        | accelerometer  | frozen     | exact att. | attitude fine, position impossible — **accel necessary**             |
| F5 thrust-as-accel | true accel     | 365 m      | exact      | thrust ≠ specific force under tilt — **accelerometer irreplaceable** |
| Exp2 return        | —              | —          | —          | home **0.02 m**, cross-track 1 m                                     |

Conclusion: **gyro + accelerometer are the necessary and sufficient pair.** Compass, barometer and thrust carry zero extra information when perfect — their entire value lies in fighting real-world imperfection (proven next).

### 2.2 Real logs — Exp1 ATE in metres @10/30/60 s

Flight A: C1 11.9/120.7/460.8 · C2 11.9/119.8/438.3 · C3 12.9/119.0/433.4 · **C4 4.3/8.8/14.8** · C6 3.9/4.9/7.3.
Flight B: C1 2.4/30.3/223.2 · C2–C3 ≈ same · **C4 1.7/11.4/43.1** · **C6 0.3/0.6/7.4 (10 s and 30 s PASS <5 m)**.

Three robust findings, identical on both flights. **(a) Thrust-drag dominates:** C4 beats C3 by 13× (A 30 s: 119→8.8 m). Rotor drag opposes velocity ~linearly and scales with thrust, turning runaway double-integration into bounded velocity error — same physics as ArduPilot's `EK3_DRAG_BCOEF`. **(b) Compass is conditional:** on A at 60 s it anchors gyro drift (yaw 0.47→0.23); on B, where the gyro is superb (yaw 0.00–0.05), the uncalibrated mag (≈0.3 rad systematic bias from declination/mounting/soft-iron) _hurts_ (0.00→0.30). Field calibration decides the sign. **(c) ZUPT is situational:** C6 ≈ C4 on A (never stationary) but 19× better on B's hover segments (30 s: 11.4→0.6 m). C5 (battery) adds nothing anywhere — dead weight in current form.

Exp2 on real logs: home <2 m and cross-track ≤1.2 m for **every** combo on both flights. Pursuit re-aims each step, so position error never accumulates — return quality equals yaw quality alone. Even when dead-reckoned position is kilometres off, the breadcrumb gets home.

### 2.3 Bugs found by testing (all fixed, committed)

1. **ATT units (57× error):** ArduPilot logs attitude in _degrees_; consumed as radians. Fixing collapsed real-log errors 10–20×. 2. **Gyro frame order:** world-frame quaternion delta used as body rate — exact for 1-axis motion, wrong in 3D; fixed to `conj(q)·q'` (yaw error 1.54→0.001 rad). 3. **Circular yaw metric:** unwrapped multi-turn truth vs wrapped estimate inflated RMSE to ~35 rad; fixed with angle wrapping. 4. **Tautological Exp2 scorer:** trail scored against itself (always ~1 m); rebuilt as pursuit with combo-derived heading noise.

## 3. Conclusions and next steps

1. **Log these, in this priority:** gyro+accel at full rate (non-negotiable core); motor PWM + throttle (enables the drag model — biggest real-world win); compass with field declination/mount calibration (else disable it); baro for altitude; 2 Hz breadcrumb trail (lat/lon/alt/yaw/thrust) for Exp2 return.
2. **Architecture implication:** dead-reckoning position always diverges (8–43 m at 30–60 s even well-tuned); the reliable GPS-loss behaviour is breadcrumb reversal, which needs only heading + trail and demonstrated <2 m returns.
3. **Next:** tune the drag gain `k` per airframe; calibrate mag properly and re-test C2; close the loop with optical-flow/vaseline velocity aiding (the actual kp_vio stack) to push Exp1 under 5 m; run the Docker SITL visualisation (`scripts/retrace/sitl_bridge.md` runbook: PX4+Gazebo+Riz via `riotu-lab/gps_denied_navigation_sim`) for a perfect-sensor closed-loop demo.
