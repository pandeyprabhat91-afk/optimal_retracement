# Optimal Retracement — Which Sensors Bring a Drone Home?

GPS-loss path retrace for multicopters: six cumulative sensor sets (C1: gyro
plus accelerometer, up to C6: adding compass, barometer, thrust, battery and
ZUPT) tested three ways — real ArduPilot log replay, a perfect analytic world,
and a live PX4 SITL square with raw and with perfect sensors.

Paper: `docs/GPS_LOSS_RETRACE_PAPER.docx` / `.pdf` (built from
`scripts/retrace/build_paper_*.py`). Results: gyro+accel sufficient when exact
(0.21 m / 801 m, 6 mm / 80 m SITL geometry); thrust-drag dominates real logs
(30 s error 119 to 8.8 m); SITL repeats the ranking at ~50x scale from measured
sensor dirt (VRE +0.46 m/s2, aliasing spikes, EKF wobble).

## Layout

- `scripts/retrace/` — all experiment code: `run_matrix.py` (72-row offline
  matrix), `optimal_world.py` (F1-F6), `sitl_optimal.py` (perfect-sensor SITL
  control), `sitl_loop.py` (autonomous loop driver), `build_paper_*.py`,
  `replay_retrace.py`, `estimator.py`, `metrics.py`, `combos.yaml`, tests.
- `sitl/` — live validation: `retrace_nav/` ROS 2 package (estimator node +
  flight commander), `container/` helpers, `bags/flight_sitl6/` recorded
  flight, `sitl6_*.csv` audit data, `RUN_SITL_LOOP.ps1`, full guide in
  `sitl/README.md`.
- `datasets/flight_logs/` — the two ArduPilot logs (122 MB total).
- `output/retrace/` — result CSVs + paper figs (regenerable, kept for convenience).
- `docs/` — paper + SITL fix plan.

## Quickstart (Windows, ~10 min, no Docker)

1. Python 3.10+, create venv in repo root:
   `py -3.11 -m venv .venv && .\.venv\Scripts\activate`
2. `pip install numpy scipy matplotlib python-docx reportlab pymavlink pyyaml`
3. Run, from repo root:
   ```
   .\.venv\Scripts\python.exe -X utf8 -u scripts\retrace\run_matrix.py
   .\.venv\Scripts\python.exe -X utf8 -u scripts\retrace\optimal_world.py
   .\.venv\Scripts\python.exe -X utf8 -u scripts\retrace\sitl_optimal.py
   ```
   Expected: F1 0.21 m; A C4 30 s 8.8 m; SITL-optimal C1 0.006 m.
4. Live SITL (Docker + PX4/Gazebo): follow `sitl/README.md` Part 2, then
   `.\sitl\RUN_SITL_LOOP.ps1`.
