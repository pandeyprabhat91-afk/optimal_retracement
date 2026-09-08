# Optimal Retracement — GPS-Loss Path Retrace (SITL + Offline)

Which sensors bring a drone home when GPS dies? This repo answers with three
experiments sharing one estimator core: real-log replay, a perfect analytic
world, and a live PX4 SITL square with raw and perfect sensors.

Paper: `docs/GPS_LOSS_RETRACE_PAPER.docx` / `.pdf` (built from
`scripts/retrace/build_paper_*.py`). headline results in §5.

## Repo map (this branch)

- `scripts/retrace/` — everything retrace: `run_matrix.py` (72-row offline
  matrix → `output/retrace/results.csv`), `optimal_world.py` (F1-F6 →
  `optimal_results.csv`), `sitl_optimal.py` (SITL perfect-sensor control →
  `sitl_optimal_results.csv`), `sitl_loop.py` (autonomous loop driver),
  `build_paper_docx.py` / `build_paper_pdf.py` (paper builders),
  `replay_retrace.py`, `estimator.py`, `metrics.py`, `combos.yaml`.
- `sitl/` — SITL package + container scripts + data for this branch:
  `retrace_nav/` (ROS 2 estimator node + flight commander),
  `container/` (bag/score/replay helpers + PX4 helpers),
  `bags/flight_sitl6/` (recorded square flight, 33 MB),
  `sitl6_*.csv` (dumped imu/odom/mag/baro/rc for audit),
  `RUN_SITL_LOOP.ps1` (one-line loop entry), `HANDOVER.md` (SITL state notes).
- `datasets/flight_logs/` — ArduPilot logs (force-added, gitignored by default):
  `00000061.BIN` (Flight A, 46 MB), `2026-06-29 17-05-31.bin` (Flight B, 76 MB).
- `output/retrace/` — result CSVs + `figs/` (force-added; builders read figs).

## Part 1 — offline + optimal (Windows only, ~10 min)

Prereqs: Python 3.10+, repo venv with numpy, scipy, matplotlib,
python-docx, reportlab (`pip install` line in root README):

```
.\.venv\Scripts\python.exe -X utf8 -u scripts/retrace/run_matrix.py
.\.venv\Scripts\python.exe -X utf8 -u scripts/retrace/optimal_world.py
.\.venv\Scripts\python.exe -X utf8 -u scripts/retrace/sitl_optimal.py
```

Inputs: `datasets/flight_logs/*.BIN` (paths at top of `run_matrix.py` /
`replay_retrace.py`; edit if your checkout lives elsewhere).
Outputs: `output/retrace/results.csv` (72 rows: C1-C6 x 10/30/60 s x
Flight A/B x Exp1/Exp2), `optimal_results.csv` (F1-F6),
`sitl_optimal_results.csv` (C1-C6 on perfect SITL geometry).

Rebuild the paper (needs `output/retrace/figs/*.png`):

```
.\.venv\Scripts\python.exe -X utf8 -u scripts/retrace/build_paper_docx.py
.\.venv\Scripts\python.exe -X utf8 -u scripts/retrace/build_paper_pdf.py
```

Expected key numbers: F1 0.21 m / 801 m; A C4 30 s 8.8 m (from 119);
B C6 30 s 0.6 m; SITL-optimal C1 0.006 m / 80 m.

## Part 2 — live SITL (Windows + Docker, ~1 h first time)

Prereqs: Docker with WSL2 backend, NVIDIA GPU optional (CPU build exists),
~45 GB free (39 GB image + PX4 source + bags), git + SSH key on GitHub.

1. Build the image (prebuilt pull 404s, build it):
   `gps_denied_navigation_docker/docker/make px4-simulation-cuda12.2.0-ubuntu22`
   (repo: `riotu-lab/gps_denied_navigation_docker`).
2. Run container `gpsdnav` (`./docker_run_wsl.sh` on Windows). User `user`,
   shared vol `~/shared_volume` ↔ a host folder you mount (ours was
   `E:\gpsdnav\shared_volume`).
3. Bootstrap workspace inside container (clones
   `riotu-lab/gps_denied_navigation_sim`, builds PX4 `navsat_callback`
   branch, MAVROS, tercom): see `sitl/container/` notes + upstream README.
4. Copy this repo's `sitl/retrace_nav` to
   `~/shared_volume/ros2_ws/src/retrace_nav` and `sitl/container/*.sh`,
   `*.py` to `~/shared_volume/`.
5. Start one `gz sim taif_test4`, one PX4 (`px4_start.sh`), MAVROS with
   `fcu_url:=udp://:14540@127.0.0.1:14580` (stock 14557 is a dead port),
   zenoh daemon, then run the loop from Windows:
   `.\sitl\RUN_SITL_LOOP.ps1` (params → fly → replay C1-C4 → score).

Critical gotchas (each cost us hours; all encoded in the loop):
COM_ARM_EKF_* must never be -1 (always-fails); single gz + single px4
(kill zombies first); commands via MAVROS services, never raw pymavlink
(GCS link is RX-only and died silently once); OFFBOARD is custom_mode 5;
replay on wall clock at -r 1 with MAVROS dead, topic-filtered play, one
`/retrace/est_path` publisher; score with bag-timestamp interpolation.

## What the numbers mean

- Offline §5.2, optimal §5.1: factor ranking + information content.
- Raw SITL §5.5 (`output/retrace/sitl_results.csv`): same ranking at ~50x
  scale from measured sensor dirt (VRE +0.46 m/s2, spikes to
  1137 m/s2 / 37 rad/s at 49 Hz). Quote the ranking, never the magnitudes.
- SITLoptimal §5.6 (`sitl_optimal_results.csv`): 6 mm on identical geometry.
  Raw-vs-perfect gap is 100% sensor dirt.

Legacy scripts in `sitl/container/` kept for provenance: `arm_test.py`,
`hb_test.py`, `ob_test.py` (host-side pymavlink, obsolete since MAVROS TX
fix), `debug_retrace.py` (offline estimator audit harness).
