"""Autonomous SITL retrace loop: params -> fly -> replay C1-C4 -> score -> paper CSV.

Runs from Windows venv: E:\\kp_vio\\kp_vio_py\\.venv\\Scripts\\python.exe -X utf8 -u scripts/retrace/sitl_loop.py [--from {params,fly,replay,score}] [--bag flight_sitlN]

Each stage verifies its gate before proceeding; any failure aborts with reason.
Guards encoded from 2026-09-08 session: no COM_ARM_EKF -1, single gz/px4,
MAVROS fcu_url 14580, MAVROS-native commander, wall-clock replay, MAVROS dead
during replay, single /retrace/est_path publisher, topic-filtered play.

Windows quoting rules (cmd.exe strips single quotes, splits pipes even inside
double quotes): dx() wraps in double quotes and NEVER sends pipes or inner
double quotes. All multi-pipe container logic lives in shared_volume/*.sh.
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

SV = "/home/user/shared_volume"
ENV = (
    "source /opt/ros/humble/setup.bash && source "
    f"{SV}/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=18 && "
    "export RMW_IMPLEMENTATION=rmw_zenoh_cpp "
)
PYPATH = f"export PYTHONPATH={SV}/ros2_ws/src/retrace_nav:$PYTHONPATH"


def sh(cmd, timeout=120):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
    return (r.stdout or "") + (r.stderr or "")


def dx(cmd, timeout=120):
    return sh(f'docker exec gpsdnav bash -c "{cmd}"', timeout)


def dxd(cmd, timeout=30):
    return sh(f'docker exec -d gpsdnav bash -c "{cmd}"', timeout)


def gate(name, ok, detail=""):
    detail = str(detail).strip().splitlines()
    detail = detail[-1] if detail else ""
    print(f"[{'OK ' if ok else 'FAIL'}] {name} {detail}")
    if not ok:
        sys.exit(f"ABORT at {name}: {detail}")


def stage_params():
    out = dx(f"tail -n 60 {SV}/px4.log")
    bad = [l for l in out.splitlines() if "-1.0000" in l and "COM_ARM_EKF" in l]
    if bad:
        print("repairing -1 thresholds live")
        dx(f"{SV}/fix_params.sh", timeout=60)
    out = dx(f"{SV}/preflight_check.sh", timeout=60)
    gate("preflight", "Preflight check: OK" in out, out)


def stage_fly(bag):
    gate("fly", "FLY_DONE" in dx(f"exec {SV}/fly_square.sh {bag}", timeout=600))


def stage_replay(bag):
    # proven sequence in shared_volume/replay_all.sh (wall clock, -r 1,
    # topic-filtered play, MAVROS dead, single-publisher guard); C5/C6 omitted
    # (battery nil offline, square has no hover for ZUPT)
    dxd(f"exec {SV}/replay_all.sh {bag}", timeout=30)
    import re as _re

    for _ in range(60):
        time.sleep(30)
        procs = dx("pgrep -f bag ; echo POLL_END", timeout=30)
        running = [l for l in procs.splitlines() if _re.fullmatch(r"\d+", l.strip())]
        done = dx(f"cat {SV}/replay_all.log", timeout=30)
        if not running and "ALL_DONE" in done:
            break
    else:
        gate("replay", False, "replay_all.sh timeout")
    for _ in range(40):
        time.sleep(30)
        if "ALL_DONE" in dx(f"cat {SV}/replay_all.log"):
            break
    else:
        gate("replay", False, "replay_all.sh timeout")
    gate("replay", True, "ALL_DONE")


def stage_score(bag):
    import re

    outpath = str(
        Path(__file__).resolve().parents[2] / "output" / "retrace" / "sitl_results.csv"
    )
    lines = ["combo,ate3d_m,ate_h_m,ate_v_m,home_h_m,truth_len_m,note"]
    for c in ("C1", "C2", "C3", "C4"):
        out = dx(
            f"{ENV} && timeout 90 python3 -u {SV}/score_hv.py "
            f"{SV}/bags/{bag} {SV}/replay/est_{c} {c}"
        )
        print(out.strip())
        d = dict(re.findall(r"(ATE3d|ATEh|ATEv|home_h)=([\d.]+)", out))
        gate(f"score-{c}", len(d) == 4, out)
        lines.append(
            f"{c},{d['ATE3d']},{d['ATEh']},{d['ATEv']},{d['home_h']},114.5,sitl-loop"
        )
    with open(outpath, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("wrote sitl_results.csv")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--from",
        dest="frm",
        default="params",
        choices=["params", "fly", "replay", "score"],
    )
    ap.add_argument("--bag", default="flight_sitl_loop")
    a = ap.parse_args()
    order = ["params", "fly", "replay", "score"]
    for s in order[order.index(a.frm) :]:
        print(f"===== {s} =====")
        {
            "params": stage_params,
            "fly": lambda: stage_fly(a.bag),
            "replay": lambda: stage_replay(a.bag),
            "score": lambda: stage_score(a.bag),
        }[s]()
    print("LOOP COMPLETE")
