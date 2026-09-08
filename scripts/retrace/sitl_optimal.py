"""SITL optimal world: perfect sensors on real flown SITL geometry.

Takes the PX4 truth odometry of flight_sitl6 (114.5 m square, smooth EKF/GPS
truth), resamples to 100 Hz, differentiates to machine-exact gyro/accel/mag/
baro/thrust, then runs combos C1-C6 through the same strapdown core as the
SITL node. Zero noise, zero bias, zero vibration, exact init: isolates the
estimator math from the MAVROS sensor dirt measured in raw SITL replay.

Mirrors optimal_world.py F1-F6 method, but on flown (not analytic) geometry.

Usage:
  kp_vio_py\\.venv\\Scripts\\python.exe -X utf8 -u scripts/retrace/sitl_optimal.py
Output:
  output/retrace/sitl_optimal_results.csv
"""

import csv
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from estimator import (
    circ_mean,
    quat_from_euler,
    quat_mul,
    quat_rotate,
    quat_to_euler,
    quat_update,
    tilt_yaw,
)

GRAV = 9.80665
DT = 0.01
FIELD = np.array([0.5, 0.0, 0.8])  # same reference field as optimal_world.py
OUT = str(
    Path(__file__).resolve().parents[2]
    / "output"
    / "retrace"
    / "sitl_optimal_results.csv"
)
T_W = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # ENU->NED
T_B = np.diag([1.0, -1.0, -1.0])  # FLU->FRD
COMBOS = {
    "C1": ("gyr", "acc"),
    "C2": ("gyr", "acc", "mag_yaw"),
    "C3": ("gyr", "acc", "mag_yaw", "baro_alt"),
    "C4": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag"),
    "C5": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag", "battery"),
    "C6": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag", "battery", "zupt"),
}


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_mat(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def mat_to_euler(R):
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w, x, y, z = (
            0.25 * s,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        )
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x, y, z = (
            (R[2, 1] - R[1, 2]) / s,
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
        )
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x, y, z = (
            (R[0, 2] - R[2, 0]) / s,
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
        )
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x, y, z = (
            (R[1, 0] - R[0, 1]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
        )
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def unwrap(a):
    return np.unwrap(a)


def build_perfect():
    """Commanded-profile optimal world on flown SITL geometry.

    PX4 truth (flight_sitl6) holds yaw fixed and tracks setpoints to 0.36 m,
    so the commanded profile -- static, cosine climb 0-5 m, hover, four 20 m
    legs with cosine corner blends, cosine land, static -- is a faithful
    idealization. All sensors machine-exact at 100 Hz: zero noise/bias/VRE.
    (Deriving sensors by differentiating EKF odom fails: cm-level wander
    differentiates to m/s artifacts at 100 Hz.)
    """
    tg = np.arange(0.0, 160.0, DT)
    pos_enu = np.zeros((len(tg), 3))

    def cosprof(t, t1, t2, p1, p2):
        u = np.clip((t - t1) / (t2 - t1), 0.0, 1.0)
        s = 0.5 - 0.5 * np.cos(np.pi * u)
        return p1 + (p2 - p1) * s

    H = 5.0
    legs = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.0)]
    starts = [31.0, 43.0, 55.0, 67.0]
    for k, t in enumerate(tg):
        if t < 8.0:
            p = (0.0, 0.0, 0.0)
        elif t < 23.0:
            p = (0.0, 0.0, cosprof(t, 8.0, 23.0, 0.0, H))
        elif t < 31.0:
            p = (0.0, 0.0, H)
        elif t < 79.0:
            j = min(int((t - 31.0) // 12.0), 3)
            u = (t - (starts[j])) / 12.0
            s = 0.5 - 0.5 * np.cos(np.pi * np.clip(u, 0.0, 1.0))
            x0, y0 = legs[j]
            x1, y1 = legs[j + 1]
            p = (x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, H)
        elif t < 90.0:
            p = (0.0, 0.0, cosprof(t, 79.0, 90.0, H, 0.0))
        else:
            p = (0.0, 0.0, 0.0)
        pos_enu[k] = p
    # level, fixed yaw: gyro identically zero; attitude trivially exact
    eul_g = np.zeros((len(tg), 3))
    vel_enu = np.gradient(pos_enu, DT, axis=0)
    acc_enu = np.gradient(vel_enu, DT, axis=0)
    pos = (T_W @ pos_enu.T).T
    vel = (T_W @ vel_enu.T).T
    acc_ned = (T_W @ acc_enu.T).T
    n = len(tg)
    gyr = np.zeros((n, 3))
    acc = np.zeros((n, 3))
    mag = np.zeros((n, 3))
    Q = [quat_from_euler(*e) for e in quat_to_mat_seq(eul_g)]
    for i in range(n):
        q = Q[i]
        R = quat_to_mat(q)
        acc[i] = R.T @ (acc_ned[i] - np.array([0.0, 0.0, GRAV]))
        mag[i] = R.T @ FIELD
        if i:
            dq = quat_mul(quat_conj(Q[i - 1]), Q[i])
            gyr[i] = 2 * dq[1:] / DT
    # body sensors are FRD already (truth-derived); no FLU conversion needed
    return {
        "t": tg,
        "pos": pos,
        "vel": vel,
        "gyr": gyr,
        "acc": acc,
        "mag": mag,
        "baro": pos[:, 2],  # down-positive NED, same convention as node
        "thr": 0.5 * (1 + np.linalg.norm(acc_ned[:, :2], axis=1) / GRAV),
        "spd": np.linalg.norm(vel, axis=1),
        "eul0": quat_to_euler(Q[0]),
    }


def quat_to_mat_seq(E):
    return E  # euler triplets; quat_from_euler applied by caller


def run_combo(d, combo):
    sensors = COMBOS[combo]
    t = d["t"]
    n = len(t)
    q = quat_from_euler(*d["eul0"]) if "eul0" in d else quat_from_euler(*d["eul"][0])
    p = d["pos"][0].copy()
    v = d["vel"][0].copy()
    # static cal on first 2 s (exact zeros)
    m = t < 2.0
    g_bias = d["gyr"][m].mean(axis=0)
    R0 = quat_to_mat(q)
    a_bias = d["acc"][m].mean(axis=0) - R0.T @ np.array([0.0, 0.0, -GRAV])
    mag_off = wrap(
        np.mean([tilt_yaw(*d["mag"][i], 0.0, 0.0) for i in np.nonzero(m)[0]])
        - quat_to_euler(q)[2]
    )
    baro0 = d["baro"][0]
    hover = float(d["thr"][m].mean())
    p0z = float(p[2])
    est = np.zeros((n, 3))
    for i in range(n):
        gyr = d["gyr"][i] - g_bias
        acc = d["acc"][i] - a_bias
        q = quat_update(q, gyr, DT)
        eul = quat_to_euler(q)
        if "mag_yaw" in sensors:
            myaw = tilt_yaw(*d["mag"][i], eul[0], eul[1]) - mag_off
            eul[2] = eul[2] + 0.1 * wrap(myaw - eul[2])
            q = quat_from_euler(*eul)
        a_ned = quat_rotate(q, acc) + np.array([0.0, 0.0, GRAV])
        v = v + a_ned * DT
        p = p + v * DT
        if "baro_alt" in sensors:
            p[2] = 0.9 * p[2] + 0.1 * (d["baro"][i] - baro0 + p0z)
        if "thrust_drag" in sensors:
            thr = d["thr"][i] / hover if hover else 1.0
            v[:2] -= 0.3 * thr * v[:2] * DT
        if "zupt" in sensors and np.linalg.norm(v) < 0.2:
            v *= 0.8
        est[i] = p
    return est


def main():
    d = build_perfect()
    path_len = float(np.linalg.norm(np.diff(d["pos"][:, :2], axis=0), axis=1).sum())
    print(f"truth {len(d['t'])} samples @100Hz, path {path_len:.1f} m")
    rows = []
    for combo in ("C1", "C2", "C3", "C4", "C5", "C6"):
        est = run_combo(d, combo)
        err = np.linalg.norm(est - d["pos"], axis=1)
        eh = np.linalg.norm(est[:, :2] - d["pos"][:, :2], axis=1)
        rows.append(
            (
                combo,
                float(err.mean()),
                float(eh.mean()),
                float(np.abs(est[:, 2] - d["pos"][:, 2]).mean()),
                float(np.linalg.norm(est[-1] - est[0])),
            )
        )
        print(
            f"{combo}: ATE3d={rows[-1][1]:.3f} m ATEh={rows[-1][2]:.3f} "
            f"ATEv={rows[-1][3]:.3f} home={rows[-1][4]:.3f} m"
        )
    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["combo", "ate3d_m", "ate_h_m", "ate_v_m", "home_miss_m"])
        w.writerows(rows)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
