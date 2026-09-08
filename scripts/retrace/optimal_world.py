"""Optimal world, fully analytic: zero noise, exact sensors by construction.

Profile (120 s @ 100 Hz, speeds like the real logs):
  0-10s hover, 10-40s straight 8 m/s N, 40-50s coordinated 90-deg turn,
  50-80s straight E, 80-90s 90-deg turn back, 90-110s straight N + climb,
  110-120s hover.
Removal combos F1-F6 test which factors are necessary/sufficient.
"""

import csv
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from estimator import (
    circ_mean,
    quat_conj,
    quat_from_euler,
    quat_mul,
    quat_rotate,
    quat_to_euler,
    quat_update,
    tilt_yaw,
)
from metrics import compute_ate, home_miss

OUT = str(
    Path(__file__).resolve().parents[2] / "output" / "retrace" / "optimal_results.csv"
)
FIELD = np.array([0.5, 0.0, 0.8])
DT = 0.01
T_END = 120.0


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def build_world():
    t = np.arange(0.0, T_END, DT)
    yaw = np.zeros_like(t)
    yaw[(t >= 40) & (t < 50)] = -0.5 * math.pi * (t[(t >= 40) & (t < 50)] - 40) / 10.0
    yaw[t >= 50] = -0.5 * math.pi
    m = (t >= 80) & (t < 90)
    yaw[m] = -0.5 * math.pi + 0.5 * math.pi * (t[m] - 80) / 10.0
    yaw[t >= 90] = 0.0
    spd = np.full_like(t, 8.0)
    spd[t < 10] = 0.0
    spd[t >= 110] = 0.0
    climb = np.zeros_like(t)
    climb[(t >= 90) & (t < 110)] = 1.0
    yaw_rate = np.gradient(yaw) / DT
    bank = np.arctan(spd * yaw_rate / 9.80665)
    eul = np.array([bank, np.zeros_like(t), yaw]).T
    vel = np.array([spd * np.cos(yaw), spd * np.sin(yaw), -climb]).T
    pos = np.cumsum(vel, axis=0) * DT
    acc_ned = np.gradient(vel, axis=0) / DT
    Q = [quat_from_euler(*e) for e in eul]
    gyr = np.zeros_like(pos)
    acc = np.zeros_like(pos)
    mag = np.zeros_like(pos)
    for i in range(len(t)):
        q = Q[i]
        R = np.array(
            [
                [
                    q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2,
                    2 * (q[1] * q[2] - q[0] * q[3]),
                    2 * (q[1] * q[3] + q[0] * q[2]),
                ],
                [
                    2 * (q[1] * q[2] + q[0] * q[3]),
                    q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2,
                    2 * (q[2] * q[3] - q[0] * q[1]),
                ],
                [
                    2 * (q[1] * q[3] - q[0] * q[2]),
                    2 * (q[2] * q[3] + q[0] * q[1]),
                    q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2,
                ],
            ]
        )
        acc[i] = R.T @ (acc_ned[i] - np.array([0.0, 0.0, 9.80665]))
        mag[i] = R.T @ FIELD
        if i:
            dq = quat_mul(quat_conj(Q[i - 1]), Q[i])
            gyr[i] = 2 * dq[1:] / DT
    return {
        "t": t,
        "pos": pos,
        "vel": vel,
        "eul": eul,
        "gyr": gyr,
        "acc": acc,
        "mag": mag,
        "baro": -pos[:, 2],
        "thr": 0.5 * (1 + np.linalg.norm(acc_ned[:, :2], axis=1) / 9.80665),
        "hover": 0.5,
    }


def run_combo(d, combo):
    t, pos = d["t"], d["pos"]
    i0 = int(np.searchsorted(t, t[0] + 10.0))
    q = quat_from_euler(*d["eul"][i0])
    p = pos[i0].copy()
    v = d["vel"][i0].copy()
    off = 0.0
    if combo in ("F1", "F2"):
        my = np.array(
            [
                tilt_yaw(*d["mag"][j], *d["eul"][j][:2])
                for j in range(max(0, i0 - 100), i0)
            ]
        )
        off = wrap(circ_mean(wrap(my - d["eul"][max(0, i0 - 100) : i0, 2])))
    est = [p.copy()]
    yaws = []
    for i in range(i0 + 1, len(t)):
        if combo == "F2":
            a = d["acc"][i]
            r = math.asin(max(-1.0, min(1.0, -a[1] / 9.80665)))
            pt = math.asin(max(-1.0, min(1.0, a[0] / 9.80665)))
            yw = tilt_yaw(*d["mag"][i], r, pt) - off
            q = quat_from_euler(r, pt, yw)
            eul = np.array([r, pt, yw])
        else:
            q = quat_update(q, d["gyr"][i], DT)
            eul = quat_to_euler(q)
            if combo == "F1":
                yw = tilt_yaw(*d["mag"][i], eul[0], eul[1]) - off
                eul[2] = eul[2] + 0.1 * wrap(yw - eul[2])
                q = quat_from_euler(*eul)
        if combo == "F3":
            pass
        elif combo == "F5":
            proxy = np.array([0.0, 0.0, -(d["thr"][i] / d["hover"]) * 9.80665])
            v = v + (quat_rotate(q, proxy) + np.array([0.0, 0.0, 9.80665])) * DT
            p = p + v * DT
        else:
            v = v + (quat_rotate(q, d["acc"][i]) + np.array([0.0, 0.0, 9.80665])) * DT
            p = p + v * DT
            if combo == "F1":
                p[2] = 0.9 * p[2] + 0.1 * pos[i][2]
        est.append(p.copy())
        yaws.append(eul[2])
    est = np.array(est)
    tru = pos[i0:]
    ate = compute_ate(est, tru)
    hm = home_miss(est[-1], tru[0])
    if combo == "F3":
        hm = home_miss(est[-1], tru[-1])
    yw = np.array(yaws)
    yrmse = float(
        np.sqrt(np.mean(wrap(np.unwrap(yw) - np.unwrap(d["eul"][i0 + 1 :, 2])) ** 2))
    )
    step = max(1, len(tru) // int(2 * (t[-1] - t[i0])))
    out = tru[::step]
    rng = np.random.default_rng(42)
    sig = max(yrmse / math.sqrt(max(1, len(yw))), 0.02)
    ret = [out[-1].copy()]
    ti, guard = len(out) - 2, 0
    while ti >= 0 and guard < 4 * len(out):
        guard += 1
        vec = out[ti] - ret[-1]
        dist = float(np.linalg.norm(vec))
        if dist < 2.0:
            ti -= 1
            continue
        h = math.atan2(vec[1], vec[0]) + rng.normal(0, sig)
        sl = min(2.5, dist)
        ret.append(
            ret[-1] + np.array([sl * math.cos(h), sl * math.sin(h), vec[2] / dist * sl])
        )
    ret = np.array(ret)
    dd = np.sqrt(((ret[:, None, :] - out[None, :, :]) ** 2).sum(axis=2))
    return {
        "ate": ate,
        "home_miss": hm,
        "yaw_rmse": yrmse,
        "cross_track": float(dd.min(axis=1).mean()),
        "ret_home": home_miss(ret[-1], out[0]),
    }


def main():
    d = build_world()
    print(
        f"world: {T_END:g}s, path {float(np.linalg.norm(np.diff(d['pos'], axis=0), axis=1).sum()):.0f}m",
        flush=True,
    )
    rows = []
    for combo in ("F1", "F2", "F3", "F4", "F5", "F6"):
        r = run_combo(d, combo)
        rows.append({"combo": combo, **r})
        print(
            f"  {combo}: ate={r['ate']:.4f} home={r['home_miss']:.4f} yaw={r['yaw_rmse']:.5f} "
            f"ret={r['ret_home']:.3f} cross={r['cross_track']:.3f}",
            flush=True,
        )
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "combo",
                "ate",
                "home_miss",
                "yaw_rmse",
                "cross_track",
                "ret_home",
            ],
        )
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    main()
