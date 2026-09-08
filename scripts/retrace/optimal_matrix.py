"""Optimal-world matrix: PERFECT sensors synthesized from real log trajectories.

Your definition: no drift, no noise, compass exact, everything works.
Method: take GPS/ATT truth from Flight A/B, smooth it, differentiate into
ideal gyro/accel, synthesize exact mag/baro/thrust. Then REMOVE sensors one
at a time to test which factors are necessary/sufficient to retrace path.

Combos (removal ablation):
  F1 full       gyro+accel+mag+baro
  F2 no-gyro    attitude per-sample from accel tilt + mag yaw
  F3 no-accel   attitude only, position frozen (expect pos fail)
  F4 no-mag     gyro-only yaw (perfect gyro never drifts)
  F5 fake-accel thrust scalar as accelerometer substitute, level assumption
  F6 no-baro    perfect vertical from accel alone
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
from replay_retrace import gps_to_ned, load_msgs

ROOT = Path(__file__).resolve().parents[2]
LOGS = {
    "A": str(ROOT / "datasets" / "flight_logs" / "00000061.BIN"),
    "B": str(ROOT / "datasets" / "flight_logs" / "2026-06-29 17-05-31.bin"),
}
OUT = str(ROOT / "output" / "retrace" / "optimal_results.csv")
FIELD = np.array([0.5, 0.0, 0.8])  # arbitrary constant earth field (body = R^T f)


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def rotmat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def movavg(x, w):
    k = np.ones(w) / w
    return np.apply_along_axis(lambda m: np.convolve(m, k, mode="same"), 0, x)


def build_ideal(path):
    msgs = load_msgs(
        path,
        types=(
            "IMU",
            "MAG",
            "BARO",
            "GPS",
            "ATT",
            "POS",
            "RCOU",
            "CTUN",
            "VIBE",
            "BAT",
            "XKF1",
        ),
    )
    gps = [g for g in msgs["GPS"] if g.get("Status", 0) >= 3 and g.get("NSats", 0) >= 5]
    tg = np.array([g["TimeUS"] for g in gps]) / 1e6
    lat0, lon0, alt0 = gps[0]["Lat"], gps[0]["Lng"], gps[0]["Alt"]
    truth = np.array(
        [gps_to_ned(g["Lat"], g["Lng"], g["Alt"], lat0, lon0, alt0) for g in gps]
    )
    imu = sorted(msgs["IMU"], key=lambda m: m["TimeUS"])
    t = np.array([m["TimeUS"] for m in imu]) / 1e6
    t = t[np.concatenate(([True], np.diff(t) > 0))]  # drop duplicate stamps
    xkf = sorted(msgs["XKF1"], key=lambda m: m["TimeUS"])
    if len(xkf) < 20:
        raise ValueError("need XKF1 velocity for clean accel synthesis")
    tx = np.array([m["TimeUS"] for m in xkf]) / 1e6
    vel_ekf = movavg(np.array([[m["VN"], m["VE"], m["VD"]] for m in xkf]), 15)
    att = sorted(msgs["ATT"], key=lambda m: m["TimeUS"])
    for m in att:  # DF ATT is degrees; everything else is radians
        m["Roll"] = math.radians(m["Roll"])
        m["Pitch"] = math.radians(m["Pitch"])
        m["Yaw"] = math.radians(m["Yaw"])
    ta = np.array([m["TimeUS"] for m in att]) / 1e6
    yaw_u = np.unwrap(np.array([m["Yaw"] for m in att]))
    eul = np.array(
        [
            np.interp(t, ta, np.array([m["Roll"] for m in att])),
            np.interp(t, ta, np.array([m["Pitch"] for m in att])),
            np.interp(t, ta, yaw_u),
        ]
    ).T
    # ideal trajectory: log path GEOMETRY, reparameterized at constant speed
    # with a coordinated-flight attitude model. Analytic, exact, zero noise:
    # sensors derived from this truth are perfect by construction.
    geo = movavg(np.array([np.interp(t, tg, truth[:, i]) for i in range(3)]).T, 150)
    seg = np.linalg.norm(np.diff(geo, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total, dur = s[-1], t[-1] - t[0]
    v0 = total / dur
    tq = t - t[0]
    sq = v0 * tq  # constant-speed reparameterization
    pos = np.array([np.interp(sq, s, geo[:, i]) for i in range(3)]).T
    # heading from geometry, banked coordinated turns, level pitch
    hd = np.unwrap(np.arctan2(np.gradient(pos[:, 1]), np.gradient(pos[:, 0])))
    yaw_rate = np.gradient(hd) / np.maximum(np.gradient(tq), 1e-6)
    bank = np.arctan(v0 * yaw_rate / 9.80665)
    eul = np.array([np.clip(bank, -0.6, 0.6), np.zeros_like(tq), hd]).T
    dtq = np.maximum(np.gradient(tq), 1e-6)
    vel = np.gradient(pos, axis=0) / dtq[:, None]
    acc_ned = np.gradient(vel, axis=0) / dtq[:, None]
    dt = np.gradient(t)
    dt = np.maximum(dt, 1e-4)
    # ideal body sensors from smoothed world
    gyr = np.zeros_like(pos)
    acc = np.zeros_like(pos)
    mag = np.zeros_like(pos)
    # ideal body sensors from smoothed world; body-rate gyro needs
    # conj(Q[i-1]) * Q[i] order (world-frame delta is wrong for 3D motion)
    Q = [quat_from_euler(*e) for e in eul]
    for i in range(len(t)):
        R = rotmat(*eul[i])
        acc[i] = R.T @ (acc_ned[i] - np.array([0.0, 0.0, 9.80665]))
        mag[i] = R.T @ FIELD
        if i:
            dq = quat_mul(quat_conj(Q[i - 1]), Q[i])
            gyr[i] = 2 * dq[1:] / max(dt[i], 1e-6)
    baro = -pos[:, 2]
    hover = 0.5
    thr = hover * (1 + np.linalg.norm(acc_ned[:, :2], axis=1) / 9.80665)
    return {
        "t": t,
        "pos": pos,
        "eul": eul,
        "gyr": gyr,
        "acc": acc,
        "mag": mag,
        "baro": baro,
        "thr": thr,
        "hover": hover,
        "vel": vel,
    }


def run_combo(d, combo):
    t, pos = d["t"], d["pos"]
    i0 = np.searchsorted(t, t[0] + 10.0)
    q = quat_from_euler(*d["eul"][i0])
    p = pos[i0].copy()
    v = d["vel"][i0].copy()  # optimal world: last known velocity at loss
    off = 0.0
    if combo in ("F1", "F2"):
        seg = slice(max(0, i0 - 75), i0)
        my = np.array(
            [
                tilt_yaw(*d["mag"][j], *d["eul"][j][:2])
                for j in range(seg.start, seg.stop)
            ]
        )
        off = wrap(circ_mean(wrap(my - d["eul"][seg, 2])))
    est = [p.copy()]
    yaws = []
    for i in range(i0 + 1, len(t)):
        dt = float(min(max(t[i] - t[i - 1], 1e-4), 0.05))
        if combo == "F2":
            a = d["acc"][i]
            r = math.asin(max(-1.0, min(1.0, -a[1] / 9.80665)))
            pt = math.asin(max(-1.0, min(1.0, a[0] / 9.80665)))
            yw = tilt_yaw(*d["mag"][i], r, pt) - off
            q = quat_from_euler(r, pt, yw)
            eul = np.array([r, pt, yw])
        else:
            q = quat_update(q, d["gyr"][i], dt)
            eul = quat_to_euler(q)
            if combo in ("F1",):
                yw = tilt_yaw(*d["mag"][i], eul[0], eul[1]) - off
                eul[2] = eul[2] + 0.1 * wrap(yw - eul[2])
                q = quat_from_euler(*eul)
        if combo == "F3":
            pass  # attitude only
        elif combo == "F5":
            proxy = np.array([0.0, 0.0, -(d["thr"][i] / d["hover"]) * 9.80665])
            a_ned = quat_rotate(q, proxy) + np.array([0.0, 0.0, 9.80665])
            v = v + a_ned * dt
            p = p + v * dt
        else:
            a_ned = quat_rotate(q, d["acc"][i]) + np.array([0.0, 0.0, 9.80665])
            v = v + a_ned * dt
            p = p + v * dt
            if combo == "F1":
                p[2] = 0.9 * p[2] + 0.1 * pos[i][2]
        est.append(p.copy())
        yaws.append(eul[2])
    est = np.array(est)
    tru = pos[i0:]
    ate = compute_ate(est, tru)
    hm = home_miss(est[-1], tru[0])
    if combo == "F3":  # never moved: miss = full outbound distance
        hm = home_miss(est[-1], tru[-1])
    yw = np.array(yaws)
    yt = d["eul"][i0 + 1 :, 2]
    dy = wrap(np.unwrap(yw) - np.unwrap(yt))
    yrmse = float(np.sqrt(np.mean(dy**2)))
    # Exp2: pursuit back along 2 Hz crumbs, perfect-yaw noise floor
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
    cross = float(dd.min(axis=1).mean())
    rhome = home_miss(ret[-1], out[0])
    return {
        "ate": ate,
        "home_miss": hm,
        "yaw_rmse": yrmse,
        "cross_track": cross,
        "ret_home": rhome,
        "exp1_ok": bool(hm < 5.0),
        "exp2_ok": bool(rhome < 5.0),
    }


def main():
    rows = []
    for name, path in LOGS.items():
        print(f"LOAD {name}", flush=True)
        d = build_ideal(path)
        dur = d["t"][-1] - d["t"][0] - 10.0
        plen = float(np.linalg.norm(np.diff(d["pos"], axis=0), axis=1).sum())
        print(f"  ideal trajectory: {dur:.0f}s, path {plen:.0f}m", flush=True)
        for combo in ("F1", "F2", "F3", "F4", "F5", "F6"):
            r = run_combo(d, combo)
            rows.append({"log": name, "combo": combo, **r})
            print(
                f"  {combo}: exp1 home={r['home_miss']:.3f} yaw={r['yaw_rmse']:.4f} "
                f"exp2 home={r['ret_home']:.3f} cross={r['cross_track']:.3f}",
                flush=True,
            )
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "log",
                "combo",
                "ate",
                "home_miss",
                "yaw_rmse",
                "cross_track",
                "ret_home",
                "exp1_ok",
                "exp2_ok",
            ],
        )
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    main()
