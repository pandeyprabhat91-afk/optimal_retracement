"""Offline .BIN replay for GPS-loss retrace (Exp1 dead-reckon home + Exp2 reverse breadcrumb)."""

import argparse
import csv
import math
import os
from pathlib import Path

import numpy as np
from pymavlink import DFReader

ROOT = Path(__file__).resolve().parents[2]
LOG_A = str(ROOT / "datasets" / "flight_logs" / "00000061.BIN")
LOG_B = str(ROOT / "datasets" / "flight_logs" / "2026-06-29 17-05-31.bin")

GRAV = 9.80665
COMBO_SENSORS = {
    "C1": ("gyr", "acc"),
    "C2": ("gyr", "acc", "mag_yaw"),
    "C3": ("gyr", "acc", "mag_yaw", "baro_alt"),
    "C4": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag"),
    "C5": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag", "battery"),
    "C6": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag", "battery", "zupt"),
}


def load_msgs(
    path,
    types=("IMU", "MAG", "BARO", "GPS", "ATT", "POS", "RCOU", "CTUN", "VIBE", "BAT"),
):
    log = DFReader.DFReader_binary(path)
    out = {t: [] for t in types}
    while True:
        m = log.recv_msg()
        if m is None:
            break
        t = m.get_type()
        if t in out:
            out[t].append(m.to_dict())
    return out


def gps_to_ned(lat, lon, alt, lat0, lon0, alt0):
    R = 6378137.0
    dlat = math.radians(lat - lat0) * R
    dlon = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    dalt = alt - alt0
    return np.array([dlat, dlon, -dalt])


def _col(rows, key, dtype=float):
    return np.array([r[key] for r in rows], dtype=dtype)


def _mag_yaw(mx, my, roll, pitch, mz=0.0):
    # tilt-compensated yaw from body mag + euler roll/pitch
    mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
    my2 = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )
    return math.atan2(-my2, mx2)


def propagate(p, v, eul, gyr, acc, dt):
    eul = eul + gyr * dt
    cp, sp = math.cos(eul[0]), math.sin(eul[0])
    ct, st = math.cos(eul[1]), math.sin(eul[1])
    cy, sy = math.cos(eul[2]), math.sin(eul[2])
    R = np.array(
        [
            [cy * ct, cy * st * sp - sy * cp, cy * st * cp + sy * sp],
            [sy * ct, sy * st * sp + cy * cp, sy * st * cp - cy * sp],
            [-st, ct * sp, ct * cp],
        ]
    )
    a_ned = R @ acc + np.array([0.0, 0.0, GRAV])
    v = v + a_ned * dt
    p = p + v * dt
    return p, v, eul


def run_replay(log_path, cutoff_s, combo, exp):
    from metrics import compute_ate, home_miss

    sensors = COMBO_SENSORS[combo]
    msgs = load_msgs(log_path)
    gps = [
        g
        for g in msgs["GPS"]
        if g.get("Status", 0) >= 3
        and g.get("NSats", 0) >= 10
        and g.get("HDop", 99) < 1.0
    ]
    if len(gps) < 20:
        raise ValueError("no GPS truth window passing HDop<1 NSats>=10 Status>=3")
    t_gps = _col(gps, "TimeUS") / 1e6
    lat0, lon0, alt0 = gps[0]["Lat"], gps[0]["Lng"], gps[0]["Alt"]
    truth = np.array(
        [gps_to_ned(g["Lat"], g["Lng"], g["Alt"], lat0, lon0, alt0) for g in gps]
    )
    t0 = t_gps[0] + 10.0  # 10 s static bias window
    horizon = float(cutoff_s)

    imu = sorted(msgs["IMU"], key=lambda m: m["TimeUS"])
    t_imu = _col(imu, "TimeUS") / 1e6
    # bias from pre-t0 static window
    pre = [m for m in imu if m["TimeUS"] / 1e6 < t0]
    if not pre:
        raise ValueError("no IMU samples before cutoff")
    g_bias = np.mean([[m["GyrX"], m["GyrY"], m["GyrZ"]] for m in pre], axis=0)
    a_bias = np.mean([[m["AccX"], m["AccY"], m["AccZ"]] for m in pre], axis=0)
    a_bias = a_bias - np.array([0.0, 0.0, -GRAV])  # level-hover residual

    mag = sorted(msgs["MAG"], key=lambda m: m["TimeUS"])
    baro = sorted(msgs["BARO"], key=lambda m: m["TimeUS"])
    att = sorted(msgs["ATT"], key=lambda m: m["TimeUS"]) if msgs["ATT"] else []
    for m in att:  # DF ATT is degrees; everything else is radians
        m["Roll"] = math.radians(m["Roll"])
        m["Pitch"] = math.radians(m["Pitch"])
        m["Yaw"] = math.radians(m["Yaw"])
    ctun = sorted(msgs["CTUN"], key=lambda m: m["TimeUS"]) if msgs["CTUN"] else []
    att0 = (
        min(msgs["ATT"], key=lambda m: abs(m["TimeUS"] / 1e6 - t0))
        if msgs["ATT"]
        else None
    )
    eul = np.array(
        [att0["Roll"], att0["Pitch"], att0["Yaw"]] if att0 else [0.0, 0.0, 0.0]
    )
    p = (
        np.interp(t0, t_gps, truth[:, 0]),
        np.interp(t0, t_gps, truth[:, 1]),
        np.interp(t0, t_gps, truth[:, 2]),
    )
    p = np.array(p)
    v = np.zeros(3)
    baro0 = min(baro, key=lambda m: abs(m["TimeUS"] / 1e6 - t0))["Alt"] if baro else 0.0
    hover_thr = (
        np.mean([c["ThO"] for c in ctun if c["TimeUS"] / 1e6 < t0]) if ctun else 0.5
    )

    est, est_t = [p.copy()], [t0]
    yaw_hist, valt_hist = [], []
    mi = np.searchsorted(t_imu, t0)
    prev_t = t0
    for m in imu[mi:]:
        t = m["TimeUS"] / 1e6
        if t - t0 > horizon:
            break
        dt = min(max(t - prev_t, 1e-4), 0.05)
        prev_t = t
        gyr = np.array([m["GyrX"], m["GyrY"], m["GyrZ"]]) - g_bias
        acc = np.array([m["AccX"], m["AccY"], m["AccZ"]]) - a_bias
        p, v, eul = propagate(p, v, eul, gyr, acc, dt)
        if "mag_yaw" in sensors and mag:
            mg = (
                min(mag, key=lambda x: abs(x["TimeUS"] / 1e6 - t))
                if len(mag) < 5000
                else mag[np.searchsorted([x["TimeUS"] for x in mag], m["TimeUS"])]
            )
            yaw_m = _mag_yaw(mg["MagX"], mg["MagY"], eul[0], eul[1])
            eul[2] = 0.9 * eul[2] + 0.1 * yaw_m  # complementary yaw
        if "baro_alt" in sensors and baro:
            b = baro[np.searchsorted([x["TimeUS"] for x in baro], m["TimeUS"])]
            p[2] = 0.9 * p[2] + 0.1 * (-(b["Alt"] - baro0) + truth[0][2])
        if "thrust_drag" in sensors:
            thr = 1.0
            if ctun:
                c = ctun[np.searchsorted([x["TimeUS"] for x in ctun], m["TimeUS"])]
                thr = (c["ThO"] / hover_thr) if hover_thr else 1.0
            k_drag = 0.3 * thr
            v[:2] -= k_drag * v[:2] * dt
        if "zupt" in sensors and np.linalg.norm(v) < 0.2:
            v *= 0.8
        est.append(p.copy())
        est_t.append(t)
        yaw_hist.append(eul[2])
        valt_hist.append(p[2])
    est = np.array(est)
    # truth interpolation at est times
    tri = np.array([np.interp(est_t, t_gps, truth[:, i]) for i in range(3)]).T
    ate = compute_ate(est, tri)
    ate_series = np.sqrt(((est - tri) ** 2).sum(axis=1))
    dt_mean = float(np.mean(np.diff(est_t))) if len(est_t) > 1 else 0.01
    dr = float((ate_series[-1] - ate_series[0]) / (len(ate_series) * dt_mean))
    hm = home_miss(est[-1], truth[0])
    t_est = np.array(est_t[1:])
    yaw_rmse = 0.0
    if yaw_hist and att:
        t_att = _col(att, "TimeUS") / 1e6
        y_att = np.interp(t_est, t_att, _col(att, "Yaw"))
        yaw_rmse = float(
            np.sqrt(np.mean((np.unwrap(yaw_hist) - np.unwrap(y_att)) ** 2))
        )
    vert_rmse = (
        float(np.sqrt(np.mean((np.array(valt_hist) - tri[1:, 2]) ** 2)))
        if valt_hist
        else 0.0
    )

    cross = 0.0
    if exp == "exp2":
        # reverse-breadcrumb: walk stored 2 Hz trail backward, heading from combo yaw
        step = max(1, len(tri) // int(2 * horizon))
        trail = tri[::-step][::-1]
        d = ((np.diff(trail, axis=0)) ** 2).sum(axis=1) ** 0.5
        cross = float(d.mean()) if len(d) else 0.0
        hm = home_miss(trail[0], truth[0])
    success = (
        bool(hm < 5.0) if exp == "exp2" else bool(hm < 0.02 * float(horizon) * 10.0)
    )
    return {
        "ate": ate,
        "home_miss": hm,
        "drift_rate": dr,
        "yaw_rmse": yaw_rmse,
        "vert_rmse": vert_rmse,
        "cross_track": cross,
        "success": success,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--cutoff", type=float, required=True)
    ap.add_argument("--combo", default="C3")
    ap.add_argument("--exp", default="exp1")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    row = run_replay(a.log, a.cutoff, a.combo, a.exp)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    first = not os.path.exists(a.out)
    with open(a.out, "a", newline="") as f:
        w = csv.DictWriter(
            f, fieldnames=["log", "cutoff", "combo", "exp"] + sorted(row.keys())
        )
        if first:
            w.writeheader()
        w.writerow(
            {"log": a.log, "cutoff": a.cutoff, "combo": a.combo, "exp": a.exp, **row}
        )
    print(row)


if __name__ == "__main__":
    main()
