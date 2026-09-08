"""Full ablation matrix: C1-C6 x 10/30/60s x Flight A/B x exp1/exp2.

Loads each .BIN once, propagates 60 s per combo, slices horizons.
Mirrors replay_retrace.run_replay estimator (preloaded-data variant).
"""

import csv
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from metrics import compute_ate, home_miss
from plots import combo_bars, error_vs_time, plan_view
from replay_retrace import COMBO_SENSORS, gps_to_ned, load_msgs

ROOT = Path(__file__).resolve().parents[2]
LOGS = {
    "A": str(ROOT / "datasets" / "flight_logs" / "00000061.BIN"),
    "B": str(ROOT / "datasets" / "flight_logs" / "2026-06-29 17-05-31.bin"),
}
HORIZONS = (10.0, 30.0, 60.0)
OUT_DIR = str(ROOT / "output" / "retrace")


def build_data(msgs):
    gps = [
        g
        for g in msgs["GPS"]
        if g.get("Status", 0) >= 3
        and g.get("NSats", 0) >= 10
        and g.get("HDop", 99) < 1.0
    ]
    if len(gps) < 20:
        raise ValueError("no GPS truth window passing gates")
    t_gps = np.array([g["TimeUS"] for g in gps]) / 1e6
    lat0, lon0, alt0 = gps[0]["Lat"], gps[0]["Lng"], gps[0]["Alt"]
    truth = np.array(
        [gps_to_ned(g["Lat"], g["Lng"], g["Alt"], lat0, lon0, alt0) for g in gps]
    )
    imu = sorted(msgs["IMU"], key=lambda m: m["TimeUS"])
    mag = sorted(msgs["MAG"], key=lambda m: m["TimeUS"])
    baro = sorted(msgs["BARO"], key=lambda m: m["TimeUS"])
    att = sorted(msgs["ATT"], key=lambda m: m["TimeUS"]) if msgs["ATT"] else []
    for m in att:  # DF ATT is degrees; everything else is radians
        m["Roll"] = math.radians(m["Roll"])
        m["Pitch"] = math.radians(m["Pitch"])
        m["Yaw"] = math.radians(m["Yaw"])
    ctun = sorted(msgs["CTUN"], key=lambda m: m["TimeUS"]) if msgs["CTUN"] else []
    vibe = sorted(msgs["VIBE"], key=lambda m: m["TimeUS"]) if msgs["VIBE"] else []
    allgps = sorted(msgs["GPS"], key=lambda m: m["TimeUS"]) if msgs["GPS"] else []
    return {
        "t_gps": t_gps,
        "truth": truth,
        "t_imu": np.array([m["TimeUS"] for m in imu]) / 1e6,
        "gyr": np.array([[m["GyrX"], m["GyrY"], m["GyrZ"]] for m in imu]),
        "acc": np.array([[m["AccX"], m["AccY"], m["AccZ"]] for m in imu]),
        "t_mag": np.array([m["TimeUS"] for m in mag]) / 1e6 if mag else np.array([]),
        "mag": np.array([[m["MagX"], m["MagY"], m["MagZ"]] for m in mag])
        if mag
        else np.zeros((0, 3)),
        "mofs": np.array(
            [[m.get("OfsX", 0), m.get("OfsY", 0), m.get("OfsZ", 0)] for m in mag]
        )
        if mag
        else np.zeros((0, 3)),
        "t_baro": np.array([m["TimeUS"] for m in baro]) / 1e6 if baro else np.array([]),
        "balt": np.array([m["Alt"] for m in baro]) if baro else np.array([]),
        "t_att": np.array([m["TimeUS"] for m in att]) / 1e6 if att else np.array([]),
        "att": np.array([[m["Roll"], m["Pitch"], m["Yaw"]] for m in att])
        if att
        else np.zeros((0, 3)),
        "t_ctun": np.array([m["TimeUS"] for m in ctun]) / 1e6 if ctun else np.array([]),
        "tho": np.array([m["ThO"] for m in ctun]) if ctun else np.array([]),
        "t_vibe": np.array([m["TimeUS"] for m in vibe]) / 1e6 if vibe else np.array([]),
        "vibe": np.array([[m["VibeX"], m["VibeY"], m["VibeZ"]] for m in vibe])
        if vibe
        else np.zeros((0, 3)),
        "t_spd": np.array([m["TimeUS"] for m in allgps]) / 1e6
        if allgps
        else np.array([]),
        "spd": np.array([m.get("Spd", 0) for m in allgps]) if allgps else np.array([]),
    }


def propagate_combo(d, combo, t0, horizon):
    import math

    from estimator import (
        GRAV,
        circ_mean,
        find_static,
        quat_from_euler,
        quat_rotate,
        quat_to_euler,
        quat_update,
        tilt_yaw,
    )

    def wrap(x):
        return (x + math.pi) % (2 * math.pi) - math.pi

    sensors = COMBO_SENSORS[combo]
    # optimal-world bias: longest proven-static window before loss
    pre_idx = np.nonzero(d["t_imu"] < t0)[0]
    use = pre_idx
    if len(d["t_spd"]) and len(pre_idx):
        spd_i = np.interp(d["t_imu"][pre_idx], d["t_spd"], d["spd"])
        if len(d["t_vibe"]):
            vib_i = np.linalg.norm(
                np.array(
                    [
                        np.interp(d["t_imu"][pre_idx], d["t_vibe"], d["vibe"][:, j])
                        for j in range(3)
                    ]
                ).T,
                axis=1,
            )
        else:
            vib_i = np.zeros_like(spd_i)
        m = find_static(d["t_imu"][pre_idx], spd_i, vib_i)
        if m is not None and int(m.sum()) > 10:
            use = pre_idx[m]
    g_bias = d["gyr"][use].mean(axis=0)
    a_bias = d["acc"][use].mean(axis=0) - np.array([0.0, 0.0, -GRAV])
    ai = np.searchsorted(d["t_att"], t0).clip(0, len(d["t_att"]) - 1)
    eul0 = d["att"][ai].copy() if len(d["t_att"]) else np.zeros(3)
    q = quat_from_euler(*eul0)
    # mag yaw-offset: circular mean vs known-good pre-loss heading
    mag_off = 0.0
    if "mag_yaw" in sensors and len(d["t_mag"]) and len(d["t_att"]):
        m0 = (d["t_mag"] >= t0 - 3.0) & (d["t_mag"] < t0)
        if int(m0.sum()) > 3:
            e0 = np.array(
                [
                    np.interp(d["t_mag"][m0], d["t_att"], d["att"][:, i])
                    for i in range(3)
                ]
            ).T
            my = np.array(
                [
                    tilt_yaw(*(d["mag"][j] - d["mofs"][j]), e_row[0], e_row[1])
                    for j, e_row in zip(np.nonzero(m0)[0], e0)
                ]
            )
            mag_off = wrap(circ_mean(wrap(my - e0[:, 2])))
    p = np.array([np.interp(t0, d["t_gps"], d["truth"][:, i]) for i in range(3)])
    v = np.zeros(3)
    bi = np.searchsorted(d["t_baro"], t0).clip(0, len(d["t_baro"]) - 1)
    baro0 = d["balt"][bi] if len(d["t_baro"]) else 0.0
    hover = d["tho"][d["t_ctun"] < t0].mean() if len(d["t_ctun"]) else 0.5
    sel = (d["t_imu"] >= t0) & (d["t_imu"] <= t0 + horizon)
    idx = np.nonzero(sel)[0]
    est = np.empty((len(idx) + 1, 3))
    est[0] = p
    yaws = np.empty(len(idx))
    valts = np.empty(len(idx))
    prev_t = t0
    for k, i in enumerate(idx):
        t = d["t_imu"][i]
        dt = float(min(max(t - prev_t, 1e-4), 0.05))
        prev_t = t
        q = quat_update(q, d["gyr"][i] - g_bias, dt)
        eul = quat_to_euler(q)
        if "mag_yaw" in sensors and len(d["t_mag"]):
            j = np.searchsorted(d["t_mag"], t).clip(0, len(d["t_mag"]) - 1)
            mv = d["mag"][j] - d["mofs"][j]
            myaw = tilt_yaw(mv[0], mv[1], mv[2], eul[0], eul[1]) - mag_off
            eul[2] = eul[2] + 0.1 * wrap(myaw - eul[2])
            q = quat_from_euler(*eul)
        a_ned = quat_rotate(q, d["acc"][i] - a_bias) + np.array([0.0, 0.0, GRAV])
        v = v + a_ned * dt
        p = p + v * dt
        if "baro_alt" in sensors and len(d["t_baro"]):
            j = np.searchsorted(d["t_baro"], t).clip(0, len(d["t_baro"]) - 1)
            p[2] = 0.9 * p[2] + 0.1 * (-(d["balt"][j] - baro0) + d["truth"][0][2])
        if "thrust_drag" in sensors:
            thr = 1.0
            if len(d["t_ctun"]):
                j = np.searchsorted(d["t_ctun"], t).clip(0, len(d["t_ctun"]) - 1)
                thr = (d["tho"][j] / hover) if hover else 1.0
            v[:2] -= 0.3 * thr * v[:2] * dt
        if "zupt" in sensors and np.linalg.norm(v) < 0.2:
            v *= 0.8
        est[k + 1] = p
        yaws[k] = eul[2]
        valts[k] = p[2]
    t_est = np.concatenate(([t0], d["t_imu"][idx]))
    return est, t_est, yaws, valts


def score(d, est, t_est, yaws, valts, horizon, exp):
    tri = np.array([np.interp(t_est, d["t_gps"], d["truth"][:, i]) for i in range(3)]).T
    ate = compute_ate(est, tri)
    series = np.sqrt(((est - tri) ** 2).sum(axis=1))
    dtm = float(np.mean(np.diff(t_est))) if len(t_est) > 1 else 0.01
    dr = float((series[-1] - series[0]) / (len(series) * dtm))
    hm = home_miss(est[-1], d["truth"][0])
    yaw_rmse = 0.0
    if len(yaws) and len(d["t_att"]):
        ya = np.interp(t_est[1:], d["t_att"], d["att"][:, 2])
        dy = np.unwrap(yaws) - np.unwrap(ya)
        dy = (dy + math.pi) % (2 * math.pi) - math.pi
        yaw_rmse = float(np.sqrt(np.mean(dy**2)))
    vert = float(np.sqrt(np.mean((valts - tri[1:, 2]) ** 2))) if len(valts) else 0.0
    cross = 0.0
    if exp == "exp2":
        # Honest return sim: outbound leg = GPS-aided tri (launch tri[0]
        # -> loss tri[-1]); on loss, pursuit back along 2 Hz crumbs with
        # per-step heading noise from combo yaw quality (random-walk:
        # sigma = yaw_rmse / sqrt(N), floor 0.02 rad, seed 42).
        step = max(1, len(tri) // int(2 * horizon))
        out = tri[::step]
        loss, launch = out[-1].copy(), out[0].copy()
        sig = max(float(yaw_rmse) / math.sqrt(max(1, len(yaws))), 0.02)
        rng = np.random.default_rng(42)
        ret = [loss]
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
                ret[-1]
                + np.array([sl * math.cos(h), sl * math.sin(h), vec[2] / dist * sl])
            )
        ret = np.array(ret)
        dd = np.sqrt(((ret[:, None, :] - out[None, :, :]) ** 2).sum(axis=2))
        cross = float(dd.min(axis=1).mean())
        hm = home_miss(ret[-1], launch)
        success = bool(hm < 5.0)
    else:
        success = bool(hm < 0.2 * float(horizon))
    return {
        "ate": ate,
        "home_miss": hm,
        "drift_rate": dr,
        "yaw_rmse": yaw_rmse,
        "vert_rmse": vert,
        "cross_track": cross,
        "success": success,
    }


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = []
    tracks = {}
    data_by_log = {}
    for name, path in LOGS.items():
        print(f"LOAD {name} {path}", flush=True)
        d = build_data(load_msgs(path))
        data_by_log[name] = d
        t0 = d["t_gps"][0] + 10.0
        for combo in ("C1", "C2", "C3", "C4", "C5", "C6"):
            est, t_est, yaws, valts = propagate_combo(d, combo, t0, max(HORIZONS))
            for h in HORIZONS:
                k = np.searchsorted(t_est, t0 + h)
                e, te = est[: k + 1], t_est[: k + 1]
                yw, va = yaws[:k], valts[:k]
                for exp in ("exp1", "exp2"):
                    r = score(d, e, te, yw, va, h, exp)
                    rows.append(
                        {"log": name, "cutoff": h, "combo": combo, "exp": exp, **r}
                    )
                    print(
                        f"{name} {combo} {h:g}s {exp}: ate={r['ate']:.1f} home={r['home_miss']:.1f}"
                        f" cross={r['cross_track']:.1f} yawrmse={r['yaw_rmse']:.2f} ok={r['success']}",
                        flush=True,
                    )
            if name == "A":
                tracks[combo] = (est, t_est)
        # keep last log's data for plots below (A only used)
    with open(os.path.join(OUT_DIR, "results.csv"), "w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "log",
                "cutoff",
                "combo",
                "exp",
                "ate",
                "home_miss",
                "drift_rate",
                "yaw_rmse",
                "vert_rmse",
                "cross_track",
                "success",
            ],
        )
        w.writeheader()
        w.writerows(rows)
    # plots: plan view + error-vs-time for A/exp1/30s, bars per (log,exp,h)
    dA = data_by_log["A"]
    te = tracks["C3"][1]
    k30 = np.searchsorted(te, te[0] + 30.0)
    tri30 = np.array(
        [np.interp(te[: k30 + 1], dA["t_gps"], dA["truth"][:, i]) for i in range(3)]
    ).T
    plan_view(
        tri30[:, :2],
        {c: tracks[c][0][: k30 + 1][:, :2] for c in tracks},
        os.path.join(OUT_DIR, "plan_A_exp1_30s.png"),
    )
    err = {
        c: np.sqrt(((tracks[c][0][: k30 + 1] - tri30) ** 2).sum(axis=1)) for c in tracks
    }
    error_vs_time(
        te[: k30 + 1] - te[0], err, os.path.join(OUT_DIR, "err_A_exp1_30s.png")
    )
    from collections import defaultdict

    groups = defaultdict(list)
    for r in rows:
        groups[(r["log"], r["exp"], r["cutoff"])].append(r)
    for (log, exp, h), g in sorted(groups.items()):
        combo_bars(
            [r["combo"] for r in g],
            [r["home_miss"] for r in g],
            os.path.join(OUT_DIR, f"bars_{log}_{exp}_{h:g}s.png"),
        )
    print(f"wrote {len(rows)} rows + plots", flush=True)


if __name__ == "__main__":
    main()
