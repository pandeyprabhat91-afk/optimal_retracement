"""Offline replica of retrace_node C1 pipeline on dumped SITL CSVs, with truth-attitude audit."""

import math
import sys

import numpy as np

GRAV = 9.80665


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_from_euler(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_mat(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_rotate(q, v):
    return quat_mul(
        quat_mul(q, np.array([0.0, v[0], v[1], v[2]])),
        np.array([q[0], -q[1], -q[2], -q[3]]),
    )[1:]


def quat_update(q, gyr, dt):
    w = np.linalg.norm(gyr)
    if w < 1e-12:
        return q / np.linalg.norm(q)
    axis = gyr / w
    dq = np.array([math.cos(w * dt / 2), *(math.sin(w * dt / 2) * axis)])
    return quat_mul(q, dq) / np.linalg.norm(quat_mul(q, dq))


def quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def mat_to_euler(R):
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


T_W = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU2FRD = np.array([1.0, -1.0, -1.0])  # elementwise


def load(p):
    return np.loadtxt(p, delimiter=",")


P = sys.argv[4] if len(sys.argv) > 4 else r"E:\gpsdnav\shared_volume\sitl5"
imu = load(P + "_imu.csv")  # t, gx,gy,gz(FLU), ax,ay,az(FLU)
odom = load(P + "_odom.csv")  # t, px,py,pz(ENU), qw,qx,qy,qz, vx,vy,vz
NEG_YZ = len(sys.argv) > 1 and sys.argv[1] == "negyz"
FILT = sys.argv[2] if len(sys.argv) > 2 else "skip"  # skip | ema | both | g15
GYR_GATE = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

# init from first odom
q0 = odom[0, 4:8]
R_enu = quat_to_mat(q0)
R_ned = T_W @ R_enu @ np.diag([1.0, -1.0, -1.0])
FULL_INIT = len(sys.argv) > 5 and sys.argv[5] == "fullinit"


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
    qq = np.array([w, x, y, z])
    return qq / np.linalg.norm(qq)


if FULL_INIT:
    q = mat_to_quat(R_ned)
    yaw0 = quat_to_euler(q)[2]
else:
    yaw0 = math.atan2(R_ned[1, 0], R_ned[0, 0])
    q = quat_from_euler(0.0, 0.0, yaw0)
p = T_W @ odom[0, 1:4]
v = T_W @ odom[0, 8:11]
t0 = odom[0, 0]
print(f"yaw0={yaw0:.3f} p0={p.round(2)} NEG_YZ={NEG_YZ}")

# cal: first 2 s of IMU (bag-static)
m = imu[imu[:, 0] < 2.0]
g_bias = m[:, 1:4].mean(axis=0) * (np.array([1.0, -1.0, -1.0]) if not NEG_YZ else 1.0)
a_bias = (m[:, 4:7] * (np.array([1.0, -1.0, -1.0]) if not NEG_YZ else 1.0)).mean(
    axis=0
) - np.array([0.0, 0.0, -GRAV])
print(f"g_bias={g_bias.round(5)} a_bias={a_bias.round(3)}")

est, att_err = [], []
prev_t = None
skip = 0
ema_g = None
ALPHA = 0.25
for row in imu:
    t = row[0]
    g = row[1:4] * (np.array([1.0, -1.0, -1.0]) if not NEG_YZ else 1.0)
    a = row[4:7] * (np.array([1.0, -1.0, -1.0]) if not NEG_YZ else 1.0)
    if prev_t is None:
        prev_t = t
        continue
    dt = t - prev_t
    prev_t = t
    if FILT in ("ema", "both"):
        ema_g = g if ema_g is None else ALPHA * g + (1 - ALPHA) * ema_g
        g_use = ema_g
    else:
        g_use = g
    if FILT in ("skip", "both", "g15"):
        if np.linalg.norm(a) > 25 or np.linalg.norm(g_use) > GYR_GATE:
            skip += 1
            continue
    if t < 2.0:
        continue
    q = quat_update(q, g_use - g_bias, dt)
    a_ned = quat_rotate(q, a - a_bias) + np.array([0.0, 0.0, GRAV])
    v = v + a_ned * dt
    p = p + v * dt
    est.append([t, *p])
    # truth attitude in NED FRD for comparison
    j = np.searchsorted(odom[:, 0], t)
    R = T_W @ quat_to_mat(odom[min(j, len(odom) - 1), 4:8]) @ np.diag([1.0, -1.0, -1.0])
    et, ee = mat_to_euler(R), quat_to_euler(q)
    att_err.append([t, *np.abs((et - ee + np.pi) % (2 * np.pi) - np.pi)])
est = np.array(est)
att_err = np.array(att_err)
print(f"skip={skip} n={len(est)}")
print(" t | |poserr| |rpy err| est_xyz")
for s in range(0, 130, 10):
    i = np.searchsorted(est[:, 0], s)
    j = np.searchsorted(odom[:, 0], s)
    e = odom[min(j, len(odom) - 1), 1:4] - (T_W @ np.array([0.0, 0.0, 0.0]))
    # truth ENU vs est->ENU
    pe = T_W @ est[min(i, len(est) - 1), 1:4]
    pt = odom[min(j, len(odom) - 1), 1:4]
    ae = att_err[min(i, len(att_err) - 1), 1:4]
    print(
        f"{s:3d} poserr={np.linalg.norm(pe - pt):9.1f} rpyerr={ae.round(3)} est={pe.round(1)}"
    )
