"""From-scratch optimal-world inertial estimator.

Assumptions (optimal): zero wind, vibration-gated static calibration, last
known-good heading at loss, hard-iron offsets applied, mag yaw-offset
calibrated once at loss and held fixed.
"""

import math

import numpy as np

GRAV = 9.80665


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


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


def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_rotate(q, v):
    qv = np.array([0.0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return quat_mul(quat_mul(q, qv), qc)[1:]


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


def tilt_yaw(mx, my, mz, roll, pitch):
    xh = mx * math.cos(pitch) + mz * math.sin(pitch)
    yh = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )
    return math.atan2(-yh, xh)


def circ_mean(angles):
    return math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))


def find_static(t, spd, vibe, spd_max=0.3, vibe_max=30.0, min_len_s=3.0):
    """Longest window with spd < max and vibe < max. Returns mask or None."""
    ok = (spd < spd_max) & (vibe < vibe_max)
    best, start, n = (None, 0), None, len(t)
    i = 0
    while i < n:
        if ok[i]:
            j = i
            while j + 1 < n and ok[j + 1]:
                j += 1
            if t[j] - t[i] >= min_len_s and (best[0] is None or j - i > best[1]):
                best = (i, j - i)
            i = j + 1
        else:
            i += 1
    if best[0] is None:
        return None
    m = np.zeros(n, dtype=bool)
    m[best[0] : best[0] + best[1] + 1] = True
    return m
