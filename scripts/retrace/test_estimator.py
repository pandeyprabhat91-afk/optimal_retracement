import math

import numpy as np

from estimator import quat_conj
from estimator import quat_from_euler, quat_rotate, quat_to_euler, quat_update


def test_quaternion_pure_yaw_spin():
    q = quat_from_euler(0.0, 0.0, 0.0)
    for _ in range(100):
        q = quat_update(q, np.array([0.0, 0.0, 1.0]), 0.01)
    assert abs(quat_to_euler(q)[2] - 1.0) < 1e-6


def test_static_hold_no_drift():
    q = quat_from_euler(0.0, 0.0, 0.5)
    p = np.zeros(3)
    v = np.zeros(3)
    for _ in range(100):
        q = quat_update(q, np.zeros(3), 0.01)
        a = quat_rotate(q, np.array([0.0, 0.0, -9.80665])) + np.array(
            [0.0, 0.0, 9.80665]
        )
        v = v + a * 0.01
        p = p + v * 0.01
    assert float(np.linalg.norm(p)) < 1e-9
    assert abs(quat_to_euler(q)[2] - 0.5) < 1e-9


def test_tilt_yaw_level_north():
    from estimator import tilt_yaw

    assert abs(tilt_yaw(1.0, 0.0, 0.2, 0.0, 0.0) - 0.0) < 1e-9


def test_body_gyro_roundtrip_3d():
    import math

    from estimator import quat_mul

    # combined 3D motion: body-rate synthesis must use conj(q0)*q1 order
    qs = [
        quat_from_euler(0.1 * math.sin(i * 0.1), 0.2 * math.sin(i * 0.07), 0.05 * i)
        for i in range(50)
    ]
    dt = 0.02
    q = qs[0]
    for i in range(1, len(qs)):
        dq = quat_mul(quat_conj(qs[i - 1]), qs[i])
        w = 2 * dq[1:] / dt
        q = quat_update(q, w, dt)
    got = quat_to_euler(q / np.linalg.norm(q))
    want = quat_to_euler(qs[-1])
    assert abs((got[2] - want[2] + math.pi) % (2 * math.pi) - math.pi) < 0.05
    assert abs(got[0] - want[0]) < 0.05
