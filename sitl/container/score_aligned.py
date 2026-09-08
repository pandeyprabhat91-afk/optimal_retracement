"""Time-aligned score: interpolate truth odom to est timestamps using bag t. Args: truth_bag est_bag."""

import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry, Path

TRUTH_BAG, EST_BAG = sys.argv[1], sys.argv[2]


def read(bag, want_odom, want_est):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
    ot, ox, et, ex = [], [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == "/target/mavros/local_position/odom" and want_odom:
            m = deserialize_message(data, Odometry())
            ot.append(t / 1e9)
            ox.append(
                [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
            )
        elif topic == "/retrace/est_path" and want_est:
            m = deserialize_message(data, Path())
            if m.poses:
                p = m.poses[-1].pose.position
                et.append(t / 1e9)
                ex.append([p.x, p.y, p.z])
    return (np.array(ot), np.array(ox)), (np.array(et), np.array(ex))


(ot, ox), (_, _) = read(TRUTH_BAG, True, False)
(_, _), (et, ex) = read(EST_BAG, False, True)
ot -= ot[0]
et -= et[0]
# interpolate truth at est times (est clock may span beyond truth; clip)
tx = np.interp(et, ot, ox[:, 0])
ty = np.interp(et, ot, ox[:, 1])
tz = np.interp(et, ot, ox[:, 2])
truth_i = np.column_stack([tx, ty, tz])
e = ex - ex[0] + truth_i[0]
err = np.linalg.norm(e - truth_i, axis=1)
print(f"n_est={len(ex)} span_est={et[-1]:.1f}s span_truth={ot[-1]:.1f}s")
print(f"ATE={err.mean():.2f}m max={err.max():.2f}m final={err[-1]:.2f}m")
print(f"est_path_len={np.linalg.norm(np.diff(ex[:, :2], axis=0), axis=1).sum():.1f}m")
print(f"RESULT ate={err.mean():.3f} max={err.max():.3f} final={err[-1]:.3f}")
