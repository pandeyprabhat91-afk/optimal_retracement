"""Time-aligned score with horizontal/vertical split. Args: truth_bag est_bag [label]."""

import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry, Path

TRUTH_BAG, EST_BAG = sys.argv[1], sys.argv[2]
LBL = sys.argv[3] if len(sys.argv) > 3 else ""


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
truth_i = np.column_stack([np.interp(et, ot, ox[:, i]) for i in range(3)])
e = ex - ex[0] + truth_i[0]
eh, th = e[:, :2], truth_i[:, :2]
ev, tv = e[:, 2], truth_i[:, 2]
ah = float(np.linalg.norm(eh - th, axis=1).mean())
av = float(np.mean(np.abs(ev - tv)))
a3 = float(np.linalg.norm(e - truth_i, axis=1).mean())
hm_h = float(np.linalg.norm((ex[-1, :2] - ex[0, :2])))
print(
    f"{LBL} ATE3d={a3:.1f}m ATEh={ah:.1f}m ATEv={av:.1f}m home_h={hm_h:.1f}m n={len(ex)}"
)
