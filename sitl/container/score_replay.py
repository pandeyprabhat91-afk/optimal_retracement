"""Score replay est_path bag vs truth odom bag. Args: truth_bag est_bag [out_csv]."""

import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry, Path

TRUTH_BAG = sys.argv[1]
EST_BAG = sys.argv[2]
OUT = sys.argv[3] if len(sys.argv) > 3 else None


def read(bag, want):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
    odom, est = [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/target/mavros/local_position/odom" and "odom" in want:
            m = deserialize_message(data, Odometry())
            odom.append(
                [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
            )
        elif topic == "/retrace/est_path" and "est" in want:
            m = deserialize_message(data, Path())
            if m.poses:
                p = m.poses[-1].pose.position
                est.append([p.x, p.y, p.z])
    return np.array(odom), np.array(est)


truth, _ = read(TRUTH_BAG, {"odom"})
_, est = read(EST_BAG, {"est"})
seg = np.linalg.norm(np.diff(truth[:, :2], axis=0), axis=1)
print(
    f"truth_n={len(truth)} est_n={len(est)} path_xy={seg.sum():.1f}m zmax={truth[:, 2].max():.2f}"
)
n = min(len(truth), len(est))
e = est[:n] - est[0] + truth[0]
err = np.linalg.norm(e - truth[:n], axis=1)
ate, mx, final = float(err.mean()), float(err.max()), float(err[-1])
hm_est = float(np.linalg.norm(est[-1] - est[0]))
hm_truth = float(np.linalg.norm(truth[-1] - truth[0]))
print(
    f"ATE={ate:.2f}m max={mx:.2f}m final={final:.2f}m hm_est={hm_est:.2f}m hm_truth={hm_truth:.2f}m"
)
if OUT:
    np.savetxt(
        OUT,
        np.column_stack([e, truth[:n]]),
        delimiter=",",
        header="est_x,est_y,est_z,truth_x,truth_y,truth_z",
    )
    print(f"wrote {OUT}")
print(f"RESULT ate={ate:.3f} max={mx:.3f} final={final:.3f} hm_est={hm_est:.3f}")
