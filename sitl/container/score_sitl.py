"""Score flight_sitl2 bag: odom truth path length/z + live C4 est_path ATE/home-miss."""

import sys

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry, Path

BAG = sys.argv[1] if len(sys.argv) > 1 else "/home/user/shared_volume/bags/flight_sitl2"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/user/shared_volume/sitl_live_c4.csv"

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id="sqlite3"), ConverterOptions("", ""))

odom_xyz, est_xyz = [], []
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == "/target/mavros/local_position/odom":
        m = deserialize_message(data, Odometry())
        odom_xyz.append(
            [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
        )
    elif topic == "/retrace/est_path":
        m = deserialize_message(data, Path())
        if m.poses:
            p = m.poses[-1].pose.position
            est_xyz.append([p.x, p.y, p.z])

odom_xyz = np.array(odom_xyz)
est_xyz = np.array(est_xyz)
seg = np.linalg.norm(np.diff(odom_xyz[:, :2], axis=0), axis=1)
print(f"odom_n={len(odom_xyz)} est_n={len(est_xyz)}")
print(
    f"path_len_xy={seg.sum():.1f}m z_mean={odom_xyz[:, 2].mean():.2f} z_max={odom_xyz[:, 2].max():.2f}"
)
print(f"start={odom_xyz[0].round(2)} end={odom_xyz[-1].round(2)}")

n = min(len(odom_xyz), len(est_xyz))
if n > 10:
    # align est start to odom start (retrace inits from odom, but resample: compare synced prefix)
    e = est_xyz[:n] - est_xyz[0] + odom_xyz[0]
    err = np.linalg.norm(e - odom_xyz[:n], axis=1)
    print(
        f"liveC4_n={n} ATE={err.mean():.2f}m max={err.max():.2f}m final={err[-1]:.2f}m"
    )
    print(
        f"home_miss_est={np.linalg.norm(est_xyz[-1] - est_xyz[0]):.2f}m home_miss_truth={np.linalg.norm(odom_xyz[-1] - odom_xyz[0]):.2f}m"
    )
    np.savetxt(
        OUT,
        np.column_stack([e, odom_xyz[:n]]),
        delimiter=",",
        header="est_x,est_y,est_z,truth_x,truth_y,truth_z",
    )
    print(f"wrote {OUT}")
