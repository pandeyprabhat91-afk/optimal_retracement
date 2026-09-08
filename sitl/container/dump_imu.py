"""Per-second mean of IMU (FRD-converted) from bag. Args: bag."""

import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Imu

BAG = sys.argv[1]
r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="sqlite3"), ConverterOptions("", ""))
t0 = None
bins = {}
n = 0
while r.has_next():
    topic, data, t = r.read_next()
    if topic != "/target/mavros/imu/data":
        continue
    m = deserialize_message(data, Imu())
    if t0 is None:
        t0 = t
    s = int((t - t0) / 1e9)
    # FLU->FRD convert like node
    acc = [m.linear_acceleration.x, -m.linear_acceleration.y, -m.linear_acceleration.z]
    gyr = [m.angular_velocity.x, -m.angular_velocity.y, -m.angular_velocity.z]
    bins.setdefault(s, []).append(acc + gyr)
    n += 1
print(f"n_imu={n}")
for s in sorted(bins)[:12]:
    a = np.array(bins[s])
    print(
        f"t={s:3d}s n={len(a):4d} acc=[{a[:, 0].mean():7.3f} {a[:, 1].mean():7.3f} {a[:, 2].mean():7.3f}] gyr=[{a[:, 3].mean():8.5f} {a[:, 4].mean():8.5f} {a[:, 5].mean():8.5f}]"
    )
