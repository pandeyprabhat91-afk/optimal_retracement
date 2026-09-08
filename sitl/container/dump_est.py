"""Dump est_path positions + deltas from replay bag to find explosion onset."""

import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Path

BAG = sys.argv[1]
r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="sqlite3"), ConverterOptions("", ""))
t0 = None
rows = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic != "/retrace/est_path":
        continue
    m = deserialize_message(data, Path())
    if not m.poses:
        continue
    if t0 is None:
        t0 = t
    p = m.poses[-1].pose.position
    rows.append([(t - t0) / 1e9, p.x, p.y, p.z])
a = np.array(rows)
print(f"n={len(a)} span={a[-1, 0]:.1f}s")
print("t(s) x y z |step| for first 15:")
prev = None
for i in range(min(15, len(a))):
    step = np.linalg.norm(a[i, 1:] - prev) if prev is not None else 0.0
    print(
        f"{a[i, 0]:8.2f} {a[i, 1]:12.1f} {a[i, 2]:12.1f} {a[i, 3]:12.1f} step={step:.1f}"
    )
    prev = a[i, 1:].copy()
steps = np.linalg.norm(np.diff(a[:, 1:], axis=0), axis=1)
print(
    f"max step={steps.max():.1f}m at idx={steps.argmax()} t={a[steps.argmax() + 1, 0]:.1f}s"
)
print(f"steps>100m: {(steps > 100).sum()}, steps>1000m: {(steps > 1000).sum()}")
