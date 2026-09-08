"""Dump imu+odom+mag+baro+rc from bag to CSVs for offline debug."""

import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Imu, MagneticField
from mavros_msgs.msg import Altitude, RCOut
from nav_msgs.msg import Odometry

BAG, OUT = sys.argv[1], sys.argv[2]
r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="sqlite3"), ConverterOptions("", ""))
imu, odom, mag, baro, rc = [], [], [], [], []
t0 = None
while r.has_next():
    topic, data, t = r.read_next()
    if t0 is None:
        t0 = t
    ts = (t - t0) / 1e9
    if topic == "/target/mavros/imu/data":
        m = deserialize_message(data, Imu())
        imu.append(
            [
                ts,
                m.angular_velocity.x,
                m.angular_velocity.y,
                m.angular_velocity.z,
                m.linear_acceleration.x,
                m.linear_acceleration.y,
                m.linear_acceleration.z,
            ]
        )
    elif topic == "/target/mavros/local_position/odom":
        m = deserialize_message(data, Odometry())
        odom.append(
            [
                ts,
                m.pose.pose.position.x,
                m.pose.pose.position.y,
                m.pose.pose.position.z,
                m.pose.pose.orientation.w,
                m.pose.pose.orientation.x,
                m.pose.pose.orientation.y,
                m.pose.pose.orientation.z,
                m.twist.twist.linear.x,
                m.twist.twist.linear.y,
                m.twist.twist.linear.z,
            ]
        )
    elif topic == "/target/mavros/imu/mag":
        m = deserialize_message(data, MagneticField())
        mag.append([ts, m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z])
    elif topic == "/target/mavros/altitude":
        m = deserialize_message(data, Altitude())
        baro.append([ts, m.relative])
    elif topic == "/target/mavros/rc/out":
        m = deserialize_message(data, RCOut())
        ch = list(m.channels[:4]) if len(m.channels) >= 4 else [1500] * 4
        rc.append([ts] + ch)
for name, arr in [
    ("imu", imu),
    ("odom", odom),
    ("mag", mag),
    ("baro", baro),
    ("rc", rc),
]:
    np.savetxt(f"{OUT}_{name}.csv", np.array(arr), delimiter=",")
    print(name, np.array(arr).shape)
