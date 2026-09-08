"""Live GPS-denied retrace estimator (C1-C6) for SITL.

Subscribes: IMU, mag, baro altitude, odom (init + v0 only, never fused),
            RC out (thrust proxy). Publishes /retrace/est_path + odometry.
No GPS subscribed: pure dead reckoning from t0 by construction.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, MagneticField

from .estimator import (
    GRAV,
    circ_mean,
    quat_from_euler,
    quat_rotate,
    quat_to_euler,
    quat_update,
    tilt_yaw,
)

COMBOS = {
    "C1": ("gyr", "acc"),
    "C2": ("gyr", "acc", "mag_yaw"),
    "C3": ("gyr", "acc", "mag_yaw", "baro_alt"),
    "C4": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag"),
    "C5": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag", "battery"),
    "C6": ("gyr", "acc", "mag_yaw", "baro_alt", "thrust_drag", "battery", "zupt"),
}


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def quat_from_msg(q):
    return np.array([q.w, q.x, q.y, q.z])


def quat_to_mat(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


# MAVROS speaks FLU-body / ENU-world (REP-103). The estimator core speaks
# ArduPilot FRD-body / NED-world. Fixed conversions at the boundary:
_T_W = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # ENU->NED


def body_flu_to_frd(v):
    v = np.asarray(v, dtype=float)
    return np.array([v[0], -v[1], -v[2]])


def world_enu_to_ned(v):
    return _T_W @ np.asarray(v, dtype=float)


def world_ned_to_enu(v):
    return _T_W @ np.asarray(v, dtype=float)  # T_W is its own inverse


def mat_to_quat(R):
    # Shepperd: rotation matrix -> quaternion [w,x,y,z]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def odom_quat_to_ned(q):
    # full FRD<-NED attitude from MAVROS odom (keeps rest tilt; level-init
    # leaks ~1 m/s2 after the first yaw turn once cal bias rotates with body)
    r_ned = _T_W @ quat_to_mat(q) @ np.diag([1.0, -1.0, -1.0])
    return mat_to_quat(r_ned)


def ros_time(node):
    s, ns = node.get_clock().now().seconds_nanoseconds()
    return s + ns * 1e-9


class RetraceNode(Node):
    def __init__(self):
        super().__init__("retrace_node")
        self.combo = self.declare_parameter("combo", "C4").value
        self.sensors = COMBOS[self.combo]
        self.calib_s = float(self.declare_parameter("calib_s", 1.0).value)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(
            Imu, "/target/mavros/imu/data", self.on_imu, sensor_qos
        )
        self.create_subscription(
            MagneticField, "/target/mavros/imu/mag", self.on_mag, sensor_qos
        )
        self.create_subscription(Altitude, "/target/mavros/altitude", self.on_baro, 10)
        self.create_subscription(
            Odometry, "/target/mavros/local_position/odom", self.on_odom, 10
        )
        try:
            from mavros_msgs.msg import RCOut

            self.create_subscription(RCOut, "/target/mavros/rc/out", self.on_rc, 10)
            self.has_rc = True
        except Exception:
            self.has_rc = False
        self.path_pub = self.create_publisher(Path, "/retrace/est_path", 10)
        self.odom_pub = self.create_publisher(Odometry, "/retrace/est_odom", 10)
        self.q = None
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.t0 = None
        self.g_bias = np.zeros(3)
        self.a_bias = np.zeros(3)
        self.mag_off = 0.0
        self.baro0 = None
        self.hover_thr = 1500.0
        self.cal_gyr, self.cal_acc, self.cal_thr = [], [], []
        self.mag_buf = []
        self.last_mag = None
        self.last_baro = None
        self.last_thr = 1500.0
        self.init_yaw = 0.0
        self.path = Path()
        self.path.header.frame_id = "map"
        self.n = 0
        # SITL-via-MAVROS IMU @~47 Hz is ALIASED: motor vibration spikes to
        # 1100 m/s2 / 37 rad/s (flight_sitl5: gyr>8 = 6.3%, acc>25 = 0.5%).
        # Gentle square never exceeds ~1.2 rad/s true rate: skip samples with
        # |gyr|>3 or |acc|>25, accumulating their dt into the next valid
        # sample so the time scale is preserved.
        self._skip_dt = 0.0
        self.n_skip = 0
        self.get_logger().info(
            f"retrace combo={self.combo} sensors={self.sensors} rc={self.has_rc}"
        )

    def on_odom(self, m):
        if self.q is not None:
            return
        self.q = odom_quat_to_ned(quat_from_msg(m.pose.pose.orientation))
        self.init_yaw = quat_to_euler(self.q)[2]
        self.p = world_enu_to_ned(
            [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
        )
        self.v = world_enu_to_ned(
            [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
        )
        self.t0 = ros_time(self)
        self.get_logger().info(
            f"t0 eul={quat_to_euler(self.q).round(3)} v0={self.v.round(2)}"
        )

    def on_mag(self, m):
        self.last_mag = body_flu_to_frd(
            [m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z]
        )

    def on_baro(self, m):
        # MAVROS relative: up-positive. NED world: down-positive.
        self.last_baro = -m.relative
        if self.baro0 is None:
            self.baro0 = -m.relative

    def on_rc(self, m):
        if m.channels:
            self.last_thr = float(sum(m.channels[:4]) / 4.0)

    def on_imu(self, m):
        if self.q is None:
            return
        t = ros_time(self)
        if not hasattr(self, "prev_t"):
            self.prev_t = t
            return
        dt = min(max(t - self.prev_t, 1e-4), 0.05)
        self.prev_t = t
        gyr = body_flu_to_frd(
            [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
        )
        acc = body_flu_to_frd(
            [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
        )
        # spike skip: non-physical for this flight envelope; bank dt for next
        if np.linalg.norm(acc) > 25.0 or np.linalg.norm(gyr) > 3.0:
            self._skip_dt += dt
            self.n_skip += 1
            return
        dt = min(dt + self._skip_dt, 0.1)
        self._skip_dt = 0.0
        # static calibration window
        if t - self.t0 < self.calib_s:
            self.cal_gyr.append(gyr)
            self.cal_acc.append(acc)
            if self.has_rc:
                self.cal_thr.append(self.last_thr)
            if self.last_mag is not None:
                eul = quat_to_euler(self.q)
                self.mag_buf.append(
                    tilt_yaw(*self.last_mag, eul[0], eul[1]) - self.init_yaw
                )
            return
        if self.n == 0 and self.cal_gyr:
            self.g_bias = np.mean(self.cal_gyr, axis=0)
            self.a_bias = np.mean(self.cal_acc, axis=0) - np.array([0.0, 0.0, -GRAV])
            if self.cal_thr:
                hover_mean = float(np.mean(self.cal_thr))
                if hover_mean > 1000.0:  # RC active; ground-disarmed reads 0
                    self.hover_thr = hover_mean
            if self.mag_buf:
                self.mag_off = wrap(circ_mean(np.array(self.mag_buf)))
            self._p0z = float(self.p[2])
            self.get_logger().info(
                f"cal done g_bias={self.g_bias.round(4)} mag_off={self.mag_off:.3f} hover={self.hover_thr:.0f}"
            )
        self.n += 1
        self.q = quat_update(self.q, gyr - self.g_bias, dt)
        eul = quat_to_euler(self.q)
        if "mag_yaw" in self.sensors and self.last_mag is not None:
            myaw = tilt_yaw(*self.last_mag, eul[0], eul[1]) - self.mag_off
            eul[2] = eul[2] + 0.1 * wrap(myaw - eul[2])
            self.q = quat_from_euler(*eul)
        a_ned = quat_rotate(self.q, acc - self.a_bias) + np.array([0.0, 0.0, GRAV])
        self.v = self.v + a_ned * dt
        self.p = self.p + self.v * dt
        if "baro_alt" in self.sensors and self.last_baro is not None:
            self.p[2] = 0.9 * self.p[2] + 0.1 * (
                self.last_baro - self.baro0 + self.p_init_z()
            )
        if "thrust_drag" in self.sensors:
            thr = (self.last_thr / self.hover_thr) if self.hover_thr else 1.0
            self.v[:2] -= 0.3 * thr * self.v[:2] * dt
        if "zupt" in self.sensors and np.linalg.norm(self.v) < 0.2:
            self.v *= 0.8
        if self.n % 25 == 0:  # ~10 Hz path
            now = self.get_clock().now().to_msg()
            p_enu = world_ned_to_enu(self.p)  # publish ENU for RViz/odom scoring
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p_enu
            self.path.poses.append(ps)
            self.path.header.stamp = now
            self.path_pub.publish(self.path)
            om = Odometry()
            om.header.stamp = now
            om.header.frame_id = "map"
            (
                om.pose.pose.position.x,
                om.pose.pose.position.y,
                om.pose.pose.position.z,
            ) = p_enu
            self.odom_pub.publish(om)

    def p_init_z(self):
        if not hasattr(self, "_p0z"):
            self._p0z = float(self.p[2])
        return self._p0z


def main():
    rclpy.init()
    node = RetraceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
