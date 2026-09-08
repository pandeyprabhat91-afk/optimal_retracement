"""Minimal flight commander for retrace SITL tests: arm, takeoff, square, land.

All PX4 I/O via MAVROS (proven TX path: fcu_url remote 14580):
- setpoints -> /target/mavros/setpoint_position/local (also the bag record topic)
- mode/arm  -> /target/mavros/set_mode + /target/mavros/cmd/arming services
- verify    -> /target/mavros/state subscription (mode string + armed flag)

No raw pymavlink: the PX4 GCS link (18570->14550 broadcast) is unreliable here
(instance observed dead while MAVROS onboard link stayed connected).
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node


class Commander(Node):
    def __init__(self):
        super().__init__("commander")
        self.sp_pub = self.create_publisher(
            PoseStamped, "/target/mavros/setpoint_position/local", 10
        )
        self.state = None
        self.create_subscription(State, "/target/mavros/state", self.on_state, 10)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.timer = self.create_timer(0.1, self.tick)
        # wait for MAVROS state (proves MAVROS<->PX4 link alive)
        end = time.time() + 30
        while time.time() < end and self.state is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.state is None:
            raise RuntimeError("no MAVROS state")
        self.get_logger().info(
            f"mavros connected={self.state.connected} mode={self.state.mode} "
            f"armed={self.state.armed}"
        )

    def on_state(self, m):
        self.state = m

    def tick(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.sp_pub.publish(self.pose)

    def goto(self, x, y, z, dwell):
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        self.get_logger().info(f"goto {(x, y, z)} dwell {dwell}s")
        end = time.time() + dwell
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_state(self, mode=None, armed=None, timeout=10):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if mode is not None and self.state.mode != mode:
                continue
            if armed is not None and self.state.armed != armed:
                continue
            return True
        return False

    def run(self):
        self.goto(0.0, 0.0, 5.0, 3)  # stream setpoints first (OFFBOARD requirement)
        mode_cli = self.create_client(SetMode, "/target/mavros/set_mode")
        arm_cli = self.create_client(CommandBool, "/target/mavros/cmd/arming")
        if not mode_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no set_mode service")
        if not arm_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no arming service")
        for attempt in range(5):
            fut = mode_cli.call_async(
                SetMode.Request(base_mode=0, custom_mode="OFFBOARD")
            )
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8)
            ok = fut.done() and fut.result() is not None and fut.result().mode_sent
            self.get_logger().info(f"OFFBOARD sent={ok} try={attempt}")
            if ok and self.wait_state(mode="OFFBOARD", timeout=6):
                break
            time.sleep(1)
        if not self.wait_state(mode="OFFBOARD", timeout=3):
            raise RuntimeError("OFFBOARD failed after 5 tries")
        time.sleep(1)
        for attempt in range(5):
            fut = arm_cli.call_async(CommandBool.Request(value=True))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8)
            ok = fut.done() and fut.result() is not None and fut.result().success
            self.get_logger().info(f"ARM success={ok} try={attempt}")
            if self.wait_state(armed=True, timeout=6):
                break
            time.sleep(1)
        if not self.wait_state(armed=True, timeout=3):
            raise RuntimeError("arm failed after 5 tries")
        self.get_logger().info("armed, takeoff")
        # gentle climb: instant 0->5 m step causes SITL sensor storm
        # (vibration aliasing + EKF attitude resets, flight_sitl5 t=21-30).
        # Ramp 0.5 m per 1.5 s, then settle at top.
        for k in range(1, 11):
            self.goto(0.0, 0.0, 0.5 * k, 1.5)
        self.get_logger().info("top, settle")
        self.goto(0.0, 0.0, 5.0, 8)
        for x, y in ((20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.0)):
            self.goto(x, y, 5.0, 12)
        self.get_logger().info("pattern done, landing")
        fut = mode_cli.call_async(SetMode.Request(base_mode=0, custom_mode="AUTO.LAND"))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8)
        landed = self.wait_state(mode="AUTO.LAND", timeout=5)
        self.get_logger().info(f"LAND mode={landed}")
        end = time.time() + 15
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("commander done")


def main():
    rclpy.init()
    node = Commander()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
