import time
from pymavlink import mavutil

ml = mavutil.mavlink_connection("udp:127.0.0.1:14550", source_system=255)
ml.wait_heartbeat(timeout=10)
SYS = 1
for mid, hz in (
    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1),
    (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1),
):
    ml.mav.command_long_send(
        SYS,
        1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mid,
        int(1e6 / hz),
        0,
        0,
        0,
        0,
        0,
    )
end = time.time() + 10
while time.time() < end:
    m = ml.recv_match(
        type=["EKF_STATUS_REPORT", "SYS_STATUS", "EXTENDED_SYS_STATE"],
        blocking=True,
        timeout=2,
    )
    if m is None:
        continue
    t = m.get_type()
    if t == "EKF_STATUS_REPORT":
        print(
            "EKF flags:",
            hex(m.flags),
            "pos_horiz_abs:",
            bool(m.flags & 0x04),
            "vel_horiz:",
            bool(m.flags & 0x10),
            "yaw:",
            bool(m.flags & 0x80),
            "mag_test:",
            hex(m.flags >> 16 & 0xFF),
        )
    elif t == "SYS_STATUS":
        print("SYS health:", hex(m.onboard_control_sensors_health))
    else:
        print("STATE landed:", m.landed_state, "vtol:", m.vtol_state)
