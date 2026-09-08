import time
from pymavlink import mavutil

ml = mavutil.mavlink_connection("udp:127.0.0.1:14550", source_system=255)
ml.wait_heartbeat(timeout=10)
SYS = 1
ml.mav.param_request_read_send(SYS, 1, b"COM_ARM_WO_GPS", -1)
end = time.time() + 6
while time.time() < end:
    m = ml.recv_match(type=["PARAM_VALUE", "STATUSTEXT"], blocking=True, timeout=2)
    if m is None:
        continue
    if m.get_type() == "PARAM_VALUE":
        print("PARAM", m.param_id, m.param_value)
        break
    print("STATUSTEXT:", m.text)

ml.mav.command_long_send(
    SYS, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
)
end = time.time() + 8
while time.time() < end:
    m = ml.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=2)
    if m is None:
        continue
    print(m.get_type(), getattr(m, "result", None), getattr(m, "text", ""))
