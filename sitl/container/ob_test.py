import time
from pymavlink import mavutil

ml = mavutil.mavlink_connection("udp:127.0.0.1:14550", source_system=255)
end = time.time() + 15
SYS = 0
while time.time() < end and not SYS:
    hb = ml.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb is None:
        continue
    if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 and hb.get_srcSystem():
        SYS = hb.get_srcSystem()
print("sys:", SYS)
t0 = time.monotonic()
for _ in range(40):  # 4 s stream first
    ml.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) % 2**32,
        SYS,
        1,
        1,
        0b110111111000,
        0.0,
        0.0,
        -5.0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    time.sleep(0.1)
ml.mav.command_long_send(
    SYS, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 5, 0, 0, 0, 0, 0
)
for i in range(10):
    ml.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) % 2**32,
        SYS,
        1,
        1,
        0b110111111000,
        0.0,
        0.0,
        -5.0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    time.sleep(0.1)
    hb = ml.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    if hb and hb.get_srcSystem() == SYS:
        print(f"t+{i}: custom={hb.custom_mode} armed={bool(hb.base_mode & 128)}")
