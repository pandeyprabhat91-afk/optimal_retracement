import time
from pymavlink import mavutil

PX4 = mavutil.mavlink.MAV_AUTOPILOT_PX4
ml = mavutil.mavlink_connection("udp:127.0.0.1:14550", source_system=255)
end = time.time() + 15
SYS = 0
while time.time() < end and not SYS:
    hb = ml.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb is None:
        continue
    if hb.autopilot == PX4 and hb.get_srcSystem():
        SYS = hb.get_srcSystem()
print("sys:", SYS)
for _ in range(5):
    ml.recv_match(blocking=True, timeout=0.3)


def px4_hb():
    end = time.time() + 4
    while time.time() < end:
        hb = ml.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb is None:
            continue
        if hb.autopilot == PX4 and hb.get_srcSystem() == SYS:
            return hb
    return None


for name, val, pt in (
    (b"COM_ARM_EKF_POS", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_EKF_VEL", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_EKF_HGT", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_EKF_YAW", 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_IMU_ACC", 99.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    (b"COM_ARM_IMU_GYR", 99.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
):
    ml.mav.param_set_send(SYS, 1, name, val, pt)
    time.sleep(0.3)
print("params set")
ml.mav.command_long_send(
    SYS, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
)
for i in range(6):
    hb = px4_hb()
    if hb is None:
        print("no px4 hb")
        continue
    print(f"custom={hb.custom_mode} armed={bool(hb.base_mode & 128)}")
    if hb.base_mode & 128:
        print("ARMED OK")
        break
    time.sleep(1)
