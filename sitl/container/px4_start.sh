#!/bin/bash
cd /home/user/shared_volume/PX4-Autopilot
export PX4_SIM_SPEED_FACTOR=1
export PX4_SYS_AUTOSTART=4022
export PX4_GZ_MODEL=x500_mono_cam_3d_lidar
export PX4_UXRCE_DDS_NS=target
export PX4_GZ_MODEL_POSE="71.26,-70.86,76.4"
export PX4_GZ_WORLD=taif_test4
FIFO=/home/user/shared_volume/px4_stdin
[ -p "$FIFO" ] || mkfifo "$FIFO"
# holder keeps fifo open so NSH stdin never sees EOF
tail -f /dev/null > "$FIFO" 2>/dev/null &
exec ./build/px4_sitl_default/bin/px4 -i 0 < "$FIFO"
