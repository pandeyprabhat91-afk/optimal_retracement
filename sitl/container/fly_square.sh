#!/bin/bash
# Full flight sequence: fresh retrace C4 + bag + gentle commander square.
# Usage: fly_square.sh <bagname> ; echoes FLY_DONE on success.
# Proven 2026-09-08 (flight_sitl6: 114.5 m, home 0.36 m).
source /opt/ros/humble/setup.bash
source /home/user/shared_volume/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=18
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export PYTHONPATH=/home/user/shared_volume/ros2_ws/src/retrace_nav:$PYTHONPATH
SV=/home/user/shared_volume
BAG=$1
ps aux | grep "[R]etraceNode" | awk '{print $2}' | xargs -r kill -9
rm -rf $SV/bags/$BAG
python3 -c "import rclpy; from retrace_nav.retrace_node import RetraceNode; rclpy.init(); n=RetraceNode(); rclpy.spin(n)" \
  --ros-args -p combo:=C4 -p calib_s:=3.0 > $SV/retrace.log 2>&1 &
sleep 2
ros2 bag record -o $SV/bags/$BAG /tf /tf_static /clock \
  /target/mavros/imu/data /target/mavros/imu/mag /target/mavros/altitude \
  /target/mavros/local_position/odom /target/mavros/local_position/pose \
  /target/mavros/rc/out /target/gt_path /retrace/est_path > $SV/bag.log 2>&1 &
sleep 3
python3 -c "from retrace_nav.commander import main; main()" > $SV/commander.log 2>&1
grep -a -q "commander done" $SV/commander.log && grep -a -q "LAND" $SV/commander.log
if [ $? -ne 0 ]; then echo FLY_FAIL; tail -n 5 $SV/commander.log; exit 1; fi
ps aux | grep "[b]ag record" | awk '{print $2}' | xargs -r kill -INT
sleep 4
echo FLY_DONE
