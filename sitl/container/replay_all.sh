#!/bin/bash
# Replay flight bag through retrace_node per combo, capture est_path per combo.
# Usage: replay_all.sh [bagname]  (C1..C6, ~3 min each at -r 1 wall clock)
source /opt/ros/humble/setup.bash
source /home/user/shared_volume/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=18
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export PYTHONPATH=/home/user/shared_volume/ros2_ws/src/retrace_nav:$PYTHONPATH
BAG=/home/user/shared_volume/bags/${1:-flight_sitl6}
OUT=/home/user/shared_volume/replay
LOG=$OUT/replay_all.log
mkdir -p $OUT
echo "replay start $(date -u +%FT%TZ) bag=$BAG" > $LOG
# kill stale estimators: two publishers on /retrace/est_path teleport the score
ps aux | grep "[R]etraceNode" | awk '{print $2}' | xargs -r kill -9
# MAVROS MUST be dead during replay: live static IMU dilutes playback maneuvers
ps aux | grep "[m]avros_node" | grep -v defunct | awk '{print $2}' | xargs -r kill -9
ps aux | grep "[m]avros.launch" | awk '{print $2}' | xargs -r kill -9
sleep 2
for C in C1 C2 C3 C4 C5 C6; do
  echo "=== $C ===" | tee -a $LOG
  rm -rf $OUT/est_$C
  # wall-clock replay at -r 1 (NO use_sim_time, NO --clock): node dt comes from
  # steady wall time; sim-clock would freeze/jump between plays and skip cal.
  python3 -c "import rclpy; from retrace_nav.retrace_node import RetraceNode; rclpy.init(); n=RetraceNode(); rclpy.spin(n)" \
    --ros-args -p combo:=$C -p calib_s:=2.0 > $OUT/node_$C.log 2>&1 &
  NODEPID=$!
  sleep 6
  ros2 bag record -o $OUT/est_$C /retrace/est_path > $OUT/rec_$C.log 2>&1 &
  RECPID=$!
  sleep 3
  # play WITHOUT recorded est/gt paths: flight bag contains live /retrace/est_path
  # which would double-publish and teleport the score
  timeout 200 ros2 bag play -r 1 $BAG --topics /target/mavros/imu/data /target/mavros/imu/mag /target/mavros/altitude /target/mavros/local_position/odom /target/mavros/local_position/pose /target/mavros/rc/out /tf /tf_static > $OUT/play_$C.log 2>&1
  sleep 5
  kill -INT $RECPID 2>/dev/null
  kill -9 $NODEPID 2>/dev/null
  sleep 2
  echo "$C done: $(grep -c . $OUT/node_$C.log 2>/dev/null) log lines" | tee -a $LOG
  tail -n 3 $OUT/node_$C.log
done
echo ALL_DONE | tee -a $LOG
