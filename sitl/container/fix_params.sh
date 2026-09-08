#!/bin/bash
# Repair poisoned arm thresholds live (COM_ARM_EKF_*=-1 always fails:
# estimatorCheck.cpp gates on test_ratio > param). Then persist.
SV=/home/user/shared_volume
for p in COM_ARM_EKF_POS COM_ARM_EKF_VEL COM_ARM_EKF_HGT COM_ARM_EKF_YAW; do
  echo "param set $p 10.0" > $SV/px4_stdin
  sleep 1
done
echo "param save" > $SV/px4_stdin
sleep 2
echo PARAMS_FIXED
