#!/bin/bash
# Single preflight verdict line for the loop to parse.
SV=/home/user/shared_volume
echo "commander check" > $SV/px4_stdin
sleep 4
tail -n 5 $SV/px4.log | grep -a "Preflight check" | tail -n 1
