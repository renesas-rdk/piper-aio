#!/usr/bin/env bash
# Usage: ./start_mode0.sh [dual|single] [extra launch args, e.g. can_port:=can0]
ROBOT="${1:-dual}"
ros2 launch piper start_leader_follower.launch.py robot:="$ROBOT" mode:=0 auto_enable:=false "${@:2}"
