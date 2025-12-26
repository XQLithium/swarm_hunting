#!/usr/bin/env bash
# Quick launcher: open two gnome-terminal windows to run README demo
# 1) rviz.launch
# 2) swarm.launch

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

launch_in_terminal() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -lc "cd \"$WS_DIR\" && source /opt/ros/noetic/setup.bash && source \"$WS_DIR/devel/setup.bash\" && $cmd; exec bash"
}

launch_in_terminal "RViz (README demo)" "roslaunch ego_planner rviz.launch"
# 延迟 5 秒再启动 swarm，避免资源抢占或启动顺序问题
sleep 5
launch_in_terminal "Hunting Demo (README)" "roslaunch ego_planner swarm_circle_demo.launch"
