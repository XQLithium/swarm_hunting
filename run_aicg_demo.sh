#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

launch_in_terminal() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -lc "cd \"$WS_DIR\" && source /opt/ros/noetic/setup.bash && source \"$WS_DIR/devel/setup.bash\" && $cmd; exec bash"
}

launch_in_terminal "RViz (AICG demo)" "roslaunch ego_planner rviz.launch"
# 延迟 5 秒再启动 swarm，避免资源抢占或启动顺序问题
sleep 5
launch_in_terminal "AICG Demo (AICG)" "roslaunch ego_planner aicg_demo.launch"