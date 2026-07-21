#!/usr/bin/env bash
set -euo pipefail

# One-window tmux launcher for pose_fusion online integration tests.
#
# Usage:
#   source /opt/ros/noetic/setup.bash
#   ./src/localization/pose_fusion/scripts/pose_fusion_tmux_test.sh
#
# Optional environment overrides:
#   SESSION=pose_fusion_test
#   WS=/home/hhy/2004ros/DeepLumin
#   BAG=/home/hhy/2004ros/data/2026-07-14-11-17-21.bag
#   PCD_PATH=/home/hhy/2004ros/relocalization_ws/global_map_manual_imu.pcd
#   BAG_RATE=1.0
#   BAG_DELAY=3.0
#   RVIZ=true

SESSION="${SESSION:-pose_fusion_test}"
WS="${WS:-/home/hhy/2004ros/DeepLumin}"
BAG="${BAG:-/home/hhy/2004ros/data/2026-07-14-11-17-21.bag}"
PCD_PATH="${PCD_PATH:-/home/hhy/2004ros/relocalization_ws/global_map_manual_imu.pcd}"
BAG_RATE="${BAG_RATE:-1.0}"
BAG_DELAY="${BAG_DELAY:-3.0}"
RVIZ="${RVIZ:-true}"

DR_CONFIG="${DR_CONFIG:-$WS/src/localization/dr_odometry/config/dr_odometry.yaml}"
SLAM_CONFIG="${SLAM_CONFIG:-$WS/src/localization/slam_odometry/config/slam_odometry_ysw.yaml}"
MANAGER_CONFIG="${MANAGER_CONFIG:-$WS/src/localization/localization_manager/config/localization_manager.yaml}"
POSE_FUSION_CONFIG="${POSE_FUSION_CONFIG:-$WS/src/localization/pose_fusion/config/pose_fusion.yaml}"
GLOBAL_MATCHER_CONFIG="${GLOBAL_MATCHER_CONFIG:-$WS/src/localization/global_matcher/config/global_matcher.yaml}"
SENSOR_FRAME_ADAPTER_CONFIG="${SENSOR_FRAME_ADAPTER_CONFIG:-$WS/src/localization/sensor_frame_adapter/config/sensor_frame_adapter.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/src/localization/pose_fusion/rviz/pose_fusion_online_test.rviz}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "[pose_fusion_tmux_test] tmux is not installed. Install tmux or run nodes manually." >&2
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[pose_fusion_tmux_test] tmux session '$SESSION' already exists."
  echo "Attach with: tmux attach -t $SESSION"
  echo "Or stop it with: tmux kill-session -t $SESSION"
  exit 1
fi

if [[ ! -f "$BAG" ]]; then
  echo "[pose_fusion_tmux_test] bag not found: $BAG" >&2
  exit 1
fi

if [[ ! -f "$PCD_PATH" ]]; then
  echo "[pose_fusion_tmux_test] pcd map not found: $PCD_PATH" >&2
  exit 1
fi

base_cmd="source /opt/ros/noetic/setup.bash; source '$WS/devel/setup.bash'; cd '$WS'"

run_cmd() {
  local pane="$1"
  local title="$2"
  local cmd="$3"
  tmux select-pane -t "$SESSION:0.$pane" -T "$title"
  tmux send-keys -t "$SESSION:0.$pane" "bash -lc \"$base_cmd; $cmd\"" C-m
}

tmux new-session -d -s "$SESSION" -n online
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format "#{pane_index}: #{pane_title}" >/dev/null

# Build a single-window tiled layout.
for _ in 1 2 3 4 5 6 7 8 9; do
  tmux split-window -t "$SESSION:0"
  tmux select-layout -t "$SESSION:0" tiled >/dev/null
done

run_cmd 0 "roscore" "roscore"
sleep 1

run_cmd 1 "params" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam set /use_sim_time true; echo '[params] /use_sim_time=true'; sleep infinity"

run_cmd 2 "dr_odometry" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam load '$DR_CONFIG' /dr_odometry; rosparam set /dr_odometry/dr/publish_tf false; rosrun dr_odometry dr_odometry_node __name:=dr_odometry"

run_cmd 3 "sensor_adapter" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam load '$SENSOR_FRAME_ADAPTER_CONFIG' /sensor_frame_adapter; rosrun sensor_frame_adapter sensor_frame_adapter_node __name:=sensor_frame_adapter"

run_cmd 4 "slam_odometry" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam load '$SLAM_CONFIG' /slam_odometry; rosparam set /slam_odometry/slam/publish_tf false; rosrun slam_odometry slam_odometry_node __name:=slam_odometry"

run_cmd 5 "manager" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam load '$MANAGER_CONFIG' /localization_manager; rosparam set /localization_manager/initial_pose_completion/pcd_path '$PCD_PATH'; rosrun localization_manager localization_manager_node __name:=localization_manager"

run_cmd 6 "pose_fusion" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam load '$POSE_FUSION_CONFIG' /pose_fusion; rosparam set /pose_fusion/runtime/publish_tf true; rosparam set /pose_fusion/runtime/publish_map_to_odom_tf true; rosparam set /pose_fusion/runtime/publish_odom_to_base_tf true; rosparam set /pose_fusion/topics/initial_pose /localization/manager/initial_pose; rosparam set /pose_fusion/initialization/operation_mode production; rosparam set /pose_fusion/initialization/allow_dr_initialization false; rosparam set /pose_fusion/initialization/allow_slam_initialization false; rosrun pose_fusion pose_fusion_node __name:=pose_fusion"

run_cmd 7 "global_matcher" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam load '$GLOBAL_MATCHER_CONFIG' /global_matcher; rosparam set /global_matcher/map/pcd_path '$PCD_PATH'; rosparam set /global_matcher/runtime/viz_test_mode true; rosparam set /global_matcher/topics/source_cloud /localization/points; rosparam set /global_matcher/runtime/source_cloud_is_odom_frame false; rosparam set /global_matcher/runtime/source_cloud_frame lidar; rosparam set /global_matcher/topics/initial_pose /localization/fused_odom; rosparam set /global_matcher/topics/source_odom /localization/slam_odom; rosrun global_matcher global_matcher_node __name:=global_matcher"

run_cmd 8 "path_compare" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; rosparam set /pose_fusion_path_compare/map_frame map; rosparam set /pose_fusion_path_compare/enable_dr true; rosparam set /pose_fusion_path_compare/enable_slam true; rosparam set /pose_fusion_path_compare/enable_fused true; rosrun pose_fusion pose_fusion_path_compare_node __name:=pose_fusion_path_compare"

if [[ "$RVIZ" == "true" ]]; then
  run_cmd 9 "rviz_bag" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; (rviz -d '$RVIZ_CONFIG' &); sleep '$BAG_DELAY'; rosbag play --clock --delay=0 -r '$BAG_RATE' '$BAG'"
else
  run_cmd 9 "rosbag" "until rostopic list >/dev/null 2>&1; do sleep 0.2; done; sleep '$BAG_DELAY'; rosbag play --clock --delay=0 -r '$BAG_RATE' '$BAG'"
fi

tmux select-layout -t "$SESSION:0" tiled >/dev/null
tmux attach -t "$SESSION"
