#!/bin/bash
set -eo pipefail

# ============================================
# 实物机器人导航一键启动脚本 (RDKS100P + Lite3)
# ============================================

WS_LOCALIZATION="$HOME/ws_localization"
WS_LIVOX="$HOME/ws_livox"

# FAST-LIVO2 实物配置文件
FASTLIVO_LIDAR_CONFIG="${FASTLIVO_LIDAR_CONFIG:-$WS_LOCALIZATION/src/FAST-LIVO2/config/mid360.yaml}"
FASTLIVO_CAMERA_CONFIG="${FASTLIVO_CAMERA_CONFIG:-$WS_LOCALIZATION/src/FAST-LIVO2/config/camera_d435i.yaml}"

# 默认地图
DEFAULT_MAP="$WS_LOCALIZATION/src/fastlivo2_nav/quadruped_nav_bringup/maps/floor_small_test.yaml"
MAP_YAML="${MAP_YAML:-$DEFAULT_MAP}"

# 各模块开关
ENABLE_LIVOX="${ENABLE_LIVOX:-true}"
ENABLE_REALSENSE="${ENABLE_REALSENSE:-true}"
ENABLE_FASTLIVO="${ENABLE_FASTLIVO:-true}"
ENABLE_NAV="${ENABLE_NAV:-true}"

# 启动延迟（秒）
SENSOR_DELAY="${SENSOR_DELAY:-4}"
FASTLIVO_DELAY="${FASTLIVO_DELAY:-10}"
NAV_DELAY="${NAV_DELAY:-5}"

PIDS=()

cleanup() {
  local exit_code=$?
  if [ ${#PIDS[@]} -gt 0 ]; then
    echo "[cleanup] stopping all launched processes..."
    for pid in "${PIDS[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
      kill -9 "$pid" 2>/dev/null || true
    done
  fi
  exit $exit_code
}
trap cleanup INT TERM EXIT

# 加载 ROS2 环境（注意顺序：系统 → Livox驱动 → 导航栈）
source /opt/ros/humble/setup.bash
source "$WS_LIVOX/install/setup.bash"
source "$WS_LOCALIZATION/install/setup.bash"
set -u

echo "========================================"
echo "  Lite3 Real Robot Navigation Bringup"
echo "========================================"
echo "  map:        $MAP_YAML"
echo "  lidar_cfg:  $FASTLIVO_LIDAR_CONFIG"
echo "  camera_cfg: $FASTLIVO_CAMERA_CONFIG"
echo "========================================"
echo

# ---------- [1/3] Sensors (Livox + RealSense并行) ----------
if [ "$ENABLE_LIVOX" = "true" ] || [ "$ENABLE_REALSENSE" = "true" ]; then
  echo "[1/3] starting sensors..."

  if [ "$ENABLE_LIVOX" = "true" ]; then
    echo "      -> Livox MID360"
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py &
    PIDS+=("$!")
  fi

  if [ "$ENABLE_REALSENSE" = "true" ]; then
    echo "      -> RealSense D435i"
    ros2 launch realsense2_camera rs_launch.py &
    PIDS+=("$!")
  fi

  sleep "$SENSOR_DELAY"
fi

# ---------- [2/3] FAST-LIVO2 ----------
if [ "$ENABLE_FASTLIVO" = "true" ]; then
  echo "[2/3] starting FAST-LIVO2 (visual-inertial odometry)..."
  ros2 run fast_livo fastlivo_mapping \
    "$FASTLIVO_LIDAR_CONFIG" \
    "$FASTLIVO_CAMERA_CONFIG" &
  PIDS+=("$!")
  sleep "$FASTLIVO_DELAY"
fi

# ---------- [3/3] Navigation Stack ----------
if [ "$ENABLE_NAV" = "true" ]; then
  echo "[3/3] starting navigation stack (bridge + floor_mapper + Nav2)..."
  ros2 launch quadruped_nav_bringup navigation_main.launch.py \
    map_yaml:="$MAP_YAML" &
  PIDS+=("$!")
  sleep "$NAV_DELAY"
fi

echo
echo "All navigation modules started."
echo "Press Ctrl-C in this terminal to stop the full stack."
echo

wait
