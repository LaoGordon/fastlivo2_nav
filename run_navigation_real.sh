#!/bin/bash
set -eo pipefail

WS_LOCALIZATION="$HOME/ws_localization"
WS_LIVOX="$HOME/ws_livox"

MAP_YAML="${MAP_YAML:-$WS_LOCALIZATION/src/fastlivo2_nav/quadruped_nav_bringup/maps/floor_small_test.yaml}"

ENABLE_LIVOX="${ENABLE_LIVOX:-true}"
ENABLE_REALSENSE="${ENABLE_REALSENSE:-true}"
ENABLE_FASTLIVO="${ENABLE_FASTLIVO:-true}"
ENABLE_NAV="${ENABLE_NAV:-true}"

SENSOR_DELAY="${SENSOR_DELAY:-6}"
FASTLIVO_DELAY="${FASTLIVO_DELAY:-15}"
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

source /opt/ros/humble/setup.bash
source "$WS_LIVOX/install/setup.bash"
source "$WS_LOCALIZATION/install/setup.bash"
set -u

echo "========================================"
echo "  Lite3 Real Robot Navigation Bringup"
echo "========================================"
echo "  map: $MAP_YAML"
echo "========================================"
echo

# Clean stale processes from previous runs
echo "[pre] cleaning stale processes..."
pkill -9 -f "fastlivo_mapping" 2>/dev/null || true
pkill -9 -f "fastlivo_nav_bridge" 2>/dev/null || true
pkill -9 -f "floor_mapper" 2>/dev/null || true
pkill -9 -f "realsense2_camera" 2>/dev/null || true
pkill -9 -f "livox_ros_driver2" 2>/dev/null || true
sleep 2

# ---------- [1/3] Sensors ----------
if [ "$ENABLE_LIVOX" = "true" ] || [ "$ENABLE_REALSENSE" = "true" ]; then
  echo "[1/3] starting sensors..."
  
  if [ "$ENABLE_LIVOX" = "true" ]; then
    echo "      -> Livox MID360"
    ros2 launch livox_ros_driver2 msg_MID360_launch.py &
    PIDS+=("$!")
  fi
  
  if [ "$ENABLE_REALSENSE" = "true" ]; then
    echo "      -> RealSense D435i"
    ros2 launch realsense2_camera rs_launch.py &
    PIDS+=("$!")
  fi
  
  echo "      waiting $SENSOR_DELAY s for sensor streams to stabilize..."
  sleep "$SENSOR_DELAY"
fi

# ---------- [2/3] FAST-LIVO2 ----------
if [ "$ENABLE_FASTLIVO" = "true" ]; then
  echo "[2/3] starting FAST-LIVO2 (visual-inertial odometry)..."
  ros2 launch fast_livo mapping_mid360.launch.py use_rviz:=True &
  PIDS+=("$!")
  
  echo "      waiting up to $FASTLIVO_DELAY s for FAST-LIVO2 initialization..."
  INIT_OK=0
  for i in $(seq 1 $FASTLIVO_DELAY); do
    sleep 1
    # Check if camera_init TF is being published
    if ros2 run tf2_ros tf2_echo camera_init body >/dev/null 2>&1; then
      echo "      -> FAST-LIVO2 initialized (camera_init TF detected after $i s)"
      INIT_OK=1
      break
    fi
    # Also check if at least the node is running and publishing odometry
    if ros2 topic echo /aft_mapped_to_init --once --timeout 1 >/dev/null 2>&1; then
      echo "      -> FAST-LIVO2 odometry detected after $i s"
      INIT_OK=1
      break
    fi
  done
  
  if [ $INIT_OK -eq 0 ]; then
    echo "      WARNING: FAST-LIVO2 may not have initialized yet (camera_init TF not found)."
    echo "               Continuing anyway, RViz may show 'frame not found' until it does."
  fi
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
