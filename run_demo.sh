#!/usr/bin/env bash
# ==============================================================================
# TurtleBot 4 - SICNav - Webots Simulation Launcher Script
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${SCRIPT_DIR}/simulator_world/worlds/turtlebot4_crowd_world.wbt"

echo "======================================================================"
echo " Starting TurtleBot 4 - SICNav Simulation Environment"
echo "======================================================================"

# 1. Check Webots
if command -v webots >/dev/null 2>&1; then
    echo "[Info] Found Webots binary in PATH."
    WEBOTS_BIN="webots"
elif [ -d "/Applications/Webots.app" ]; then
    echo "[Info] Found Webots.app on macOS."
    WEBOTS_BIN="/Applications/Webots.app/Contents/MacOS/webots"
else
    WEBOTS_BIN=""
fi

# 2. Check ROS 2
if [ -n "$ROS_DISTRO" ]; then
    echo "[Info] ROS 2 environment detected ($ROS_DISTRO)."
else
    echo "[Warn] ROS 2 environment not sourced in current shell."
    echo "       (To run ROS 2 nodes, source /opt/ros/<distro>/setup.bash first)"
fi

echo ""
echo "Select launch mode:"
echo " 1) Run Integration Pipeline Test (Python standalone test)"
echo " 2) Launch Webots 3D Simulation World"
echo " 3) Launch ROS 2 Bridge Node"
echo " 4) Full Launch (Webots + ROS 2 Interface + SICNav)"
echo ""

MODE="${1:-1}"

case "$MODE" in
    1)
        echo "[Launch] Running integration test..."
        python3 "${SCRIPT_DIR}/test_integration.py"
        ;;
    2)
        if [ -n "$WEBOTS_BIN" ]; then
            echo "[Launch] Opening Webots world: ${WORLD_FILE}"
            "$WEBOTS_BIN" "${WORLD_FILE}"
        else
            echo "[Error] Webots not found. Please install Webots or run mode 1."
            exit 1
        fi
        ;;
    3)
        echo "[Launch] Launching SICNav ROS 2 Bridge Node..."
        python3 "${SCRIPT_DIR}/sicnav_ros2_interface/sicnav_ros2_bridge/sicnav_ros2_node.py"
        ;;
    4)
        if [ -n "$WEBOTS_BIN" ]; then
            echo "[Launch] Launching Webots in background..."
            "$WEBOTS_BIN" "${WORLD_FILE}" &
        fi
        echo "[Launch] Launching SICNav ROS 2 Bridge Node..."
        python3 "${SCRIPT_DIR}/sicnav_ros2_interface/sicnav_ros2_bridge/sicnav_ros2_node.py"
        ;;
    *)
        echo "Usage: ./run_demo.sh [1|2|3|4]"
        ;;
esac

