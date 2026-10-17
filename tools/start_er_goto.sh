#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-only
# Start the RealSense RGB-D camera for er_goto (once) and walk the robot to a target.

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_success() {
    echo -e "${GREEN}$1${NC}"
}

print_info() {
    echo -e "${YELLOW}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

show_usage() {
    echo "Usage: $0 [--dry-run] [--debug-images] [--restart-camera] TARGET..."
    echo
    echo "Walks the robot to TARGET using Gemini Robotics-ER and the RealSense camera."
    echo "Run tools/start_robot.sh (default policy) first; after this script starts,"
    echo "press Y on the gamepad to hand control to /cmd_vel. Ctrl+C stops the robot."
    echo
    echo "  --dry-run         publish to /cmd_vel_test instead of /cmd_vel (robot not needed)"
    echo "  --debug-images    save each LOOK frame with the Gemini point to ~/er_goto_debug"
    echo "  --restart-camera  restart the er_camera_session camera before running"
    echo
    echo "Example: $0 chair"
    echo "Example: $0 --dry-run walk to the red chair"
}

wait_for_node() {
    local node_pattern=$1
    local timeout_seconds=$2
    local attempts=$((timeout_seconds * 2))
    local i

    for ((i = 0; i < attempts; ++i)); do
        if ros2 node list 2>/dev/null | grep -qE "$node_pattern"; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_topic() {
    local topic=$1
    local timeout_seconds=$2
    local attempts=$((timeout_seconds * 2))
    local i

    for ((i = 0; i < attempts; ++i)); do
        if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

verify_topic_stream() {
    local topic=$1
    local label=$2
    local attempt
    local hz_line
    local hz_output

    if ! wait_for_topic "$topic" 15; then
        print_error "$label topic missing: $topic"
        print_info "Check the camera log: screen -r $CAMERA_SESSION"
        return 1
    fi

    for ((attempt = 1; attempt <= 2; ++attempt)); do
        # ROS CLI initialization and DDS discovery can exceed four seconds on the X5.
        # Capture the complete probe so an early grep exit cannot break its output pipe.
        hz_output=$(PYTHONUNBUFFERED=1 timeout --signal=INT --kill-after=5s 15s ros2 topic hz "$topic" 2>&1)
        hz_line=$(grep -m 1 "average rate" <<< "$hz_output")
        if [ -n "$hz_line" ]; then
            print_success "$label stream OK ($hz_line)"
            return 0
        fi
    done

    print_error "$label topic exists but the stream check timed out or failed: $topic"
    if [ -n "$hz_output" ]; then
        printf '%s\n' "$hz_output"
    fi
    print_info "Check the camera log: $CAMERA_LOG (or screen -r $CAMERA_SESSION)"
    return 1
}

session_running() {
    screen -ls 2>/dev/null | grep -q "[0-9]\.$1[[:space:]]"
}

DRY_RUN=false
DEBUG_IMAGES=false
RESTART_CAMERA=0
TARGET_WORDS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --debug-images)
            DEBUG_IMAGES=true
            shift
            ;;
        --restart-camera)
            RESTART_CAMERA=1
            shift
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        -*)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
        *)
            TARGET_WORDS+=("$1")
            shift
            ;;
    esac
done

TARGET="${TARGET_WORDS[*]}"
if [ -z "$TARGET" ]; then
    print_error "Missing TARGET"
    show_usage
    exit 1
fi

cd "$(dirname "$0")"
cd ..

CAMERA_SESSION="er_camera_session"
CAMERA_CONFIG="$(pwd)/er_goto/config/realsense_d435i_rgbd.yaml"
CAMERA_LOG="$(pwd)/log/er_camera_session.log"
COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/assets/rt_fastdds_profile.xml"

if [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    print_error "DDS profile not found: $FASTRTPS_DEFAULT_PROFILES_FILE"
    exit 1
fi

# GEMINI_API_KEY from .env (see .env.example)
if [ -f .env ]; then
    set -a
    source .env
    set +a
fi
if [ -z "${GEMINI_API_KEY:-}" ]; then
    print_error "GEMINI_API_KEY is not set. Copy .env.example to .env and fill it in."
    exit 1
fi

source tools/ros_environment.sh || exit 1
setup_ros_environment || exit 1

for cmd in colcon ros2 screen; do
    if ! command -v "$cmd" &>/dev/null; then
        print_error "$cmd is not installed"
        exit 1
    fi
done
if ! python3 -c "import google.genai" &>/dev/null; then
    print_error "google-genai is not installed: pip3 install google-genai"
    exit 1
fi

print_info "Building er_goto..."
COLCON_BASE_PATHS=(er_goto)
BUILD_PACKAGES=(er_goto)
ALLOW_OVERRIDING=()
if [ -f src/camera/thirdparty/realsense-ros/realsense2_camera/package.xml ]; then
    COLCON_BASE_PATHS+=(src/camera/thirdparty/realsense-ros)
    BUILD_PACKAGES+=(realsense2_camera)
    ALLOW_OVERRIDING=(
        --allow-overriding
        realsense2_camera
        realsense2_camera_msgs
        realsense2_description
    )
fi

colcon build --symlink-install \
    --base-paths "${COLCON_BASE_PATHS[@]}" \
    --packages-up-to "${BUILD_PACKAGES[@]}" \
    "${ALLOW_OVERRIDING[@]}" || {
    print_error "er_goto build failed"
    exit 1
}
source install/setup.bash

if ! ros2 pkg prefix realsense2_camera &>/dev/null; then
    print_error "realsense2_camera not found. Check src/camera/thirdparty/realsense-ros or install the ROS 2 RealSense package"
    exit 1
fi

if session_running realsense_session; then
    print_error "realsense_session (tools/start_camera.sh, parkour config) is running and uses the same camera."
    print_info "Stop it first: screen -S realsense_session -X quit; screen -S camera_session -X quit"
    exit 1
fi

if [ "$RESTART_CAMERA" -eq 1 ]; then
    screen -S "$CAMERA_SESSION" -X quit 2>/dev/null
    sleep 1
fi

if session_running "$CAMERA_SESSION"; then
    print_info "Reusing running camera session: $CAMERA_SESSION"
else
    print_info "Starting $CAMERA_SESSION (RealSense RGB-D)..."
    print_info "RealSense config: $CAMERA_CONFIG"
    mkdir -p "$(dirname "$CAMERA_LOG")"
    screen -dmS "$CAMERA_SESSION" bash -c "source install/setup.bash; export RMW_IMPLEMENTATION='$RMW_IMPLEMENTATION'; export RMW_FASTRTPS_USE_QOS_FROM_XML='$RMW_FASTRTPS_USE_QOS_FROM_XML'; export FASTRTPS_DEFAULT_PROFILES_FILE='$FASTRTPS_DEFAULT_PROFILES_FILE'; ros2 launch realsense2_camera rs_launch.py config_file:='$CAMERA_CONFIG' 2>&1 | tee '$CAMERA_LOG'; exec bash"

    if ! wait_for_node "^/camera/camera$" 15; then
        print_error "RealSense did not start. Startup output: $CAMERA_LOG"
        print_info "Inspect: screen -r $CAMERA_SESSION"
        print_info "After fixing the error, retry with --restart-camera."
        exit 1
    fi
fi

verify_topic_stream "$COLOR_TOPIC" "RGB" || exit 1
verify_topic_stream "$DEPTH_TOPIC" "Aligned depth" || exit 1

if [ "$DRY_RUN" = true ]; then
    print_info "Dry run: commands go to /cmd_vel_test; the robot is not moved."
else
    if ! ros2 node list 2>/dev/null | grep -q "inference_node"; then
        print_error "inference_node is not running. Start it first: ./tools/start_robot.sh"
        exit 1
    fi
    print_success "----------------------------------------"
    print_success "Hold the gamepad. Start the policy as usual, then press Y to hand over to /cmd_vel."
    print_success "Y takes control back instantly, B pauses inference, Ctrl+C here stops the robot."
    print_success "----------------------------------------"
fi

print_info "Target: $TARGET"
print_info "Camera keeps running for the next command; stop it with: screen -S $CAMERA_SESSION -X quit"
ros2 launch er_goto er_goto.launch.py "target:=$TARGET" "dry_run:=$DRY_RUN" "save_debug_images:=$DEBUG_IMAGES"
