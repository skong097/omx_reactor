#!/bin/bash
# 단일 터미널 데모. Ctrl+C 한 번에 자식 다 종료.
# 옵션:
#   --camera=v4l2|file|external|gazebo   (default v4l2)
#   --video-device=/dev/videoN           (camera=v4l2 인 경우, default /dev/video0)
#   --file-path=/abs/path/to/clip.mp4    (camera=file 인 경우)
#   --robot=sim|real                      (default sim) — real 은 OMX follower 직접 구동
#   --port=/dev/...                       (real 모드 follower OpenRB-150 serial port)
#   --no-browser                          브라우저 자동 open 생략
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"

CAMERA=v4l2
VIDEO_DEVICE=""
FILE_PATH=""
ROBOT=sim
PORT=""
BROWSER=1
for arg in "$@"; do
    case "$arg" in
        --camera=*)        CAMERA="${arg#*=}" ;;
        --video-device=*)  VIDEO_DEVICE="${arg#*=}" ;;
        --file-path=*)     FILE_PATH="${arg#*=}" ;;
        --robot=*)         ROBOT="${arg#*=}" ;;
        --port=*)          PORT="${arg#*=}" ;;
        --no-browser)      BROWSER=0 ;;
        -h|--help)         sed -n '2,/^$/p' "$0" | sed 's/^# *//' ; exit 0 ;;
        *) echo "알 수 없는 옵션: $arg" >&2; exit 1 ;;
    esac
done

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "★ install/setup.bash 없음 — 먼저 'colcon build'" >&2; exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
# robot_arm install (open_manipulator) 도 source — 사용자 환경에 ros-jazzy-open-manipulator-* 가 apt 설치돼 있으면 불필요
if [ -f "$HOME/robot_arm/install/setup.bash" ]; then
    source "$HOME/robot_arm/install/setup.bash"
fi
set -u

# 사전 잔존 청소
bash "$SCRIPT_DIR/stop_demo.sh" >/dev/null 2>&1 || true

# 브라우저 백그라운드 open (3초 후)
if [ "$BROWSER" = 1 ]; then
    ( sleep 3 && xdg-open http://localhost:8800/ >/dev/null 2>&1 ) &
fi

LAUNCH_ARGS="camera:=$CAMERA robot:=$ROBOT"
[ -n "$VIDEO_DEVICE" ] && LAUNCH_ARGS="$LAUNCH_ARGS video_device:=$VIDEO_DEVICE"
[ -n "$FILE_PATH" ] && LAUNCH_ARGS="$LAUNCH_ARGS file_path:=$FILE_PATH"
[ -n "$PORT" ] && LAUNCH_ARGS="$LAUNCH_ARGS port_name:=$PORT"

# foreground — Ctrl+C 한 번에 자식 다 죽음
exec ros2 launch omx_reactor demo.launch.py $LAUNCH_ARGS
