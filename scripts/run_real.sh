#!/bin/bash
# omx_real.launch.py wrapper — env (cyclone DDS) 잡고 startup log 저장.
# 옵션:
#   --port=/dev/...      follower OpenRB-150 serial port
#   --no-init-position   init_position 비활성화
#   --prefix=<str>       joint/link prefix
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/omx_real_$(date +%Y%m%d_%H%M%S).log"

PORT=""
INIT_POSITION="true"
PREFIX=""
for arg in "$@"; do
    case "$arg" in
        --port=*)           PORT="${arg#*=}" ;;
        --no-init-position) INIT_POSITION="false" ;;
        --prefix=*)         PREFIX="${arg#*=}" ;;
        -h|--help)          sed -n '2,/^$/p' "$0" | sed 's/^# *//' ; exit 0 ;;
        *) echo "알 수 없는 옵션: $arg" >&2; exit 1 ;;
    esac
done

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "★ install/setup.bash 없음 — 먼저 'colcon build'" >&2; exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
if [ -f "$HOME/robot_arm/install/setup.bash" ]; then
    source "$HOME/robot_arm/install/setup.bash"
fi
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

LAUNCH_ARGS="init_position:=$INIT_POSITION"
[ -n "$PORT" ]   && LAUNCH_ARGS="$LAUNCH_ARGS port_name:=$PORT"
[ -n "$PREFIX" ] && LAUNCH_ARGS="$LAUNCH_ARGS prefix:=$PREFIX"

echo "=== omx_real launch ==="
echo "  log:        $LOG_FILE"
echo "  RMW:        $RMW_IMPLEMENTATION"
echo "  args:       $LAUNCH_ARGS"
echo "======================="

# stdbuf -oL -eL : line-buffer so log 가 실시간 flush 됨
exec stdbuf -oL -eL ros2 launch omx_reactor omx_real.launch.py $LAUNCH_ARGS 2>&1 | tee "$LOG_FILE"
