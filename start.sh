#!/usr/bin/env bash

# Launch Automate first, then launch Navigation after a delay.
# Usage: ./start.sh [delay_seconds]

set -e

DELAY_SECONDS="${1:-5}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Prefer Humble by default. Fall back to ROS_DISTRO when provided.
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
	source "/opt/ros/humble/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
	source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
	source "${WORKSPACE_ROOT}/install/setup.bash"
fi

echo "Launching Automate..."
ros2 launch Automate launch.py &
AUTOMATE_PID=$!

cleanup() {
	echo
	echo "Stopping launched processes..."
	if kill -0 "${AUTOMATE_PID}" 2>/dev/null; then
		kill "${AUTOMATE_PID}" 2>/dev/null || true
	fi
}

trap cleanup INT TERM EXIT

echo "Waiting ${DELAY_SECONDS}s before launching Navigation..."
sleep "${DELAY_SECONDS}"

echo "Launching Navigation..."
ros2 launch Navigation launch.py
