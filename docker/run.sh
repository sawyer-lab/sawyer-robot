#!/bin/bash
# Start the robot container.
# Usage:
#   ./run.sh          — real robot mode (auto-discovers IPs)
#   ./run.sh --sim    — simulation mode (uses 127.0.0.1)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

xhost +local:root 2>/dev/null || true

SIM_MODE=false
for arg in "$@"; do
    [[ "$arg" == "--sim" ]] && SIM_MODE=true
done

if $SIM_MODE; then
    echo "Configuring for SIMULATION mode..."
    ./update_configs.sh --sim
    ROBOT_MODE=sim
else
    echo "Searching for Sawyer robot on the network …"
    ./update_configs.sh
    eval "$(./find_robot.sh)"   # re-export ROBOT_IP for firewall rule below

    # Allow inbound connections from the robot
    if ! sudo -n iptables -C INPUT -s "$ROBOT_IP" -j ACCEPT 2>/dev/null; then
        sudo -n iptables -I INPUT -s "$ROBOT_IP" -j ACCEPT 2>/dev/null || \
            echo "  ⚠ Could not add firewall rule (run: sudo iptables -I INPUT -s $ROBOT_IP -j ACCEPT)"
    fi
    ROBOT_MODE=real
fi

echo ""
echo "Starting container (ROBOT_MODE=$ROBOT_MODE)..."
docker rm -f robo2025_dev 2>/dev/null || true
ROBOT_MODE=$ROBOT_MODE docker compose up -d robot_dev

echo ""
echo "✓ Container started!"
echo "  Enter: ./exec.sh"
