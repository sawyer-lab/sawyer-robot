#!/bin/bash
# Build the Docker image.
# Usage:
#   ./build.sh          — real robot mode (auto-discovers IPs)
#   ./build.sh --sim    — simulation mode (uses 127.0.0.1)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SIM_MODE=false
for arg in "$@"; do
    [[ "$arg" == "--sim" ]] && SIM_MODE=true
done

if $SIM_MODE; then
    echo "Configuring for SIMULATION mode..."
    ./update_configs.sh --sim
else
    echo "Detecting network configuration..."
    ./update_configs.sh || echo "  ⚠ IP detection failed — using last known values"
fi

echo ""
echo "Building sawyer-robot Docker image..."
docker compose build robot_dev
echo "✓ Build complete"
