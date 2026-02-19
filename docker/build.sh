#!/bin/bash
# Build the Docker image
# Automatically discovers robot/host IPs and updates configs before building.
# Usage: ./build.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Detecting network configuration..."
./update_configs.sh

echo ""
echo "Building sawyer-robot Docker image..."
docker compose build robot_dev
echo "✓ Build complete"
