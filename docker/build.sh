#!/bin/bash
# Build the Docker image
# Usage: ./build.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Building sawyer-robot Docker image..."
docker compose build robot_dev
echo "✓ Build complete"
