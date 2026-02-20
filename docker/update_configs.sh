#!/bin/bash
# Patch docker-compose.yml and Dockerfile with the correct IPs.
#
# Usage:
#   ./update_configs.sh              — discover real robot IP
#   ./update_configs.sh --sim        — patch everything to 127.0.0.1

SIM_MODE=false
for arg in "$@"; do
    [[ "$arg" == "--sim" ]] && SIM_MODE=true
done

if $SIM_MODE; then
    ROBOT_IP=127.0.0.1
    HOST_IP=127.0.0.1
    echo "Sim mode: using ROBOT_IP=$ROBOT_IP  HOST_IP=$HOST_IP"
else
    echo "Searching for Sawyer robot..."
    eval "$(./find_robot.sh)"

    if [ -z "$ROBOT_IP" ] || [ -z "$HOST_IP" ]; then
        echo "Warning: Could not detect Robot or Host IP — skipping config update."
        exit 0
    fi

    echo "Detected Robot IP: $ROBOT_IP"
    echo "Detected Host IP:  $HOST_IP"
fi

# Patch docker-compose.yml
echo "Updating docker-compose.yml..."
sed -i "s|ROS_MASTER_URI=http://[0-9.]*:11311|ROS_MASTER_URI=http://$ROBOT_IP:11311|g" docker-compose.yml
sed -i "s/ROS_IP=[0-9.]*/ROS_IP=$HOST_IP/g"             docker-compose.yml
sed -i "s/ROS_HOSTNAME=[0-9.]*/ROS_HOSTNAME=$HOST_IP/g" docker-compose.yml
sed -i "s/HOST_IP=\${HOST_IP:-[0-9.]*}/HOST_IP=\${HOST_IP:-$HOST_IP}/g" docker-compose.yml
sed -i "s/ROBOT_IP=\${ROBOT_IP:-[0-9.]*}/ROBOT_IP=\${ROBOT_IP:-$ROBOT_IP}/g" docker-compose.yml
sed -i "/021607CP00070\.local:/c\      - 021607CP00070.local:$ROBOT_IP" docker-compose.yml

# Patch Dockerfile
echo "Updating Dockerfile..."
sed -i "s/your_ip=\"[0-9.]*\"/your_ip=\"$HOST_IP\"/g"           Dockerfile
sed -i "s/robot_hostname=\"[0-9.]*\"/robot_hostname=\"$ROBOT_IP\"/g" Dockerfile

echo "✓ Config updated  (ROBOT_IP=$ROBOT_IP  HOST_IP=$HOST_IP)"
