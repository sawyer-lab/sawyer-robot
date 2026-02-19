# sawyer-robot

Platform package for Rethink Robotics Sawyer — reusable across projects.

## Structure

```
sawyer-robot/
├── docker/          # Dockerfile, docker-compose, helper scripts
├── ros/             # ROS server code (runs inside the container)
│   └── robot_api/
│       ├── hardware/   # Camera, Gripper (ClickSmart), Head, Lights, Robot
│       └── servers/    # ZMQ server
└── robot_client/    # Host-side Python package (pip installable)
    └── robot_client/
        ├── robot.py, gripper.py, camera.py, head.py, lights.py …
        └── protocols/  # ZMQ, HTTP, WebSocket transports
```

## Quick start

### 1. Install the client library (host machine)

```bash
pip install -e /path/to/sawyer-robot/robot_client
```

### 2. Start the robot container

```bash
cd /path/to/sawyer-robot/docker
./run.sh          # start container
./exec.sh         # open a shell inside
```

### 3. Start the ZMQ server inside the container

```bash
python ros/robot_api/servers/zmq_server.py
```

### 4. Use it in your project

```python
from robot_client import RobotClient, GripperClient, CameraClient

robot   = RobotClient()
gripper = GripperClient()

robot.move_to_joints([0, 0, 0, 0, 0, 0, 0])
gripper.close()
```

## Robot server modes

The ZMQ server accepts a `--mode` argument:

```bash
python zmq_server.py           # real hardware (default)
python zmq_server.py --sim     # Gazebo simulation
```

## Requirements

- ROS Noetic (provided by the Docker image)
- Python ≥ 3.8 on the host
- Sawyer robot at a reachable IP (set `ROS_MASTER_URI` in docker-compose)
