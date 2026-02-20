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
        ├── sawyer_robot.py   # SawyerRobot — single entry point
        ├── components/       # arm, gripper, camera, head, lights
        ├── protocols/        # ZMQ transport
        └── geometry.py       # JointAngles, Pose, Position, …
```

## Requirements

- Docker (see platform notes below)
- Python ≥ 3.8 on the host machine
- Sawyer robot on the same network (for real-hardware mode)

---

## Platform setup

### Linux (Ubuntu / Debian)

```bash
# Install Docker Engine
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # re-login after this
```

### Windows

Install WSL2 with Ubuntu, then follow the exact same Linux instructions inside
the WSL2 terminal. No extra steps — Docker Engine inside WSL2 is identical to
running it on a native Linux machine.

```powershell
# In PowerShell (one-time WSL2 setup)
wsl --install          # installs Ubuntu by default, requires reboot
```

```bash
# Then open the Ubuntu terminal and run:
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # re-login after this
```

> **Important:** clone this repo inside the WSL2 filesystem (`~/sawyer-robot`),
> not on the Windows drive (`/mnt/c/…`). File permissions and line endings will
> break the scripts otherwise.

### macOS

Install [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/).
The `.sh` scripts run from any terminal.

---

## Quick start

### 1. Clone and install the client library

```bash
git clone git@github.com:sawyer-lab/sawyer-robot.git
cd sawyer-robot
pip install -e robot_client
```

### 2. Build the Docker image

```bash
./docker/build.sh
```

### 3. Start the container

```bash
# Simulation (no robot needed)
./docker/run.sh --sim

# Real robot (auto-discovers Sawyer on the network)
./docker/run.sh
```

### 4. Start the ZMQ server inside the container

```bash
./docker/exec.sh
# inside the container:
python src/custom/robot_api/servers/zmq_server.py
```

### 5. Use it in your project

```python
from robot_client import SawyerRobot

with SawyerRobot() as robot:
    joints = robot.arm.get_joints()
    joints.j0 += 0.1
    robot.arm.move(joints)

    robot.gripper.open()
    robot.gripper.close()

    image = robot.hand_camera.get_image()   # numpy BGR array

    robot.head.display_image(image)
    robot.lights.set("head_green_light", True)
```

---

## Examples

All examples are in `robot_client/examples/` and use only `SawyerRobot`.

| Script | What it does |
|--------|-------------|
| `gripper.py` | Interactive open / close / state |
| `keyboard.py` | Keyboard joint control + gripper |
| `camera.py` | Head or hand camera viewer with save |
| `head.py` | Head pan control + display test image |
| `lights.py` | Numbered toggle for all 6 lights |
| `camera_to_head_display.py` | Stream any camera to the head display |
| `hand_camera_settings.py` | Strobe / exposure / gain + frame capture |

```bash
python robot_client/examples/gripper.py
python robot_client/examples/camera_to_head_display.py
```

---

## Sim vs real mode

```bash
./docker/run.sh --sim   # ROS_IP=127.0.0.1, ROBOT_MODE=sim, no robot needed
./docker/run.sh         # auto-discovers robot IP via mDNS / arp-scan
```

The `ROBOT_MODE` env var is passed into the container. The ZMQ server reads it
and switches the gripper driver between Gazebo (sim) and ClickSmart (real).

---

## Custom project mounts

Copy `docker/docker-compose.override.example.yml` to
`docker/docker-compose.override.yml` and edit it to mount your project's source
tree into the container. This file is gitignored so it stays local to your
machine.

---

## Requirements (inside container)

- ROS Noetic
- Python 3.8
- intera_sdk (included as submodule in the workspace)

