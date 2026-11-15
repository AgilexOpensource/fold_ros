# fold_ros

[中文](README.zh-CN.md)

ROS 2 driver for the 4-DOF fold module (RB200 protocol) with profile position control, built on [agx_motor_sdk](https://github.com/kehuanjack/agx_motor_sdk).

> **Safety:** Default startup configures and enables the motors and accepts commands. Check wiring, node IDs, calibration, limits and mechanical clearance first, and provide an independent emergency stop.

## Requirements

Tested on Ubuntu 22.04 with ROS 2 Humble.

## Install

The SDK compiles a C++ extension during install, so install the build tools and Python and ROS tooling first. `rosdep init` runs only on a machine where it has never run:

```bash
sudo apt install build-essential cmake libasio-dev libyaml-cpp-dev python3-dev \
  python3-pip python3-venv python3-colcon-common-extensions python3-rosdep
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
mkdir -p ~/fold_ws/src && cd ~/fold_ws
git clone https://github.com/AgilexOpensource/fold_ros.git src/fold_ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Install the SDK with system pip and build:

```bash
pip3 install git+https://github.com/kehuanjack/agx_motor_sdk.git
colcon build --packages-select fold_ros
source install/setup.bash
```

Or, recommended to keep the system Python clean, a virtual environment that still sees the ROS Python packages:

```bash
python3 -m venv --system-site-packages ~/fold_ws/.venv
source ~/fold_ws/.venv/bin/activate
pip install git+https://github.com/kehuanjack/agx_motor_sdk.git
python3 -m colcon build --packages-select fold_ros
source install/setup.bash
```

Activate the venv in every new terminal before `source install/setup.bash`. `python3 -m colcon` makes the node use the venv Python; plain `colcon` would use the system Python, which cannot import `agx_motor_sdk` from the venv.

## Quick start

Bring up `can0` at 1 Mbit/s and launch the driver. Run `ip link set` only while the interface is down and unused.

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch fold_ros fold.launch.py
```

The node is `/fold/fold_node` with joints `torso_joint1` through `torso_joint4`, all on `can0`. To check the connection without configuring, enabling or accepting commands:

```bash
ros2 launch fold_ros fold.launch.py \
  auto_configure:=false auto_enable:=false control_enabled:=false
```

In another terminal, read feedback and send a position target. **Commands move the joints.**

```bash
ros2 topic echo --once /fold/feedback/joint_states
ros2 topic echo --once /fold/feedback/motor_status
ros2 topic pub --once /fold/control/joint_states sensor_msgs/msg/JointState \
  "{name: ['torso_joint1'], position: [0.0]}"
```

Close the command gate, then disable the motors:

```bash
ros2 service call /fold/control_enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /fold/enable std_srvs/srv/SetBool '{data: false}'
```

## Interfaces

Names are relative to the node namespace (default `/fold`). Topics use reliable, volatile QoS with depth 10.

| Name | Kind | Type | Description |
|---|---|---|---|
| `control/joint_states` | Subscribed topic | `sensor_msgs/msg/JointState` | Joint targets; dropped for joints whose motor is not enabled |
| `feedback/joint_states` | Published topic | `sensor_msgs/msg/JointState` | Joint feedback at `feedback_rate` |
| `feedback/motor_status` | Published topic | `diagnostic_msgs/msg/DiagnosticArray` | One status per joint at `status_rate` |
| `enable` | Service | `std_srvs/srv/SetBool` | Enable or disable the motors |
| `control_enable` | Service | `std_srvs/srv/SetBool` | Open or close the command gate |

Command rules and units are in [docs/reference.md](docs/reference.md#commands-and-feedback); the `motor_status` fields are in [docs/reference.md](docs/reference.md#diagnostics).

## Configuration

- [config/fold.yaml](config/fold.yaml): hardware module `torso` with `torso_joint1` through `torso_joint4` (IDs 1 to 4), polarity, limits, profile and `torque_b`. See the [SDK YAML reference](https://github.com/kehuanjack/agx_motor_sdk/blob/main/YAML_CONFIG.md).
- [config/fold_ros.yaml](config/fold_ros.yaml): node parameters, listed in [docs/reference.md](docs/reference.md#ros-parameters).

Common launch arguments (empty keeps the value from `params_file`):

| Argument | Default | Description |
|---|---|---|
| `can_channel` | empty | CAN interface override |
| `control_enabled` | empty | Initial command gate, `true` or `false` |
| `auto_configure`, `auto_enable` | empty | Configure and enable at startup, `true` or `false` |

`ros2 launch fold_ros fold.launch.py --show-args` lists all arguments. The full parameter and launch argument lists, hardware YAML lookup and operating boundaries are in [docs/reference.md](docs/reference.md).

## Troubleshooting

- `GitHub release metadata request failed` while installing `agx_motor_sdk`: the SDK downloads protocol libraries from GitHub, and anonymous access allows 60 API requests per hour per IP. Wait for the limit to reset, or set `GITHUB_TOKEN` to a GitHub token and install again.
- `No module named 'agx_motor_sdk'`: activate the venv and rebuild with `python3 -m colcon build`.
- `failed to open CAN: can0`: bring the interface up as in [Quick start](#quick-start) and check `ip -brief link show`.
- Log says `External control gate is closed`: call `control_enable` with `{data: true}`.
- Log says `motor not enabled`: call `enable` with `{data: true}` first. Targets sent to a disabled RB200 make it drop in and out of operation_enabled, so they are not forwarded.

Other symptoms are in [docs/reference.md](docs/reference.md#troubleshooting).

## License

Apache-2.0, see [LICENSE](LICENSE).
