# fold_ros

[English](README.md)

4 自由度折叠模块（RB200 协议）的 ROS 2 驱动，使用轮廓位置控制，基于 [agx_motor_sdk](https://github.com/kehuanjack/agx_motor_sdk)。

> **Safety:** 默认启动会配置并使能电机，并接受控制指令。启动前请检查接线、节点 ID、标定、限位和机械空间，并准备独立的急停装置。

## 环境要求

已在 Ubuntu 22.04 + ROS 2 Humble 上测试。

## 安装

SDK 安装时会编译 C++ 扩展，需先安装构建工具：

```bash
sudo apt install build-essential cmake libasio-dev libyaml-cpp-dev python3-dev
mkdir -p ~/fold_ws/src && cd ~/fold_ws
git clone https://github.com/AgilexOpensource/fold_ros.git src/fold_ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

用系统 pip 安装 SDK 并构建：

```bash
pip3 install git+https://github.com/kehuanjack/agx_motor_sdk.git
colcon build --packages-select fold_ros
source install/setup.bash
```

或者，推荐使用虚拟环境，避免改动系统 Python。该虚拟环境仍可访问 ROS 的 Python 包：

```bash
python3 -m venv --system-site-packages ~/fold_ws/.venv
source ~/fold_ws/.venv/bin/activate
pip install git+https://github.com/kehuanjack/agx_motor_sdk.git
python3 -m colcon build --packages-select fold_ros
source install/setup.bash
```

每个新终端都要先激活虚拟环境，再执行 `source install/setup.bash`。`python3 -m colcon` 让节点使用虚拟环境中的 Python；直接用 `colcon` 会使用系统 Python，无法导入虚拟环境中的 `agx_motor_sdk`。

## 快速开始

以 1 Mbit/s 启用 `can0` 并启动驱动。仅在接口处于关闭状态且未被其他程序使用时执行 `ip link set`。

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch fold_ros fold.launch.py
```

节点为 `/fold/fold_node`，关节为 `torso_joint1` 到 `torso_joint4`，均在 `can0` 上。只检查连接、不配置、不使能、不接受指令：

```bash
ros2 launch fold_ros fold.launch.py \
  auto_configure:=false auto_enable:=false control_enabled:=false
```

在另一个终端读取反馈并发送位置目标。**指令会让关节运动。**

```bash
ros2 topic echo /fold/feedback/joint_states
ros2 topic echo /fold/feedback/motor_status
ros2 topic pub --once /fold/control/joint_states sensor_msgs/msg/JointState \
  "{name: ['torso_joint1'], position: [0.0]}"
```

关闭指令门控，再失能电机：

```bash
ros2 service call /fold/control_enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /fold/enable std_srvs/srv/SetBool '{data: false}'
```

## 接口

名称相对于节点命名空间（默认 `/fold`）。话题使用 reliable、volatile QoS，深度 10。

| 名称 | 类别 | 类型 | 说明 |
|---|---|---|---|
| `control/joint_states` | 订阅话题 | `sensor_msgs/msg/JointState` | 关节目标 |
| `feedback/joint_states` | 发布话题 | `sensor_msgs/msg/JointState` | 按 `feedback_rate` 发布的关节反馈 |
| `feedback/motor_status` | 发布话题 | `diagnostic_msgs/msg/DiagnosticArray` | 按 `status_rate` 发布，每关节一条状态 |
| `enable` | 服务 | `std_srvs/srv/SetBool` | 使能或失能电机 |
| `control_enable` | 服务 | `std_srvs/srv/SetBool` | 打开或关闭指令门控 |

指令规则：

- `name` 和 `position` 必须非空、长度相同且无重复。可以只发部分关节；未知关节会被跳过。
- 用到的值必须有限；无效消息整条拒绝。单个关节写入失败会记录日志，其他关节继续执行。
- 只使用 `position`。每个关节的第一个目标同时选择轮廓位置模式，后续目标不改变模式。通过 `enable` 服务失能后会重置这一状态。
- 忽略消息头；不检查时效，也没有应答。

单位为 rad、rad/s 和 N·m。

没有 SDK 状态的关节不出现在 `feedback/joint_states` 中。`motor_status` 字段见 [docs/reference.zh-CN.md](docs/reference.zh-CN.md#诊断)。

## 配置

- [config/fold.yaml](config/fold.yaml)：硬件模块 `torso`，包含 `torso_joint1` 到 `torso_joint4`（ID 1 到 4）、极性、限位、运动曲线和 `torque_b`。字段见 [SDK YAML 参考](https://github.com/kehuanjack/agx_motor_sdk/blob/main/YAML_CONFIG.md)。
- [config/fold_ros.yaml](config/fold_ros.yaml)：节点参数，见 [docs/reference.zh-CN.md](docs/reference.zh-CN.md#ros-参数)。

启动参数（留空则使用 `params_file` 中的值）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `fold` | 节点命名空间 |
| `params_file` | `config/fold_ros.yaml` | 节点参数文件 |
| `yaml_path` | 空 | 硬件 YAML 路径 |
| `module_key` | 空 | YAML 顶层模块名 |
| `can_channel` | 空 | 覆盖 CAN 接口 |
| `control_enabled` | 空 | 初始指令门控，`true` 或 `false` |
| `auto_configure`, `auto_enable` | 空 | 启动时配置和使能，`true` 或 `false` |
| `feedback_rate`, `status_rate` | 空 | 发布频率，Hz |
| `status_timeout` | 空 | 诊断新鲜度上限，s |

`ros2 launch fold_ros fold.launch.py --show-args` 可列出全部参数。硬件 YAML 查找和运行边界见 [docs/reference.zh-CN.md](docs/reference.zh-CN.md)。

## 常见问题

- 安装 `agx_motor_sdk` 时报 `GitHub release metadata request failed`：SDK 安装时会从 GitHub 下载协议库，匿名访问每个 IP 每小时只能请求 60 次 API。等限额重置，或把 `GITHUB_TOKEN` 设为 GitHub token 后重装。
- `No module named 'agx_motor_sdk'`：激活虚拟环境，并用 `python3 -m colcon build` 重新构建。
- `failed to open CAN: can0`：按[快速开始](#快速开始)启用接口，并用 `ip -brief link show` 检查。
- `fold YAML not found`：传入绝对路径的 `yaml_path`，或安装在 `share/fold_ros/config` 下的文件名。
- `fold configure failed` 或 `fold enable failed`：检查供电、接线和节点 ID，再运行连接检查。
- 日志显示 `External control gate is closed`：调用 `control_enable`，参数为 `{data: true}`。
- 日志显示 `set_target failed for torso_joint1`：检查该关节的供电、CAN 和使能状态。
- 日志显示 `control/joint_states ignored`：按指令规则修正消息。
- `motor_status` 条目为 `STALE`：`status_timeout` 内没有新的反馈；检查供电和 CAN。

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
