# fold_ros 参考

[English](reference.md)

`fold_node` 的参数、硬件 YAML、诊断和运行边界。入门请先看 [README](../README.zh-CN.md)。

## ROS 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `yaml_path` | 空 | 硬件 YAML；[fold_ros.yaml](../config/fold_ros.yaml) 设为 `fold.yaml` |
| `module_key` | 空 | YAML 顶层模块名；[fold_ros.yaml](../config/fold_ros.yaml) 设为 `torso` |
| `can_channel` | 空 | 覆盖 CAN 接口；[fold_ros.yaml](../config/fold_ros.yaml) 设为 `can0` |
| `auto_configure` | `true` | 启动时配置关节组 |
| `auto_enable` | `true` | 启动时使能电机 |
| `control_enabled` | `true` | 初始指令门控 |
| `feedback_rate` | `100.0` | 关节反馈频率，Hz |
| `status_rate` | `2.0` | 诊断频率，Hz |
| `status_timeout` | `2.0` | 诊断新鲜度上限，s |

- 频率和 `status_timeout` 必须有限且大于 0。
- 所有参数在启动后只读。
- `control_enabled` 保持启动值；`control_enable` 服务修改实际门控。
- 逐关节使能和失能帧之间间隔 0.1 s。

## 硬件 YAML

- 相对的 `yaml_path` 在安装后的 `share/fold_ros/config` 下查找。其他文件请用绝对路径；不支持 `package://` URL。
- `yaml_path` 为空时依次使用环境变量 `FOLD_ROS_CONFIG` 和 `fold.yaml`。
- `module_key` 为空时用文件名主干，因此 `fold.yaml` 对应模块 `fold`。[fold.yaml](../config/fold.yaml) 定义的模块是 `torso`，使用它时请保留 [fold_ros.yaml](../config/fold_ros.yaml) 中的 `module_key: torso`。
- `can_channel` 会替换模块默认接口，以及所有与之相同的关节接口。

字段含义见 [SDK YAML 参考](https://github.com/kehuanjack/agx_motor_sdk/blob/main/YAML_CONFIG.md)。本驱动不应用 `torque_limit`。

## 诊断

`feedback/motor_status` 每次发布一个 `DiagnosticArray`，每个关节一条 `DiagnosticStatus`。`name` 为关节名，`hardware_id` 为 `<can_channel>:<node_id>`，所有值均为字符串。所有关节都使用 RB200 协议，报告 CiA 402 状态字：

| 键 | 说明 |
|---|---|
| `enabled` | 是否处于 `operation_enabled` 状态 |
| `cia402_state` | 驱动器状态，如 `switch_on_disabled`、`operation_enabled` 或 `fault` |
| `ready_to_switch_on`, `switched_on`, `fault`, `voltage_enabled`, `switch_on_disabled` | 状态字第 0、1、3、4、6 位 |
| `warning`, `remote`, `target_reached`, `internal_limit_active`, `homing_complete` | 状态字第 7、9、10、11、15 位 |
| `quick_stop_active` | 驱动器处于快速停止状态 |
| `error_code`, `status_word` | 十六进制，如 `0x0000` |
| `control_mode` | 固定为 `pos` |

`fault` 位置位时为 `ERROR`，消息为 `<state>, error_code 0x....`；`warning` 位置位时为 `WARN`；否则为 `OK`，消息为驱动器状态。

时间戳缺失、非正、在未来或已过期时为 `STALE`，值为 `unknown`。新鲜指按主机挂钟时间 `0 <= age <= status_timeout`。

## 运行边界

- 关闭指令门控不会失能电机，也不会取消已发送的目标。
- 诊断过期不会停止指令，也不会清除缓存的关节反馈。
- 节点退出时关闭 SDK 连接，但不失能电机。
- `enable` 服务返回 SDK 的结果；硬件实际状态请看 `motor_status`。
- 看门狗、轨迹执行和故障恢复应由本驱动的上层负责。
