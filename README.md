# fold_ros

ROS 2 driver for the 4-DOF fold module (RB200 protocol) with profile position
control.

> **Safety:** Default startup configures and enables the motors and accepts
> commands. Check wiring, node IDs, calibration, limits and mechanical clearance
> first, and provide an independent emergency stop.

## Installation

Use a terminal with your ROS 2 environment active, colcon, and Python pip.

```bash
python3 -m pip install git+https://github.com/kehuanjack/agx_motor_sdk.git
mkdir -p ~/fold_ws/src
cd ~/fold_ws
git clone https://github.com/AgilexOpensource/fold_ros.git src/fold_ros
colcon build --packages-select fold_ros
source install/setup.bash
```

## Launch

The default configuration uses `can0` at 1 Mbit/s. Activate it with the
interface down; do not reconfigure an adapter in use.

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch fold_ros fold.launch.py
```

The node is `/fold/fold_node` with joints `torso_joint1` through `torso_joint4`. For a connection check
without configure, enable or external commands:

```bash
ros2 launch fold_ros fold.launch.py \
  auto_configure:=false auto_enable:=false control_enabled:=false
```

## Control and Feedback

Run commands in another terminal with ROS 2 and the workspace loaded.
**Commands can move the joints.** The examples command `torso_joint1` to 0.

```bash
ros2 topic echo /fold/feedback/joint_states
ros2 topic echo /fold/feedback/motor_status
ros2 topic pub --once /fold/control/joint_states sensor_msgs/msg/JointState \
  "{name: ['torso_joint1'], position: [0.0]}"
```

`enable` switches motor enable; `control_enable` opens or closes the software
command gate:

```bash
ros2 service call /fold/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /fold/control_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /fold/control_enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /fold/enable std_srvs/srv/SetBool '{data: false}'
```

Names below are relative to the namespace (default `/fold`). Topics use
reliable, volatile QoS with depth 10.

| Direction | Name | Type |
|---|---|---|
| Subscribe | `control/joint_states` | `sensor_msgs/JointState` |
| Publish | `feedback/joint_states` | `sensor_msgs/JointState` |
| Publish | `feedback/motor_status` | `diagnostic_msgs/DiagnosticArray` |
| Service | `enable` | `std_srvs/SetBool` |
| Service | `control_enable` | `std_srvs/SetBool` |

Command rules:

- `name` and `position` must be nonempty, equal length, with no duplicate names.
  Subsets are allowed; unknown joints are skipped.
- Only `position` is used; velocity/effort are ignored.
- Used values must be finite; an invalid packet is rejected before sending.
  Writes are per joint; a failed write is logged and other joints continue.
- Command headers are ignored; there is no age check or acknowledgment.

Joint feedback is published from cached SDK snapshots at `feedback_rate`.
Missing joints are omitted.

## Fold Behavior

**Position control only; MIT is not supported.** There is no `control_mode`
parameter. Command positions use rad. The first target per joint selects
position mode, later targets keep the mode unchanged, and disabling resets this
tracking. YAML `torque_limit` is not applied; check hardware protection
separately.

## Configuration

- [fold.yaml](config/fold.yaml): module `torso`, `torso_joint1` through
  `torso_joint4` (IDs 1 through 4), polarity, limits, profile settings and `torque_b`.
- [fold_ros.yaml](config/fold_ros.yaml): startup parameters, including
  `module_key: torso`.

To change the CAN interface, update any explicit per-joint `can_channel` fields too.

### Hardware YAML

Relative `yaml_path` values resolve under the installed `share/fold_ros/config`;
use absolute paths for external files (`package://` is not supported). An empty
`yaml_path` checks `FOLD_ROS_CONFIG`, then `fold.yaml`. For `fold.yaml` the module key defaults to `torso`. See the
[SDK YAML reference](https://github.com/kehuanjack/agx_motor_sdk/blob/main/YAML_CONFIG.md).

### ROS parameters

| Parameter | Node default | Meaning |
|---|---|---|
| `yaml_path` | empty | Hardware YAML; `fold_ros.yaml` sets `fold.yaml` |
| `module_key` | empty | Top-level YAML module; inferred from the file stem when empty |
| `can_channel` | empty | Override the YAML CAN interface |
| `auto_configure` | `true` | Configure the joint group at startup |
| `auto_enable` | `true` | Enable motors at startup |
| `control_enabled` | `true` | Initial software command gate |
| `feedback_rate` | `100.0` | Joint feedback rate, Hz |
| `status_rate` | `2.0` | Diagnostics rate, Hz |
| `status_timeout` | `2.0` | Low-speed feedback freshness bound, s |

Rates and timeout must be finite and positive. All parameters are read-only after startup.
Enable/disable frames are always sent 0.1 s apart per joint.
The gate parameter keeps its startup value; `control_enable` changes the live gate.
Launch accepts these overrides plus `namespace` and `params_file`; empty values
keep the parameter file. List them with
`ros2 launch fold_ros fold.launch.py --show-args`.

### Diagnostics

`/fold/feedback/motor_status` is one `DiagnosticArray` per publish with one
`DiagnosticStatus` per joint: `name` is the joint name and `hardware_id` is
`<can_channel>:<node_id>`. Values are strings.

RB200 joints publish the CiA 402 status word decoded by bit and by drive state:

| Fields | Meaning |
|---|---|
| `enabled` | Operation-enabled bit (bit 2) |
| `cia402_state` | `not_ready_to_switch_on`, `switch_on_disabled`, `ready_to_switch_on`, `switched_on`, `operation_enabled`, `quick_stop_active`, `fault_reaction_active` or `fault` |
| `ready_to_switch_on`, `switched_on`, `fault`, `voltage_enabled`, `switch_on_disabled`, `warning`, `remote`, `target_reached`, `internal_limit_active`, `homing_complete` | Status word bits 0, 1, 3, 4, 6, 7, 9, 10, 11, 15 |
| `quick_stop_active` | `true` in the quick-stop-active state |
| `error_code` | Raw RB200 error code, hex (`0x8130`) |
| `status_word` | Raw status word, hex (`0x1637`) |
| `control_mode` | `pos` |

The fault bit gives `ERROR` with the error code in the message; the warning
bit gives `WARN`; otherwise the message is the drive state.

Fresh, fault-free status is `OK`, even when disabled. Missing, nonpositive,
future or expired timestamps give `STALE` with values `unknown`. Freshness is
`0 <= age <= status_timeout` on host wall-clock time.

### Operating boundaries

- Closing the command gate does not disable motors or cancel sent targets.
- Stale diagnostics do not stop commands or expire cached joint feedback.
- Node exit closes the SDK connection without explicitly disabling motors.
- The enable service reports the SDK result; check diagnostics for hardware state.
- Provide watchdogs, trajectory execution and recovery upstream.

## License

Apache-2.0; see `LICENSE`.
