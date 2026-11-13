# fold_ros reference

[中文](reference.zh-CN.md)

Parameters, hardware YAML, diagnostics and operating boundaries of `fold_node`. Start with the [README](../README.md).

## ROS parameters

| Parameter | Default | Description |
|---|---|---|
| `yaml_path` | empty | Hardware YAML; [fold_ros.yaml](../config/fold_ros.yaml) sets `fold.yaml` |
| `module_key` | empty | Top-level YAML module; [fold_ros.yaml](../config/fold_ros.yaml) sets `torso` |
| `can_channel` | empty | CAN interface override; [fold_ros.yaml](../config/fold_ros.yaml) sets `can0` |
| `auto_configure` | `true` | Configure the joint group at startup |
| `auto_enable` | `true` | Enable the motors at startup |
| `control_enabled` | `true` | Initial command gate |
| `feedback_rate` | `100.0` | Joint feedback rate, Hz |
| `status_rate` | `2.0` | Diagnostics rate, Hz |
| `status_timeout` | `2.0` | Diagnostics freshness bound, s |

- Rates and `status_timeout` must be finite and greater than 0.
- All parameters are read-only after startup.
- `control_enabled` keeps its startup value; the `control_enable` service changes the live gate.
- Per-joint enable and disable frames are sent 0.1 s apart.

## Hardware YAML

- A relative `yaml_path` is looked up under the installed `share/fold_ros/config`. Use an absolute path for other files; `package://` URLs are not supported.
- An empty `yaml_path` uses the `FOLD_ROS_CONFIG` environment variable, then `fold.yaml`.
- An empty `module_key` uses the file stem, so `fold.yaml` maps to module `fold`. [fold.yaml](../config/fold.yaml) defines module `torso`, so keep `module_key: torso` from [fold_ros.yaml](../config/fold_ros.yaml) when using it.
- `can_channel` replaces the module default and every joint channel equal to it.

Field meanings are in the [SDK YAML reference](https://github.com/kehuanjack/agx_motor_sdk/blob/main/YAML_CONFIG.md). `torque_limit` is not applied by this driver.

## Diagnostics

`feedback/motor_status` publishes one `DiagnosticArray` with one `DiagnosticStatus` per joint. `name` is the joint name, `hardware_id` is `<can_channel>:<node_id>` and all values are strings. All joints use the RB200 protocol and report the CiA 402 status word:

| Key | Description |
|---|---|
| `enabled` | `operation_enabled` state |
| `cia402_state` | Drive state, such as `switch_on_disabled`, `operation_enabled` or `fault` |
| `ready_to_switch_on`, `switched_on`, `fault`, `voltage_enabled`, `switch_on_disabled` | Status word bits 0, 1, 3, 4, 6 |
| `warning`, `remote`, `target_reached`, `internal_limit_active`, `homing_complete` | Status word bits 7, 9, 10, 11, 15 |
| `quick_stop_active` | Drive is in the quick stop state |
| `error_code`, `status_word` | Hex, such as `0x0000` |
| `control_mode` | Always `pos` |

The `fault` bit gives `ERROR` with message `<state>, error_code 0x....`; the `warning` bit gives `WARN`. Otherwise the status is `OK` with the drive state as message.

Missing, nonpositive, future or expired timestamps give `STALE` with values `unknown`. Fresh means `0 <= age <= status_timeout` on the host wall clock.

## Operating boundaries

- Closing the command gate does not disable the motors or cancel targets already sent.
- Stale diagnostics do not stop commands or clear cached joint feedback.
- Node exit closes the SDK connection without disabling the motors.
- The `enable` service reports the SDK result; read `motor_status` for the hardware state.
- Watchdogs, trajectory execution and fault recovery belong upstream of this driver.
