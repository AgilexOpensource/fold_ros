# fold_ros

ROS 2 support for the 4-DOF fold module with profile position control.

See [motor_ros](https://github.com/AgilexOpensource/motor_ros#readme) for
CAN setup, shared topic/service usage, and safety precautions.
This package installs independently; motor_ros is not required.

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

Use `can0` at 1 Mbit/s and start with the reviewed configuration:

```bash
ros2 launch fold_ros fold.launch.py
```

Use `/fold` instead of `/motor` in the motor_ros topic and service examples,
with joint names `torso_joint1` through `torso_joint4`.

## Fold Differences

**Position control only; MIT is not supported.** There is no `control_mode`
parameter. Command positions use rad; velocity/effort commands are ignored.

Both nodes use the SDK's `ArmJointGroup`; the differences are in the ROS adapter,
not a separate CAN protocol implementation.

| Item | fold_ros | motor_ros |
| --- | --- | --- |
| Control mode | Fixed position control; no `control_mode` parameter | `pos` or `mit`, selectable at runtime when supported by the hardware |
| Command fields | Uses `position` only; ignores `velocity` and `effort` | Position mode uses `position`; MIT uses position, velocity, and torque |
| Position submission | Calls each motor's `set_target`; the first successful target selects position mode, later targets preserve it; disabling resets this tracking | Calls the group interface `set_target_position` |
| YAML `torque_limit` | Not parsed or applied by the ROS node; hardware protection must be checked separately | With `auto_configure`, converts the configured torque limit to a current limit and sends it to the motor |
| Runtime dependency | Independent node; motor_ros is not required | General-purpose node also used by lift_ros and head_ros |

## Configuration

- [fold.yaml](config/fold.yaml): module `torso`, `torso_joint1` through `torso_joint4`
	(IDs 1 through 4), polarity, limits, profile settings, and torque calibration.
	Verify calibration for your hardware.
- [fold_ros.yaml](config/fold_ros.yaml): startup settings.

To change the CAN interface, update any explicit per-joint `can_channel` fields too.

## License

Apache-2.0; see `LICENSE`.
