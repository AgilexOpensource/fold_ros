#!/usr/bin/env python3
"""ROS 2 node bridging agx_motor_sdk ArmJointGroup for fold (arm protocol, position only)."""

import math
import os
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from agx_motor_sdk.mechanism import ArmJointGroup

PACKAGE_NAME = "fold_ros"
DEFAULT_FOLD_YAML = "fold.yaml"
DEFAULT_MODULE_KEY = "fold"
TARGET_MODE_POSITION = 1
TARGET_MODE_UNCHANGED = 0xFF


def package_config_dir() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME)) / "config"


def resolve_yaml_path(yaml_path: str) -> Path:
    """Resolve fold YAML: empty → fold.yaml; relative → share/config/."""
    raw = yaml_path.strip()
    if not raw:
        raw = os.environ.get("FOLD_ROS_CONFIG", "").strip() or DEFAULT_FOLD_YAML

    path = Path(raw)
    if path.is_absolute():
        if not path.is_file():
            raise FileNotFoundError("fold YAML not found: %s" % path)
        return path

    share_config = package_config_dir()
    candidates = [
        share_config / path,
        share_config / path.name,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError(
        "fold YAML not found: %s (searched under %s)" % (raw, share_config)
    )


def resolve_module_key(module_key: str, config_path: Path) -> str:
    """Use explicit module_key, else infer from YAML filename stem."""
    key = str(module_key).strip()
    if key:
        return key
    stem = config_path.stem
    if stem == "fold":
        return DEFAULT_MODULE_KEY
    if stem:
        return stem
    raise ValueError("module_key is empty and cannot be inferred from yaml_path")


class FoldRosNode(Node):
    """Publish joint feedback and accept JointState position commands."""

    def __init__(self) -> None:
        super().__init__("fold_node")

        startup_parameters = {
            "yaml_path": "",
            "module_key": "",
            "can_channel": "",
            "auto_configure": True,
            "auto_enable": True,
            "feedback_rate": 100.0,
            "status_rate": 2.0,
            "status_timeout": 2.0,
            "control_enabled": True,
        }
        for name, default in startup_parameters.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))

        yaml_path = str(self.get_parameter("yaml_path").value).strip()
        can_channel = str(self.get_parameter("can_channel").value).strip() or None
        self._auto_configure = bool(self.get_parameter("auto_configure").value)
        self._auto_enable = bool(self.get_parameter("auto_enable").value)
        feedback_rate = float(self.get_parameter("feedback_rate").value)
        if not math.isfinite(feedback_rate) or feedback_rate <= 0.0:
            raise ValueError("feedback_rate must be finite and > 0")
        status_rate = float(self.get_parameter("status_rate").value)
        self._status_timeout = float(self.get_parameter("status_timeout").value)
        if not math.isfinite(status_rate) or status_rate <= 0.0:
            raise ValueError("status_rate must be finite and > 0")
        if not math.isfinite(self._status_timeout) or self._status_timeout <= 0.0:
            raise ValueError("status_timeout must be finite and > 0")
        self._control_enabled = bool(self.get_parameter("control_enabled").value)
        self._control_gate_block_logged = False

        config_path = resolve_yaml_path(yaml_path)
        module_key = resolve_module_key(
            str(self.get_parameter("module_key").value), config_path
        )
        self._group = ArmJointGroup.from_yaml(str(config_path), module_key, can_channel)

        self._joint_by_name: Dict[str, int] = {
            name: self._group.index_of(name) for name in self._group.joint_names
        }
        self._joint_names = list(self._group.joint_names)
        self._position_mode_sent: Dict[str, bool] = {
            name: False for name in self._joint_names
        }

        try:
            if not self._group.connect():
                raise RuntimeError("failed to open CAN: %s" % self._group.can_channel)
            if self._auto_configure and not self._group.configure():
                raise RuntimeError("fold configure failed")
            if self._auto_enable and not self._group.enable():
                raise RuntimeError("fold enable failed")
        except Exception:
            try:
                self._group.close()
            except Exception as exc:
                self.get_logger().warn("startup cleanup: %s" % exc)
            raise

        self._feedback_pub = self.create_publisher(JointState, "feedback/joint_states", 10)
        self._status_pub = self.create_publisher(DiagnosticArray, "feedback/motor_status", 10)
        self._control_sub = self.create_subscription(
            JointState, "control/joint_states", self._on_control, 10
        )
        self.create_service(SetBool, "enable", self._on_enable)
        self.create_service(SetBool, "control_enable", self._on_control_enable)

        period = 1.0 / feedback_rate
        self._feedback_timer = self.create_timer(period, self._publish_feedback)
        self._status_timer = self.create_timer(1.0 / status_rate, self._publish_status)

        self.get_logger().info(
            "fold_ros ready module=%s can=%s joints=[%s] gate=%s"
            % (
                module_key,
                self._group.can_channel,
                ", ".join(self._joint_names),
                self._control_enabled,
            )
        )

    def _check_can_control(self) -> bool:
        if not self._control_enabled:
            if not self._control_gate_block_logged:
                self.get_logger().info(
                    "External control gate is closed, ignore control commands"
                )
                self._control_gate_block_logged = True
            return False
        self._control_gate_block_logged = False
        return True

    def _publish_status(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        now_ns = time.time_ns()
        fault_fields = (
            "undervoltage", "motor_overtemp", "driver_overcurrent",
            "driver_overtemp", "collision_tripped", "driver_error", "stall_tripped",
        )
        numeric_fields = (
            "bus_voltage_v", "driver_temp_deg", "motor_temp_deg", "bus_current", "status_raw",
        )
        for index in range(self._group.joint_count):
            motor = self._group.motor(index)
            status = DiagnosticStatus()
            status.name = self._joint_names[index]
            status.hardware_id = "%s:%s" % (motor.can_channel, motor.node_id)
            status.level = DiagnosticStatus.STALE
            status.message = "low-speed feedback unavailable"
            values = {
                name: "unknown" for name in ("enabled",) + fault_fields + numeric_fields
            }
            values["control_mode"] = "pos"
            feedback = motor.get_low_speed_feedback()
            if feedback is not None:
                age = (now_ns - feedback.timestamp_ns) / 1e9
                if feedback.timestamp_ns > 0 and 0.0 <= age <= self._status_timeout:
                    values["enabled"] = str(feedback.status.enabled).lower()
                    faults = []
                    for name in fault_fields:
                        active = getattr(feedback.status, name)
                        values[name] = str(active).lower()
                        if active:
                            faults.append(name)
                    for name in numeric_fields:
                        values[name] = str(getattr(feedback, name))
                    status.level = DiagnosticStatus.OK
                    status.message = "enabled" if feedback.status.enabled else "disabled"
                    if faults:
                        status.level = DiagnosticStatus.ERROR
                        status.message = ", ".join(faults)
                else:
                    status.message = "low-speed feedback timestamp invalid or expired"
            status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
            msg.status.append(status)
        self._status_pub.publish(msg)

    def _publish_feedback(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for index in range(self._group.joint_count):
            state = self._group.get_state(index)
            if state is None:
                continue
            msg.name.append(self._joint_names[index])
            msg.position.append(state.position_rad)
            msg.velocity.append(state.velocity_rad_s)
            msg.effort.append(state.torque_nm)

        if msg.name:
            self._feedback_pub.publish(msg)

    def _on_control(self, msg: JointState) -> None:
        if not self._check_can_control():
            return
        if not msg.name or not msg.position:
            self.get_logger().warn("control/joint_states ignored: empty name or position")
            return
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("control/joint_states ignored: name/position size mismatch")
            return
        if len(set(msg.name)) != len(msg.name):
            self.get_logger().warn("control/joint_states ignored: duplicate joint names")
            return
        if any(not math.isfinite(value) for value in msg.position):
            self.get_logger().warn("control/joint_states ignored: non-finite command")
            return

        for idx, name in enumerate(msg.name):
            index = self._joint_by_name.get(name)
            if index is None or index == ArmJointGroup.INVALID_JOINT_INDEX:
                self.get_logger().debug("skip unknown joint: %s" % name)
                continue

            position = float(msg.position[idx])
            mode = (
                TARGET_MODE_UNCHANGED
                if self._position_mode_sent.get(name)
                else TARGET_MODE_POSITION
            )
            motor = self._group.motor(index)
            if not motor.set_target(position, mode):
                self.get_logger().warn("set_target failed for %s" % name)
            elif mode == TARGET_MODE_POSITION:
                self._position_mode_sent[name] = True

    def _on_control_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._control_enabled = bool(request.data)
        self._control_gate_block_logged = False
        state = "opened" if request.data else "closed"
        response.success = True
        response.message = "External control gate %s" % state
        self.get_logger().info(response.message)
        return response

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        ok = self._group.enable() if request.data else self._group.disable()
        if not request.data:
            for name in self._position_mode_sent:
                self._position_mode_sent[name] = False
        response.success = ok
        response.message = "enabled" if request.data else "disabled"
        if not ok:
            response.message += " (failed)"
        return response

    def destroy_node(self) -> bool:
        try:
            self._group.close()
        except Exception as exc:
            self.get_logger().warn("shutdown cleanup: %s" % exc)
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FoldRosNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
