import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


class DiagnosticArray:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.status = []


class DiagnosticStatus:
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


@pytest.fixture
def fold_node(monkeypatch):
    class StubNode:
        parameter_overrides = {}

        def __init__(self, name):
            self.parameters = {}
            self.descriptors = {}
            self.timers = []

        def declare_parameter(self, name, default, descriptor=None):
            self.parameters[name] = self.parameter_overrides.get(name, default)
            self.descriptors[name] = descriptor

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, *args):
            return SimpleNamespace(publish=lambda message: None)

        def create_subscription(self, *args):
            return SimpleNamespace()

        def create_service(self, *args):
            return SimpleNamespace()

        def create_timer(self, period, callback):
            self.timers.append((period, callback))
            return SimpleNamespace()

        def get_logger(self):
            return SimpleNamespace(info=lambda message: None, warn=lambda message: None)

    module_attributes = {
        "rclpy": {},
        "rclpy.node": {"Node": StubNode},
        "rclpy.executors": {"ExternalShutdownException": type("ExternalShutdownException", (Exception,), {})},
        "diagnostic_msgs": {},
        "diagnostic_msgs.msg": {
            "DiagnosticArray": DiagnosticArray,
            "DiagnosticStatus": DiagnosticStatus,
            "KeyValue": SimpleNamespace,
        },
        "rcl_interfaces": {},
        "rcl_interfaces.msg": {"ParameterDescriptor": SimpleNamespace},
        "ament_index_python": {},
        "ament_index_python.packages": {
            "get_package_share_directory": lambda name: "unused"
        },
        "sensor_msgs": {},
        "sensor_msgs.msg": {"JointState": SimpleNamespace},
        "std_srvs": {},
        "std_srvs.srv": {
            "SetBool": SimpleNamespace(Request=object, Response=object)
        },
        "agx_motor_sdk": {},
        "agx_motor_sdk.mechanism": {
            "JointGroup": SimpleNamespace(INVALID_JOINT_INDEX=-1)
        },
    }
    for name, attributes in module_attributes.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[1] / "fold_ros" / "fold_node.py"
    spec = importlib.util.spec_from_file_location("fold_test_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.time, "time_ns", lambda: 10_000_000_000)
    return module


def make_control_node(module):
    calls = []
    node = module.FoldRosNode.__new__(module.FoldRosNode)
    node._control_enabled = True
    node._control_gate_block_logged = False
    node._joint_names = ["joint1", "joint2"]
    node._joint_by_name = {"joint1": 0, "joint2": 1}
    node._position_mode_sent = {name: False for name in node._joint_names}

    def motor(index):
        def set_target(position, mode):
            calls.append((index, position, mode))
            return True
        return SimpleNamespace(set_target=set_target)

    node._group = SimpleNamespace(motor=motor)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: None, warn=lambda message: None,
        debug=lambda message: None,
    )
    return node, calls


@pytest.mark.parametrize("position", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_command_rejected_atomically(fold_node, position):
    node, calls = make_control_node(fold_node)
    node._on_control(SimpleNamespace(name=["joint1", "joint2"], position=[0.2, position]))
    assert calls == []
    assert not any(node._position_mode_sent.values())


def test_duplicate_command_rejected(fold_node):
    node, calls = make_control_node(fold_node)
    node._on_control(SimpleNamespace(name=["joint1", "joint1"], position=[0.2, 0.3]))
    assert calls == []


def test_position_mode_sent_once(fold_node):
    node, calls = make_control_node(fold_node)
    for position in (0.2, 0.3):
        node._on_control(SimpleNamespace(name=["joint1"], position=[position]))
    assert calls == [(0, 0.2, 1), (0, 0.3, 0xFF)]


def test_closed_gate_blocks_command(fold_node):
    node, calls = make_control_node(fold_node)
    node._control_enabled = False
    node._on_control(SimpleNamespace(name=["joint1"], position=[0.2]))
    assert calls == []


def make_feedback(enabled=True, timestamp_ns=9_000_000_000):
    return SimpleNamespace(
        status=SimpleNamespace(
            enabled=enabled, undervoltage=False, motor_overtemp=False,
            driver_overcurrent=False, driver_overtemp=False, collision_tripped=False,
            driver_error=False, stall_tripped=False,
        ),
        timestamp_ns=timestamp_ns, bus_voltage_v=48.0, driver_temp_deg=35,
        motor_temp_deg=40, bus_current=1.2, status_raw=64 if enabled else 0,
    )


def publish_status(module, feedbacks):
    node = module.FoldRosNode.__new__(module.FoldRosNode)
    motors = [
        SimpleNamespace(
            can_channel="can%d" % index, node_id=index + 1,
            get_low_speed_feedback=lambda feedback=feedback: feedback,
        )
        for index, feedback in enumerate(feedbacks)
    ]
    node._group = SimpleNamespace(joint_count=len(motors), motor=motors.__getitem__)
    node._joint_names = ["joint%d" % (index + 1) for index in range(len(motors))]
    node._status_timeout = 2.0
    published = []
    node._status_pub = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: "ros_stamp")
    )
    node._publish_status()
    assert len(published) == 1
    assert published[0].header.stamp == "ros_stamp"
    for status in published[0].status:
        assert {entry.key for entry in status.values} == {
            "enabled", "undervoltage", "motor_overtemp", "driver_overcurrent",
            "driver_overtemp", "collision_tripped", "driver_error", "stall_tripped",
            "bus_voltage_v", "driver_temp_deg", "motor_temp_deg", "bus_current",
            "status_raw", "control_mode",
        }
    return published[0].status


def test_enabled_disabled_and_unavailable_status(fold_node):
    statuses = publish_status(fold_node, [make_feedback(), make_feedback(False), None])
    assert [status.name for status in statuses] == ["joint1", "joint2", "joint3"]
    assert [status.hardware_id for status in statuses] == ["can0:1", "can1:2", "can2:3"]
    assert [status.level for status in statuses] == [0, 0, 3]
    assert [status.values[0].value for status in statuses] == ["true", "false", "unknown"]


@pytest.mark.parametrize("timestamp_ns", [0, -1, 7_999_999_999, 10_000_000_001])
def test_expired_or_invalid_feedback_is_unknown(fold_node, timestamp_ns):
    status = publish_status(fold_node, [make_feedback(timestamp_ns=timestamp_ns)])[0]
    assert status.level == DiagnosticStatus.STALE
    assert status.values[0].value == "unknown"
    assert all(
        entry.value == "unknown" for entry in status.values if entry.key != "control_mode"
    )


def test_timeout_boundary_and_status_fields(fold_node):
    status = publish_status(fold_node, [make_feedback(timestamp_ns=8_000_000_000)])[0]
    values = {value.key: value.value for value in status.values}
    assert status.level == DiagnosticStatus.OK
    assert values["bus_voltage_v"] == "48.0"
    assert values["driver_temp_deg"] == "35"
    assert values["motor_temp_deg"] == "40"
    assert values["bus_current"] == "1.2"
    assert values["status_raw"] == "64"
    assert values["control_mode"] == "pos"


@pytest.mark.parametrize("fault", [
    "undervoltage", "motor_overtemp", "driver_overcurrent", "driver_overtemp",
    "collision_tripped", "driver_error", "stall_tripped",
])
def test_fault_feedback_reports_error(fold_node, fault):
    feedback = make_feedback()
    setattr(feedback.status, fault, True)
    status = publish_status(fold_node, [feedback])[0]
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == fault


def configure_startup(module, monkeypatch):
    closed = []
    group = SimpleNamespace(
        joint_names=["joint1"], joint_count=1, index_of=lambda name: 0,
        can_channel="can0", connect=lambda: True, configure=lambda: True,
        enable_intervals=[], close=lambda: closed.append(True),
    )
    group.enable = lambda interval=0.0: group.enable_intervals.append(interval) or True
    monkeypatch.setattr(module, "resolve_yaml_path", lambda value: Path("fold.yaml"))
    monkeypatch.setattr(module.JointGroup, "from_yaml", lambda *args: group, raising=False)
    return group, closed


@pytest.mark.parametrize("method", ["connect", "configure", "enable"])
def test_startup_failure_closes_group(fold_node, monkeypatch, method):
    group, closed = configure_startup(fold_node, monkeypatch)
    setattr(group, method, lambda *args: False)
    with pytest.raises(RuntimeError):
        fold_node.FoldRosNode()
    assert closed == [True]


@pytest.mark.parametrize("parameter", ["feedback_rate", "status_rate", "status_timeout"])
@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_rates_rejected_before_connect(fold_node, parameter, value):
    fold_node.FoldRosNode.__bases__[0].parameter_overrides = {parameter: value}
    with pytest.raises(ValueError, match="finite and > 0"):
        fold_node.FoldRosNode()


def test_startup_parameters_readonly_and_default_rates(fold_node, monkeypatch):
    configure_startup(fold_node, monkeypatch)
    node = fold_node.FoldRosNode()
    assert len(node.descriptors) == 9
    assert all(descriptor.read_only for descriptor in node.descriptors.values())
    assert [period for period, _ in node.timers] == [0.01, 1.0 / 30.0]


def test_enable_uses_frame_interval(fold_node, monkeypatch):
    group, _ = configure_startup(fold_node, monkeypatch)
    node = fold_node.FoldRosNode()
    assert group.enable_intervals == [0.1]
    disable_intervals = []
    group.disable = lambda interval=0.0: disable_intervals.append(interval) or True
    for data in (True, False):
        node._on_enable(SimpleNamespace(data=data), SimpleNamespace())
    assert group.enable_intervals == [0.1, 0.1]
    assert disable_intervals == [0.1]


def test_main_shuts_down_after_construction_failure(fold_node, monkeypatch):
    closed = []
    monkeypatch.setattr(fold_node.rclpy, "init", lambda **kwargs: None, raising=False)
    monkeypatch.setattr(fold_node.rclpy, "try_shutdown", lambda: closed.append(True), raising=False)
    monkeypatch.setattr(fold_node, "FoldRosNode", lambda: (_ for _ in ()).throw(RuntimeError("startup")))
    with pytest.raises(RuntimeError, match="startup"):
        fold_node.main()
    assert closed == [True]


RB200_BIT_POSITIONS = {
    "ready_to_switch_on": 0, "switched_on": 1, "operation_enabled": 2, "fault": 3,
    "voltage_enabled": 4, "quick_stop": 5, "switch_on_disabled": 6, "warning": 7,
    "remote": 9, "target_reached": 10, "internal_limit_active": 11, "homing_complete": 15,
}
RB200_KEYS = {
    "enabled", "cia402_state", "ready_to_switch_on", "switched_on", "fault", "voltage_enabled",
    "switch_on_disabled", "warning", "remote", "target_reached", "internal_limit_active",
    "homing_complete", "quick_stop_active", "error_code", "status_word", "control_mode",
}


def make_rb200_feedback(status_word, timestamp_ns=9_000_000_000, error_code=0):
    return SimpleNamespace(
        status_word=status_word, error_code=error_code, tpdo2_count=1,
        status=SimpleNamespace(**{
            name: bool(status_word >> bit & 1) for name, bit in RB200_BIT_POSITIONS.items()
        }),
        timestamp_ns=timestamp_ns,
    )


def publish_rb200_status(module, feedbacks):
    node = module.FoldRosNode.__new__(module.FoldRosNode)
    motors = [
        SimpleNamespace(
            can_channel="can0", node_id=index + 1, protocol="rb200",
            get_low_speed_feedback=lambda feedback=feedback: feedback,
        )
        for index, feedback in enumerate(feedbacks)
    ]
    node._group = SimpleNamespace(joint_count=len(motors), motor=motors.__getitem__)
    node._joint_names = ["joint%d" % (index + 1) for index in range(len(motors))]
    node._status_timeout = 2.0
    node._control_mode = "pos"
    published = []
    node._status_pub = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: "ros_stamp")
    )
    node._publish_status()
    return published[0].status


def test_rb200_status_word_published(fold_node):
    statuses = publish_rb200_status(fold_node, [
        make_rb200_feedback(0x0637), make_rb200_feedback(0x0631),
        make_rb200_feedback(0x0638), make_rb200_feedback(0x0637, 7_000_000_000), None,
    ])
    values = [{entry.key: entry.value for entry in status.values} for status in statuses]
    assert set(values[0]) == RB200_KEYS
    assert [v["status_word"] for v in values] == [
        "0x0637", "0x0631", "0x0638", "unknown", "unknown",
    ]
    assert [v["enabled"] for v in values] == ["true", "false", "false", "unknown", "unknown"]
    assert [status.level for status in statuses] == [
        DiagnosticStatus.OK, DiagnosticStatus.OK, DiagnosticStatus.ERROR,
        DiagnosticStatus.STALE, DiagnosticStatus.STALE,
    ]
    assert [status.message for status in statuses[:3]] == ["operation_enabled", "ready_to_switch_on", "fault, error_code 0x0000"]


def test_rb200_cia402_state_decoding(fold_node):
    cia402_state = fold_node.cia402_state

    assert cia402_state(0x0000) == "not_ready_to_switch_on"
    assert cia402_state(0x0040) == "switch_on_disabled"
    assert cia402_state(0x0021) == "ready_to_switch_on"
    assert cia402_state(0x0023) == "switched_on"
    assert cia402_state(0x1637) == "operation_enabled"
    assert cia402_state(0x0007) == "quick_stop_active"
    assert cia402_state(0x000F) == "fault_reaction_active"
    assert cia402_state(0x0008) == "fault"


def test_rb200_status_bits_and_error_code(fold_node):
    statuses = publish_rb200_status(fold_node, [
        make_rb200_feedback(0x1637), make_rb200_feedback(0x0638, error_code=0x8130),
        make_rb200_feedback(0x00B7), make_rb200_feedback(0x0017),
    ])
    values = [{item.key: item.value for item in status.values} for status in statuses]

    assert values[0]["cia402_state"] == "operation_enabled"
    assert values[0]["target_reached"] == "true"
    assert values[0]["remote"] == "true"
    assert values[0]["quick_stop_active"] == "false"
    assert values[1]["error_code"] == "0x8130"
    assert statuses[1].message == "fault, error_code 0x8130"
    assert statuses[2].level == DiagnosticStatus.WARN
    assert values[2]["warning"] == "true"
    assert values[3]["cia402_state"] == "quick_stop_active"
    assert values[3]["quick_stop_active"] == "true"
