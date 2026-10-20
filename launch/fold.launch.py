import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    overrides = {}

    yaml_path = LaunchConfiguration("yaml_path").perform(context).strip()
    if yaml_path:
        overrides["yaml_path"] = yaml_path

    module_key = LaunchConfiguration("module_key").perform(context).strip()
    if module_key:
        overrides["module_key"] = module_key

    can_channel = LaunchConfiguration("can_channel").perform(context).strip()
    if can_channel:
        overrides["can_channel"] = can_channel

    for name in ("control_enabled", "auto_configure", "auto_enable"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value == "true"
    for name in ("feedback_rate", "status_rate", "status_timeout"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = float(value)

    parameters = [params_file]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="fold_ros",
            executable="fold_node",
            name="fold_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("fold_ros")
    default_params = os.path.join(share, "config", "fold_ros.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="fold",
                description="ROS namespace (default: fold).",
            ),
            DeclareLaunchArgument(
                "yaml_path",
                default_value="",
                description="Fold YAML path (empty = use params_file).",
            ),
            DeclareLaunchArgument(
                "can_channel",
                default_value="",
                description="Override CAN channel (empty = use params_file / fold YAML).",
            ),
            DeclareLaunchArgument(
                "module_key",
                default_value="",
                description="YAML module key (empty = use params_file).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="fold_ros parameter file",
            ),
            DeclareLaunchArgument(
                "control_enabled",
                default_value="",
                choices=["", "true", "false"],
                description="Control gate (empty = use params_file).",
            ),
            *[
                DeclareLaunchArgument(
                    name, default_value="", choices=["", "true", "false"],
                    description="Startup setting (empty = use params_file).",
                )
                for name in ("auto_configure", "auto_enable")
            ],
            *[
                DeclareLaunchArgument(
                    name, default_value="",
                    description="Positive finite value (empty = use params_file).",
                )
                for name in ("feedback_rate", "status_rate", "status_timeout")
            ],
            OpaqueFunction(function=_launch_setup),
        ]
    )
