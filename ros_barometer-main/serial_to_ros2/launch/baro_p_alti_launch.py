import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_params_file(raw_path: str, default_path: str) -> str:
    """
    Resolve user-provided params file robustly.

    This handles common operator inputs such as:
    - ./floor_estimate/ros_barometer-main/...
    - floor_estimate/ros_barometer-main/...
    - ~/floor_estimate/...
    """
    expanded = os.path.expandvars(os.path.expanduser(raw_path.strip()))
    home = os.path.expanduser("~")

    candidates = []
    if expanded:
        candidates.append(expanded)
        if not os.path.isabs(expanded):
            candidates.append(os.path.abspath(expanded))
            candidates.append(os.path.join(home, expanded.lstrip("./")))

            marker = "floor_estimate/"
            marker_idx = expanded.find(marker)
            if marker_idx != -1:
                candidates.append(os.path.join(home, expanded[marker_idx:]))

    for candidate in candidates:
        normalized = os.path.normpath(candidate)
        if os.path.isfile(normalized):
            return normalized

    return os.path.normpath(default_path)


def _launch_setup(context, share_dir: str):
    default_params_file = os.path.join(share_dir, "config", "esp32_serial_baro.yaml")
    requested = LaunchConfiguration("esp32_serial_baro_params_file").perform(context)
    resolved = _resolve_params_file(requested, default_params_file)

    if os.path.normpath(resolved) != os.path.normpath(default_params_file):
        status = f"[baro_p_alti_launch] Using params file: {resolved}"
    else:
        status = (
            "[baro_p_alti_launch] Requested params file not found, "
            f"fallback to default: {resolved}"
        )

    return [
        LogInfo(msg=status),
        Node(
            package="serial_to_ros2",
            executable="esp32_serial_baro",
            name="esp32_serial_baro",
            namespace="",
            parameters=[resolved],
            output="screen",
        ),
    ]


def generate_launch_description():
    share_dir = get_package_share_directory("serial_to_ros2")
    default_params_file = os.path.join(share_dir, "config", "esp32_serial_baro.yaml")

    esp32_serial_baro_params_file = DeclareLaunchArgument(
        "esp32_serial_baro_params_file",
        default_value=default_params_file,
        description="File path to the ROS2 parameters file to use",
    )

    return LaunchDescription(
        [
            esp32_serial_baro_params_file,
            OpaqueFunction(function=_launch_setup, kwargs={"share_dir": share_dir}),
        ]
    )
