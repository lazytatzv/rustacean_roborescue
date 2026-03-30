import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    this_dir = os.path.dirname(__file__)
    repo_root = os.path.dirname(this_dir)
    zenoh_config_path = os.path.join(repo_root, "zenoh_ope.json5")
    zenoh_config_uri = f"file://{zenoh_config_path}"

    set_rmw = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    set_zenoh_uri = SetEnvironmentVariable("RMW_ZENOH_CONFIG_URI", zenoh_config_uri)

    operator_node = Node(
        package="gst_webrtc_operator",
        executable="operator_webrtc_node",
        name="operator_webrtc",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(set_rmw)
    ld.add_action(set_zenoh_uri)
    ld.add_action(operator_node)
    return ld
