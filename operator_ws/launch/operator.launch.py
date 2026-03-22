# Operator側起動スクリプト
# 操縦側で立ち上げるのはこれだけで完結させること

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_foxglove = LaunchConfiguration("use_foxglove", default="true")

    # Set RMW implementation and Zenoh config URI for all nodes launched here
    # Compute zenoh config path relative to this file: ../zenoh_ope.json5
    this_dir = os.path.dirname(__file__)
    repo_root = os.path.dirname(this_dir)
    zenoh_config_path = os.path.join(repo_root, "zenoh_ope.json5")
    zenoh_config_uri = f"file://{zenoh_config_path}"

    set_rmw = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    set_zenoh_uri = SetEnvironmentVariable("RMW_ZENOH_CONFIG_URI", zenoh_config_uri)

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "address": "127.0.0.1",
                "send_buffer_limit": 10000000,
                "max_qos_depth": 10,
                "num_threads": 0,
                "use_compression": False,
            }
        ],
    )

    ld = LaunchDescription()
    # environment first
    ld.add_action(set_rmw)
    ld.add_action(set_zenoh_uri)
    ld.add_action(
        DeclareLaunchArgument(
            "use_foxglove", default_value="true", description="Start foxglove_bridge"
        )
    )
    ld.add_action(joy_node)
    # foxglove_bridge is optional; keep it enabled by default
    ld.add_action(foxglove_node)

    return ld
