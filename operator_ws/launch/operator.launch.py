# Operator側起動スクリプト
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_operator_config(launch_dir: str) -> dict:
    config_path = os.path.join(launch_dir, "operator_config.yaml")
    if not os.path.isfile(config_path):
        return {}
    try:
        with open(config_path) as f:
            loaded = yaml.safe_load(f)
        return loaded if isinstance(loaded, dict) else {}
    except Exception:
        return {}


def _b(val: bool) -> str:
    return "true" if val else "false"


def _build_operator_actions(context):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(this_dir)
    cert = os.path.join(repo_root, "quic", "server.crt")

    if os.path.isfile(cert):
        tls_block = f'    link: {{\n      tls: {{\n        root_ca_certificate: "{cert}"\n      }}\n    }}\n'
        endpoints = '"quic/10.42.0.1:7447", "tcp/10.42.0.1:7447", "quic/100.114.200.30:7447", "tcp/100.114.200.30:7447"'
    else:
        tls_block = ""
        endpoints = '"tcp/10.42.0.1:7447", "tcp/100.114.200.30:7447"'

    ope_cfg_content = f"""\
{{
  mode: "client",
  connect: {{
    endpoints: [{endpoints}]
  }},
  scouting: {{ multicast: {{ enabled: false }} }},
  transport: {{
    shared_memory: {{ enabled: true }},
{tls_block}  }}
}}
"""
    fd, dynamic_ope_cfg = tempfile.mkstemp(prefix="zenoh_ope_dynamic_", suffix=".json5")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(ope_cfg_content)

    return [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
        SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", dynamic_ope_cfg),
        # 0 = ロボット側 router が現れるまで無限待機 (-1 はスキップ = 即クラッシュ)
        SetEnvironmentVariable("ZENOH_ROUTER_CHECK_ATTEMPTS", "0"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
    ]


def generate_launch_description():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    cfg = _load_operator_config(this_dir)

    # operator_config.yaml の値をデフォルトに使う
    default_use_foxglove = _b(cfg.get("use_foxglove", True))
    default_use_joy = _b(cfg.get("use_joy", True))
    default_use_audio = _b(cfg.get("use_audio", True))
    default_foxglove_port = str(cfg.get("foxglove_port", 8765))
    default_foxglove_address = str(cfg.get("foxglove_address", "0.0.0.0"))
    default_mic_device = str(cfg.get("ope_mic_device", ""))
    default_spk_device = str(cfg.get("ope_spk_device", ""))
    default_bitrate = str(cfg.get("audio_bitrate", 32000))

    use_foxglove = DeclareLaunchArgument("use_foxglove", default_value=default_use_foxglove)
    use_joy = DeclareLaunchArgument("use_joy", default_value=default_use_joy)
    use_audio = DeclareLaunchArgument("use_audio", default_value=default_use_audio)
    foxglove_port = DeclareLaunchArgument("foxglove_port", default_value=default_foxglove_port)
    foxglove_address = DeclareLaunchArgument(
        "foxglove_address", default_value=default_foxglove_address
    )
    ope_mic_device = DeclareLaunchArgument("ope_mic_device", default_value=default_mic_device)
    ope_spk_device = DeclareLaunchArgument("ope_spk_device", default_value=default_spk_device)
    bitrate = DeclareLaunchArgument("bitrate", default_value=default_bitrate)

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=IfCondition(LaunchConfiguration("use_joy")),
        respawn=True,
        respawn_delay=3.0,
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
        parameters=[
            {
                "port": LaunchConfiguration("foxglove_port"),
                "address": LaunchConfiguration("foxglove_address"),
            }
        ],
        respawn=True,
        respawn_delay=3.0,
    )

    audio_sender = Node(
        package="audio_bridge",
        executable="audio_sender",
        name="operator_audio_sender",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[
            {
                "topic": "/operator/audio",
                "device": LaunchConfiguration("ope_mic_device"),
                "bitrate": LaunchConfiguration("bitrate"),
            }
        ],
        respawn=True,
        respawn_delay=3.0,
    )

    audio_receiver = Node(
        package="audio_bridge",
        executable="audio_receiver",
        name="operator_audio_receiver",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[{"topic": "/robot/audio", "device": LaunchConfiguration("ope_spk_device")}],
        respawn=True,
        respawn_delay=3.0,
    )

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=_build_operator_actions))
    ld.add_action(use_foxglove)
    ld.add_action(use_joy)
    ld.add_action(use_audio)
    ld.add_action(foxglove_port)
    ld.add_action(foxglove_address)
    ld.add_action(ope_mic_device)
    ld.add_action(ope_spk_device)
    ld.add_action(bitrate)
    ld.add_action(joy_node)
    ld.add_action(foxglove_node)
    ld.add_action(audio_sender)
    ld.add_action(audio_receiver)

    return ld
