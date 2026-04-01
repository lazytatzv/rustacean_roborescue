import os
import shlex

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    # bringupパッケージのshareディレクトリから設定ファイルの絶対パスを動的に取得
    bringup_dir = get_package_share_directory("bringup")
    # zenohd (router daemon) 用の設定: mode=router, listen tcp/0.0.0.0:7447
    zenoh_router_config = os.path.join(bringup_dir, "config", "zenoh_router.json5")
    # ロボット側 ROS2 ノード用の設定: mode=client, connect 127.0.0.1:7447
    # zenohd と同じ router config を使うとポート競合するため別ファイルを使う
    zenoh_robot_config = os.path.join(bringup_dir, "config", "zenoh_robot.json5")

    # Nix環境では rmw_zenoh_cpp の .so が LD_LIBRARY_PATH に
    # 自動で入らないケースがあるため、起動時に補完する。
    zenoh_lib_paths = []
    try:
        zenoh_prefix = get_package_prefix("rmw_zenoh_cpp")
        for d in ("lib", "lib64"):
            candidate = os.path.join(zenoh_prefix, d)
            if os.path.isdir(candidate):
                zenoh_lib_paths.append(candidate)
    except PackageNotFoundError:
        zenoh_lib_paths = []

    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_parts = [p for p in zenoh_lib_paths if p]
    if current_ld_library_path:
        ld_parts.append(current_ld_library_path)
    patched_ld_library_path = ":".join(ld_parts)

    router_cmd = (
        "set -e; "
        "if ss -ltn '( sport = :7447 )' | grep -q ':7447'; then "
        "  if ss -ltnp '( sport = :7447 )' 2>/dev/null | grep -q 'zenohd'; then "
        "    echo '[network.launch] zenoh router already running on :7447, skip local zenohd'; "
        "    exit 0; "
        "  fi; "
        "  echo '[network.launch] :7447 is busy by non-zenoh process, starting zenohd on :17447'; "
        "  _tmp_cfg=$(mktemp); "
        "  trap 'rm -f \"$_tmp_cfg\"' EXIT; "
        "  sed 's#tcp/0\\.0\\.0\\.0:7447#tcp/0.0.0.0:17447#g' "
        + shlex.quote(zenoh_router_config)
        + " > \"$_tmp_cfg\"; "
        "  exec zenohd --config \"$_tmp_cfg\"; "
        "else "
        "  exec zenohd --config "
        + shlex.quote(zenoh_router_config)
        + "; "
        "fi"
    )

    actions = [
        # ==========================================
        # 1. RMW (ミドルウェア) の環境変数を強制上書き
        # ==========================================
        # このLaunchファイル（およびこれをincludeした親Launch）から
        # 起動されるすべてのROS 2ノードは、強制的にZenohプロトコルで通信します。
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
        # ROS2ノードは zenohd (client として localhost:7447 に接続) を使う。
        SetEnvironmentVariable("RMW_ZENOH_CONFIG_URI", f"file://{zenoh_robot_config}"),
        # ==========================================
        # 2. Zenohルーター (zenohd) のデーモン起動
        # ==========================================
        # QUIC でリッスンし、オペレータ PC からの接続を受け付ける。
        # SHM は zenohd 側でも有効にする必要があるため --config で渡す。
        ExecuteProcess(
            cmd=["bash", "-lc", router_cmd],
            output="screen",
            respawn=False,
            name="zenoh_router_daemon",
        ),
    ]

    if patched_ld_library_path:
        actions.insert(
            0, SetEnvironmentVariable("LD_LIBRARY_PATH", patched_ld_library_path)
        )

    return LaunchDescription(actions)
